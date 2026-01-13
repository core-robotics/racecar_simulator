import csv
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


def interpolate_polyline(points, step_m):
    if not points:
        return []
    if len(points) == 1:
        return points[:]
    step_m = max(step_m, 1e-6)

    out = [points[0]]
    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            continue
        n = max(1, int(math.ceil(dist / step_m)))
        out.extend((x0 + (i / n) * dx, y0 + (i / n) * dy) for i in range(1, n + 1))
    return out


def write_centerline_csv(path, points):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x_m", "y_m"])
        w.writerows(((f"{x:.6f}", f"{y:.6f}") for x, y in points))


class CenterlineRecorder(Node):
    def __init__(self):
        super().__init__("centerline_recorder")
        self.declare_parameter("topic", "/clicked_point")
        self.declare_parameter("output_csv", "centerline.csv")
        self.declare_parameter("step_m", 0.10)

        self.topic = self.get_parameter("topic").value
        self.output_csv = self.get_parameter("output_csv").value
        self.step_m = float(self.get_parameter("step_m").value)

        self.clicked_points = []
        self.create_subscription(PointStamped, self.topic, self.on_clicked_point, 10)

    def on_clicked_point(self, msg: PointStamped) -> None:
        self.clicked_points.append((float(msg.point.x), float(msg.point.y)))

    def save(self) -> None:
        centerline = interpolate_polyline(self.clicked_points, self.step_m)
        if centerline:
            write_centerline_csv(self.output_csv, centerline)


def main():
    rclpy.init()
    node = CenterlineRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
