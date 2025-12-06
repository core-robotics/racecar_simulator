// src/nn_aukf_node.cpp

#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <torch/script.h>
#include <torch/torch.h>

// Convert tensor shape to string like "[1, 5, 5]"
std::string tensor_shape_str(const torch::Tensor &t)
{
  auto sizes = t.sizes();
  std::ostringstream oss;
  oss << "[";
  for (int64_t i = 0; i < sizes.size(); ++i)
  {
    if (i > 0)
    {
      oss << ", ";
    }
    oss << sizes[i];
  }
  oss << "]";
  return oss.str();
}

class NNAukfNode : public rclcpp::Node
{
public:
  NNAukfNode()
      : Node("nn_aukf"),
        module_loaded_(false),
        use_gpu_(false),
        device_(torch::kCPU)
  {
    // Parameters
    model_path_ = this->declare_parameter<std::string>("model_path", "");
    bool use_gpu_param =this->declare_parameter<bool>("use_gpu", true);
    double hz =this->declare_parameter<double>("timer_period", 10.0);
    double period_sec = 1.0 / hz;

    std::cout << "[NN_AUKF] model_path  = " << model_path_ << std::endl;
    std::cout << "[NN_AUKF] use_gpu    = " << std::boolalpha << use_gpu_param << std::endl;
    std::cout << "[NN_AUKF] timer Hz   = " << hz
              << " (period = " << period_sec << " s)" << std::endl;

    // Select device
    if (use_gpu_param && torch::cuda::is_available())
    {
      use_gpu_ = true;
      device_ = torch::Device(torch::kCUDA);
      std::cout << "Using GPU."
                << std::endl;
    }
    else
    {
      use_gpu_ = false;
      device_ = torch::Device(torch::kCPU);
      std::cout << "No GPU! "
                << std::endl;
    }

    // Load model 
    load_model();

    // Timer
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(std::chrono::duration<double>(period_sec), std::bind(&NNAukfNode::timer_callback, this));
  }

private:
  void load_model()
  {
    std::cout << "[NN_AUKF] Loading TorchScript model from: "
              << model_path_ << std::endl;

    module_ = torch::jit::load(model_path_);
    module_.eval();
    module_.to(device_);

    module_loaded_ = true;
    std::cout << "[NN_AUKF] Model loaded successfully on "
              << (use_gpu_ ? "GPU." : "CPU.") << std::endl;
  }

  void timer_callback()
  {
    if (!module_loaded_)
    {
      std::cerr << "[NN_AUKF] timer_callback called but model not loaded."
                << std::endl;
      return;
    }

    // Random input tensors
    auto opts = torch::TensorOptions().dtype(torch::kFloat32).device(device_);

    //    x:           [1, 5]
    //    u:           [1, 2]
    //    z:           [1, 4]
    //    P:           [1, 5, 5]
    //    innov_hist:  [1, 30, 9]
    //    R_diag_prev: [1, 4]
    torch::Tensor x = torch::rand({1, 5}, opts);
    torch::Tensor u = torch::rand({1, 2}, opts);
    torch::Tensor z = torch::rand({1, 4}, opts);
    torch::Tensor P = torch::rand({1, 5, 5}, opts);
    torch::Tensor innov_hist = torch::rand({1, 30, 9}, opts);
    torch::Tensor R_diag_prev = torch::rand({1, 4}, opts);

    // std::cout << "\n[NN_AUKF] ===== New Inference on "
    //           << (use_gpu_ ? "GPU" : "CPU") << " =====" << std::endl;

    // std::cout << "[NN_AUKF] Input tensor shapes:" << std::endl;
    // std::cout << "  x          = " << tensor_shape_str(x) << std::endl;
    // std::cout << "  u          = " << tensor_shape_str(u) << std::endl;
    // std::cout << "  z          = " << tensor_shape_str(z) << std::endl;
    // std::cout << "  P          = " << tensor_shape_str(P) << std::endl;
    // std::cout << "  innov_hist = " << tensor_shape_str(innov_hist) << std::endl;
    // std::cout << "  R_diag_prev= " << tensor_shape_str(R_diag_prev) << std::endl;

    torch::NoGradGuard no_grad;

    std::vector<torch::jit::IValue> inputs;
    inputs.reserve(6);
    inputs.push_back(x);
    inputs.push_back(u);
    inputs.push_back(z);
    inputs.push_back(P);
    inputs.push_back(innov_hist);
    inputs.push_back(R_diag_prev);

    c10::IValue out_iv = module_.forward(inputs);
    auto out_tuple = out_iv.toTuple();

    if (out_tuple->elements().size() != 4)
    {
      std::cerr << "[NN_AUKF] Unexpected number of outputs: "
                << out_tuple->elements().size() << std::endl;
      return;
    }

    torch::Tensor x_pred = out_tuple->elements()[0].toTensor();
    torch::Tensor P_pred = out_tuple->elements()[1].toTensor();
    torch::Tensor innov = out_tuple->elements()[2].toTensor();
    torch::Tensor R_diag = out_tuple->elements()[3].toTensor();

    std::cout << "[NN_AUKF] Output tensor shapes:" << std::endl;
    std::cout << "  x_pred = " << tensor_shape_str(x_pred) << std::endl;
    std::cout << "  P_pred = " << tensor_shape_str(P_pred) << std::endl;
    std::cout << "  innov  = " << tensor_shape_str(innov) << std::endl;
    std::cout << "  R_diag = " << tensor_shape_str(R_diag) << std::endl;

    // Print a few sample values
    std::cout << "[NN_AUKF] Sample values:" << std::endl;
    std::cout << "  x[0]          = "
              << x.flatten()[0].item<float>() << std::endl;
    std::cout << "  x_pred[0]     = "
              << x_pred.flatten()[0].item<float>() << std::endl;
    std::cout << "  innov[0]      = "
              << innov.flatten()[0].item<float>() << std::endl;
    std::cout << "  R_diag[0]     = "
              << R_diag.flatten()[0].item<float>() << std::endl;

    std::cout << "[NN_AUKF] Inference finished on "
              << (use_gpu_ ? "GPU." : "CPU.") << std::endl;
  }

  // Members
  std::string model_path_;
  bool module_loaded_;
  bool use_gpu_;
  torch::Device device_;
  torch::jit::script::Module module_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NNAukfNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
