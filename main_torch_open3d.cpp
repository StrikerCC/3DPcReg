#include <iostream>
#include "torch/torch.h"
#include "open3d/Open3D.h"
#include "torch/script.h"

//int main() {
//    open3d::geometry::PointCloud pc = open3d::geometry::PointCloud();
//    std::cout << &pc << std::endl;
//
//    std::cout << "Hello, World!" << std::endl;
//    torch::Tensor tensor = torch::rand({2, 3});
//    tensor = tensor.cuda();
//    std::cout << tensor << std::endl;
//    std::cout << tensor.sizes() << std::endl;
//    return 0;
//}

int main() {
    torch::DeviceType device_type;
    if (torch::cuda::is_available()) {
        device_type = torch::kCUDA;
    }
    else {
        device_type = torch::kCPU;
    }
    torch::Device device(device_type);


    std::string model_pb = "../weights/model_best_recall.pth";
    auto module = torch::jit::load(model_pb, device);
    module.to(at::kCUDA);
}