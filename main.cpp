#include <vector>
#include <iostream>
#include "pc_reg.h"

int main() {
    std::cout << "Point cloud Reg testing" << std::endl;
    pc_reg reg = pc_reg();
//    reg.vis();
//    reg.showBall();
//    reg.LoadModel("/home/cheng/proj/3d/3DPcReg/data/3D_model.pcd");
//    reg.LoadModel("/home/cheng/proj/3d/3DPcReg/data/3D_model_bag.pcd");
    reg.LoadModel("/home/cheng/proj/3d/3DPcReg/data/3D_model_head.pcd");
    float time_read_total {0}, time_reg_total {0.0};
    for (int i=0; i<10000; i++) {
        float time_read {0.0}, time_reg {0.0};

        clock_t clock_start = clock();
        reg.TakeNewFrame("/home/cheng/proj/3d/3DPcReg/data/000000.pcd");
        time_read = ((float) (clock()-clock_start)) / ((float) CLOCKS_PER_SEC);
        time_read_total += time_read;

        clock_start = clock();
        reg.Register();
        time_reg = ((float) (clock()-clock_start)) / ((float) CLOCKS_PER_SEC);
        time_reg_total += time_reg;

        std::cout << "Frame # " << reg.get_num_frame() << " read takes " << time_read << " seconds " << ", reg takes " << time_reg << " seconds" << std::endl;
        std::cout << "Frame # " << reg.get_num_frame() << " read averg " << time_read_total/float (i+1) << " seconds" << ", reg takes " << time_reg_total/float (i+1) << " seconds" << std::endl;
//        reg.DrawReg();
    }

    std::cout << "reg tf" << std::endl;
    std::cout << reg.pose_current << std::endl;
    return 0;
}