#include <vector>
#include <iostream>

#include "PointCloudReader.h"
#include "Registration.h"
#include "Registration_mix.h"

int main() {
    /*
    std::cout << "Point cloud Reg testing" << std::endl;
    Registration_mix reg = Registration_mix();
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

        std::cout << "Frame # " << reg.get_num_frame() << " loadModel takes " << time_read << " seconds " << ", reg takes " << time_reg << " seconds" << std::endl;
        std::cout << "Frame # " << reg.get_num_frame() << " loadModel averg " << time_read_total/float (i+1) << " seconds" << ", reg takes " << time_reg_total/float (i+1) << " seconds" << std::endl;
//        reg.DrawReg();
    }

    std::cout << "reg tf" << std::endl;
    std::cout << reg.pose_current << std::endl;

     */
    //////////////////////////////////////////////
    std::cout << "Start reading" << std::endl;

    clock_t clock_start = clock();

    PointCloudReaderFromJson reader = PointCloudReaderFromJson();
    reader.loadJson("/home/cheng/proj/3d/3DPcReg/data/data.json");

//    PointCloudReader reader = PointCloudReader();
//
//    reader.loadModel("/home/cheng/proj/3d/3DPcReg/data/3D_model_head.pcd");
//    reader.loadOneFrame("/home/cheng/proj/3d/3DPcReg/data/000000.pcd");
//    reader.loadOnePose("/home/cheng/proj/3d/3DPcReg/data/000000.txt");

    Registration reg = Registration();

    for (int i = 0; i < reader.get_length(); ++i) {
        sourceTargetAndPose pc_pair = reader.getModelAndOneFrame(i);

        float time = reg.register_ransac_icp(*pc_pair.src, *pc_pair.tgt, pc_pair.pose);
        std::cout << "Time total                " <<  time << std::endl;
    }
    return 0;
}
