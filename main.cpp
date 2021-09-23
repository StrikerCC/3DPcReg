//#pragma once
#include <vector>
#include <iostream>

#include "PointCloudReader.h"
//#include "Registration.h"
#include "Registration_mix.h"
#include "Evaluation.h"

int main() {
    std::cout << "Point cloud Reg testing" << std::endl;
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

    Evaluation eval = Evaluation();
//    Registration reg = Registration();

    for (int i = 0; i < reader.get_length(); ++i) {
        float time;
        register_result error;
        sourceTargetAndPose pc_pair = reader.getModelAndOneFrame(i);

        verification::statistic_reg statistics_ = eval.register_ransac_icp(*pc_pair.src, *pc_pair.tgt, pc_pair.pose);
        statistics_.src = std::to_string(i);
        eval.recordError(statistics_);

        std::cout << "Time total                " <<  statistics_.time_total << std::endl;
    }
    eval.save("/home/cheng/proj/3d/3DPcReg/snapshot/00000.json");
    return 0;
}
