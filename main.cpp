//#pragma once
#include <vector>
#include <iostream>

#include "PointCloudReader.h"
//#include "Registration.h"
//#include "Registration_mix.h"
#include "Evaluation.h"
#include "statistics.h"

int main() {
    std::cout << "Point cloud Reg testing" << std::endl;
    //////////////////////////////////////////////
    std::cout << "Start reading" << std::endl;

//    PointCloudReader reader = PointCloudReader();
//
//    reader.loadModel("/home/cheng/proj/3d/3DPcReg/data/3D_model_head.pcd");
//    reader.loadOneFrame("/home/cheng/proj/3d/3DPcReg/data/000000.pcd");
//    reader.loadOnePose("/home/cheng/proj/3d/3DPcReg/data/000000.txt");

    Evaluation eval = Evaluation();
    PointCloudReaderFromJson reader = PointCloudReaderFromJson();

    //    reader.loadDataFromRootDirAndJson("/home/cheng/proj/3d/3DPcReg/data/data.json");
    reader.loadDataFromRootDirAndJson("/home/cheng/proj/3d/TEASER-plusplus/",
                                      "/home/cheng/proj/3d/TEASER-plusplus/data/human_data/data.json");


//    for (int i = 0; i < reader.get_length(); ++i) {
    for (int i = 0; i < 1; ++i) {
        sourceTargetAndPose pc_pair = reader.getModelAndOneFrame(i);    // read a pair of pc
        statistics statistics_ = eval.register_ransac_icp(*pc_pair.src, *pc_pair.tgt, pc_pair.pose);   // multi-level reg

        // makeup some input data info
        statistics_.src = reader.getSourcePath(i);
        statistics_.voxel_size = reader.getVoxelSize(i);
        statistics_.noise_src = reader.getSourceNoise(i);
        eval.recordstatistics(statistics_);
        std::cout << "Time total                " <<  statistics_.time << std::endl;
    }

     std::string output_dir = "/home/cheng/proj/3d/3DPcReg/snapshot/";
//    std::string output_dir = "/home/cheng_chen/proj/3d/3DPcReg/snapshot/";

    eval.save(output_dir);

    return 0;
}
