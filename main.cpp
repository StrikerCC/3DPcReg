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

    Evaluation eval = Evaluation();
    PointCloudReaderFromJson reader = PointCloudReaderFromJson();

    reader.loadDataFromRootDirAndJson("/home/cheng_chen/3d_projects/Teaser-plusplus-testing/", "/home/cheng_chen/3d_projects/Teaser-plusplus-testing/data/human_data_1152/data.json");
    // reader.loadDataFromRootDirAndJson("/home/cheng/proj/3d/TEASER-plusplus/", "/home/cheng_chen/3d_projects/Teaser-plusplus-testing/data/human_data_1152/data.json");

    for (int i = 0; i < reader.get_length(); ++i) {
    // for (int i = 0; i < 100; ++i) {
        sourceTargetAndPose pc_pair = reader.getModelAndOneFrame(i);    // read a pair of pc
        statistics statistics_ = eval.register_ransac_icp(*pc_pair.src, *pc_pair.tgt, pc_pair.pose);   // multi-level reg

        // makeup some input data info
        statistics_.src = reader.getSourcePath(i);
        statistics_.voxel_size = reader.getVoxelSize(i);
        statistics_.noise_src = reader.getSourceNoise(i);
        eval.recordstatistics(statistics_);
        
        std::cout << "Iter " << i << "/" << reader.get_length() << std::endl;
        std::cout << "  voxel size  " <<  statistics_.voxel_size << std::endl;
        std::cout << "  noise sigma " <<  statistics_.noise_src << std::endl;
        std::cout << "  time total  " <<  statistics_.time << std::endl;
    }

    //  std::string output_dir = "/home/cheng/proj/3d/3DPcReg/snapshot/";
    std::string output_dir = "/home/cheng_chen/3d_projects/3DPcReg/snapshot/";

    eval.save(output_dir);

    return 0;
}
