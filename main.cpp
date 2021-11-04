//#pragma once
#include <vector>
#include <iostream>
#include <iomanip>
//#include "PointCloudReader.h"
#include "CameraToOpen3d.h"
#include "Registration.h"
#include "utils.h"
//#include "Registration_mix.h"
//#include "Evaluation.h"
//#include "statistics.h"
#include <thread>

int demo() {
//    std::string file_src_path = "../data/3D_mode_manl_face.pcd";
    std::string file_src_path = "../data/3D_model_face_from_mr.pcd";
    double voxel_size_down = 0.5;

    CameraToOpen3d cam = CameraToOpen3d();
    if (not cam.WorkingProperly()) return -1;

    Registration reg = Registration();

    /// read source point cloud
    open3d::geometry::PointCloud pc_src_original = open3d::geometry::PointCloud();
    open3d::io::ReadPointCloud(file_src_path, pc_src_original);
    std::shared_ptr<open3d::geometry::PointCloud> pc_src = pc_src_original.VoxelDownSample(voxel_size_down);

    /// target point cloud parameters


    /// vis ready
//    open3d::visualization::Visualizer vis;
//    vis.CreateVisualizerWindow();
//    const std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr = std::make_shared<open3d::geometry::PointCloud>(*pc_src);
//    const std::shared_ptr<open3d::geometry::PointCloud> target_ptr= std::make_shared<open3d::geometry::PointCloud>(*pc_src);;
//    vis.AddGeometry(source_transformed_ptr);
//    vis.AddGeometry(target_ptr);
//    vis.Run();

    /// reg iteration
    for (int i = 0; i < 7; i++) {
        if (not cam.WorkingProperly()) return -1;

        /// read target point cloud from CameraToOpen3d
        auto pc_tgt_original = cam.GetNewFrame();
        auto pc_tgt = pc_tgt_original->VoxelDownSample(voxel_size_down);

        /// reg
        auto result = reg.register_ransac_icp(pc_src, pc_tgt);

        /// cout
        auto ransac_reg_result = result.global_reg_result;
        auto icp_reg_result = result.local_reg_result;
        DrawReg(*pc_src, *pc_tgt, ransac_reg_result->transformation_, "ransac");

        std::cout << "Final " << std::endl;
        std::cout << "  fitness: " << icp_reg_result->fitness_ << ", inlier rmse " << icp_reg_result->inlier_rmse_ << std::endl;
        std::cout << "  time each : " << result.time << std::endl;
        std::cout << "  time total:" << (double) result.time["time_global_features"] + (double) result.time["time_local_features"] + (double) result.time["time_global_reg"] + (double) result.time["time_local_reg"] << std::endl;
        std::cout << std::endl;

        /// visual
        std::cout << ransac_reg_result->correspondence_set_.size() << std::endl;
        DrawReg(*pc_src, *pc_tgt, ransac_reg_result->transformation_, "Ransac");
        DrawReg(*pc_src, *pc_tgt, icp_reg_result->transformation_, "ICP Final");

//        std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(new open3d::geometry::PointCloud);
//        std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);

//        *source_transformed_ptr = *pc_src;
//        source_transformed_ptr->Transform(tf);
//        source_transformed_ptr->PaintUniformColor({0.0, 0.0, 1.0});
//
//        *target_ptr = *pc_tgt;
//        target_ptr->PaintUniformColor({0.0, 1.0, 0.0});

//        vis.UpdateGeometry(target_ptr);
//        vis.UpdateGeometry(source_transformed_ptr);
//        vis.PollEvents();
//        vis.UpdateRender();
//        DrawReg(*pc_src, *pc_tgt);
    }
//    vis.DestroyVisualizerWindow();
    return 0;
}


int compare() {
//    std::string file_src_path = "../data/3D_mode_manl_face.pcd";
    std::string file_src_path = "../data/3D_model_face_from_mr.pcd";
    std::string file_tgt_path = "./pc_temp.ply";

    double voxel_size_down = 0.5;

    Registration reg = Registration();

    /// read source point cloud
    open3d::geometry::PointCloud pc_src_original = open3d::geometry::PointCloud();
    open3d::io::ReadPointCloud(file_src_path, pc_src_original);
    std::shared_ptr<open3d::geometry::PointCloud> pc_src = pc_src_original.VoxelDownSample(voxel_size_down);

    /// target point cloud parameters


    /// vis ready
//    open3d::visualization::Visualizer vis;
//    vis.CreateVisualizerWindow();
//    const std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr = std::make_shared<open3d::geometry::PointCloud>(*pc_src);
//    const std::shared_ptr<open3d::geometry::PointCloud> target_ptr= std::make_shared<open3d::geometry::PointCloud>(*pc_src);;
//    vis.AddGeometry(source_transformed_ptr);
//    vis.AddGeometry(target_ptr);
//    vis.Run();

    /// reg iteration
    for (int i = 0; i < 7; i++) {

        /// read target point cloud from CameraToOpen3d
//        auto pc_tgt_original = cam.GetNewFrame();
        open3d::geometry::PointCloud pc_tgt_original;
        open3d::io::ReadPointCloud(file_tgt_path, pc_tgt_original);
        auto pc_tgt = FilterPointsOutBound(pc_tgt_original);
        pc_tgt = pc_tgt->VoxelDownSample(voxel_size_down);

        /// reg
        open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
        auto result = reg.register_ransac_icp(pc_src, pc_tgt);

        /// cout
        auto ransac_reg_result = result.global_reg_result;
        auto icp_reg_result = result.local_reg_result;

        std::cout << "Final " << std::endl;
        std::cout << "  fitness: " << icp_reg_result->fitness_ << ", inlier rmse " << icp_reg_result->inlier_rmse_ << std::endl;
        std::cout << "  time each : " << result.time << std::endl;
        std::cout << "  time total:" << (double) result.time["time_global_features"] + (double) result.time["time_local_features"] + (double) result.time["time_global_reg"] + (double) result.time["time_local_reg"] << std::endl;
        std::cout << std::endl;

        /// visual
        DrawReg(*pc_src, *pc_tgt, ransac_reg_result->transformation_, "Ransac");
        DrawReg(*pc_src, *pc_tgt, icp_reg_result->transformation_, "ICP Final");
//        std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(new open3d::geometry::PointCloud);
//        std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);

//        *source_transformed_ptr = *pc_src;
//        source_transformed_ptr->Transform(tf);
//        source_transformed_ptr->PaintUniformColor({0.0, 0.0, 1.0});
//
//        *target_ptr = *pc_tgt;
//        target_ptr->PaintUniformColor({0.0, 1.0, 0.0});

//        vis.UpdateGeometry(target_ptr);
//        vis.UpdateGeometry(source_transformed_ptr);
//        vis.PollEvents();
//        vis.UpdateRender();
//        DrawReg(*pc_src, *pc_tgt);

    }
//    vis.DestroyVisualizerWindow();
    return 0;
}


//int eval() {
//    std::cout << "Point cloud Reg eval" << std::endl;
//    //////////////////////////////////////////////
//    std::cout << "Start reading" << std::endl;
//
//    Evaluation eval = Evaluation();
//    PointCloudReaderFromJson reader = PointCloudReaderFromJson();
//
////    reader.loadDataFromRootDirAndJson("/home/cheng_chen/3d_projects/Teaser-plusplus-testing/", "/home/cheng_chen/3d_projects/Teaser-plusplus-testing/data/human_data_1152/data.json");
//    reader.loadDataFromRootDirAndJson("/home/cheng/proj/3d/TEASER-plusplus/", "/home/cheng/proj/3d/TEASER-plusplus/data/human_data/data.json");
//
//    for (int i = 0; i < reader.get_length(); ++i) {
//        // for (int i = 0; i < 100; ++i) {
//        sourceTargetAndPose pc_pair = reader.getModelAndOneFrame(i);    // read a pair of pc
//        statistics statistics_ = eval.register_ransac_icp(*pc_pair.src, *pc_pair.tgt, pc_pair.pose);   // multi-level reg
//
//        // makeup some input data info
//        statistics_.src = reader.getSourcePath(i);
//        statistics_.voxel_size = reader.getVoxelSize(i);
//        statistics_.noise_src = reader.getSourceNoise(i);
//        eval.recordstatistics(statistics_);
//
//        std::cout << "Iter " << i << "/" << reader.get_length() << std::endl;
//        std::cout << "  voxel size  " <<  statistics_.voxel_size << std::endl;
//        std::cout << "  noise sigma " <<  statistics_.noise_src << std::endl;
//        std::cout << "  time total  " <<  statistics_.time << std::endl;
//    }
//
////    std::string output_dir = "/home/cheng_chen/3d_projects/3DPcReg/snapshot/";
//    std::string output_dir = "/home/cheng/proj/3d/3DPcReg/snapshot/";
//
//    eval.save(output_dir);
//
//    return 0;
//}

int ThreadTest_() {
    std::thread t1([] {
        for (int i = 0; i < 10; i++) {
            std::cout << "Thread: " << i << std::endl;
        }
    });
//    t1.join();
    for (int i = 0; i < 100; i++) {
        std::cout << "Main: " << i << std::endl;
    }
//    t1.detach();
    return 1;
}


int main() {
    compare();
//    demo();
//    ThreadTest_();
    return 1;
}