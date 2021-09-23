//
// Created by cheng on 9/16/21.
//

#ifndef INC_3DPCREG_POINTCLOUDREADER_H
#define INC_3DPCREG_POINTCLOUDREADER_H
#pragma once
#include <iostream>
#include <fstream>
//#include <cassert>
#include <vector>
#include "nlohmann/json.hpp"

#include "Eigen/Core"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"

struct sourceTargetAndPose {
//    double voxel_sizes;
    std::shared_ptr<open3d::geometry::PointCloud> src;
    std::shared_ptr<open3d::geometry::PointCloud> tgt;
    Eigen::Matrix4d pose;
//    std::shared_ptr<Eigen::Matrix4d> pose;
};

class PointCloudReader {
public:
    virtual bool loadModel(std::string file_path);
    virtual bool loadOneFrame(std::string file_path);
    virtual bool loadFramesAndPoseFromDir(std::string frames_dir_path, std::string poses_dir_path);
    virtual bool loadOnePose(std::string poses_dir_path);
    virtual int get_length();
    virtual Eigen::Matrix4d readPose(std::string pose_file_path);
    sourceTargetAndPose getModelAndOneFrame(int index_frame);

    PointCloudReader();
//    virtual ~PointCloudReader();

protected:
    std::vector<std::string> model_paths;
    std::vector<std::string> frame_paths;
    std::vector<std::string> pose_file_paths;
    std::vector<Eigen::Matrix4d> poses;

private:
    bool safe_model = true;
    float voxel_size_read_pc = 0.02;
//    float voxel_size_model = 5;
//    float voxel_size_frame = 5;

    std::shared_ptr<open3d::geometry::PointCloud> readPointCloud(std::string file_path, double voxel_size_downsampling);
};

///////////////////////////////////////////////////////////////////////////////

class PointCloudReaderFromJson: public PointCloudReader {
public:
    bool loadJson(std::string file_path);

    PointCloudReaderFromJson();

protected:


private:
    std::string root_path = "/home/cheng/proj/3d/TEASER-plusplus/";
    nlohmann::json sources;
};


#endif //INC_3DPCREG_POINTCLOUDREADER_H
