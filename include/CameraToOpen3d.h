//
// Created by cheng on 10/26/21.
//

#ifndef INC_3DPCREG_CAMERATOOPEN3D_H
#define INC_3DPCREG_CAMERATOOPEN3D_H
#pragma once
#include <iostream>
#include <string>
#include "3DCamera.hpp"
#include "open3d/Open3D.h"
#include <cstdio>
#include "utils.h"


class CameraToOpen3d {
public:
    CameraToOpen3d();
    ~CameraToOpen3d();
    ERROR_CODE SetExposure(float exposure);
//    ERROR_CODE SetGain(float gain_);

    std::shared_ptr<open3d::geometry::PointCloud> GetNewFrame();
    int SaveNewFrame(const std::string& file_path) const;

    bool WorkingProperly() const {return working_normal_;};

//    void CheckAndMaintainCamera();

private:
    cs::ICameraPtr camera = nullptr;
    bool working_normal_ = false;

    int depthRange_min_ = 100;        // PROPERTY_EXT_DEPTH_RANGE
    int depthRange_max_ = 1000;
    int algorithm_contrast_ = 10;     // the threshold of algorithm

    float exposure_ = 3998;          // exposure_ time (micro second)
    float gain_ = 1;                 // sensor gain_ (intensity of inferred laser ?)

    Eigen::Vector3d min_bound_pc_tgt = {-5000.0, -5000.0, 200.0};
    Eigen::Vector3d max_bound_pc_tgt = {5000.0, 5000.0, 800.0};
};


#endif //INC_3DPCREG_CAMERATOOPEN3D_H
