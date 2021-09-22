//
// Created by cheng on 9/17/21.
//

#ifndef INC_3DPCREG_REGISTRATION_H
#define INC_3DPCREG_REGISTRATION_H

#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
# include "FeatureCompute.h"
#include "SolveCorrespondenceAndRigidTransformation.h"
#include "utils.h"

class Registration {
public:
    float register_ransac_icp(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                             const Eigen::Matrix4d& tf_gt=Eigen::Matrix4d::Identity());

    bool DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                 const Eigen::Matrix4d &transformation=Eigen::Matrix4d::Identity(), const std::string &win_name="Registration") const;

    Registration();
private:
    bool visualize = true;
    std::vector<double> voxel_size_global {5};
    std::vector<double> voxel_size_local {6, 3, 1, 0.4};
//    std::vector<double> voxel_size_local {5};
};


#endif //INC_3DPCREG_REGISTRATION_H
