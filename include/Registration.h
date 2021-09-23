//
// Created by cheng on 9/17/21.
//

#ifndef INC_3DPCREG_REGISTRATION_H
#define INC_3DPCREG_REGISTRATION_H

#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_map>
#include "nlohmann/json.hpp"
#include "Eigen/Core"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
# include "FeatureCompute.h"
#include "SolveCorrespondenceAndRigidTransformation.h"
#include "utils.h"

class Registration {
public:
    verification::statistic_reg register_ransac_icp(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                                                    const Eigen::Matrix4d& tf_gt=Eigen::Matrix4d::Identity());


    Registration();

private:
    bool visualize = true;
//    std::vector<double> voxel_size_global {6, 4};
//    std::vector<double> voxel_size_local {5, 0.8};
    std::vector<verification::statistic_reg> statistics;
};


#endif //INC_3DPCREG_REGISTRATION_H
