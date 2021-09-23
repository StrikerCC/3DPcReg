//
// Created by cheng on 9/22/21.
//

#ifndef INC_3DPCREG_EVALUATION_H
#define INC_3DPCREG_EVALUATION_H

#pragma once
#include "utils.h"
#include "Registration.h"
#include "PointCloudReader.h"
#include "nlohmann/json.hpp"


class Evaluation {
public:
    Evaluation();
    verification::statistic_reg register_ransac_icp(const open3d::geometry::PointCloud& source,
                        const open3d::geometry::PointCloud& target,
                        const Eigen::Matrix4d& tf_gt);

    bool recordError(verification::statistic_reg statistic_reg_);
    bool save(const std::string& output_json_path = "");

private:

    bool visualize = false;
    std::vector<double> voxel_size_global {6, 4};
    std::vector<double> voxel_size_local {5, 3, 1, 0.4};
    std::vector<verification::statistic_reg> statistics;

    bool addAvgStddev();
};





#endif //INC_3DPCREG_EVALUATION_H
