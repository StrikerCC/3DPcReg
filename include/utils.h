//
// Created by cheng on 9/8/21.
//

#ifndef INC_3DPCREG_UTILS_H
#define INC_3DPCREG_UTILS_H

#pragma once
#include <iostream>
#include "Eigen/Eigen"
#include "nlohmann/json.hpp"
#include "open3d/Open3D.h"


bool IsCloseEnough(float num1, float num2);

bool IsCloseEnough(double num1, double num2);

bool IsCloseEnough(int num1, int num2);

struct register_result {
    double error_rotation {0.0};
    double error_translation {0.0};
};
register_result ComputeRegError(const Eigen::Matrix4d &pose_1, const Eigen::Matrix4d &pose_2);


namespace verification {
    struct statistic {
        std::string method;
        double voxel_size;
        double time;
        double error_r;
        double error_t;
    };

    struct statistic_reg {
        std::string src;
        double voxel_size_final;
        double time_total;
        double error_r_final;
        double error_t_final;
        std::vector<statistic> statistics;
    };

    void to_json(nlohmann::json &j, const statistic_reg &statistics_);
};

bool DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
             const Eigen::Matrix4d &transformation=Eigen::Matrix4d::Identity(), const std::string &win_name="Registration");

#endif //INC_3DPCREG_UTILS_H
