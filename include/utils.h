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

bool DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
             const Eigen::Matrix4d &transformation=Eigen::Matrix4d::Identity(), const std::string &win_name="Registration");

std::shared_ptr<open3d::geometry::PointCloud> FilterPointsOutBound(const open3d::geometry::PointCloud &pc,
                                                                   const Eigen::Vector3d& min_bound = {-5000.0, -5000.0, 200.0},
                                                                   const Eigen::Vector3d& max_bound = {5000.0, 5000.0, 800.0});

std::shared_ptr<open3d::geometry::LineSet> GetCorrespoundenceLines(const open3d::geometry::PointCloud &src,
                                                                               const open3d::geometry::PointCloud &tgt,
                                                                               const open3d::pipelines::registration::CorrespondenceSet &corr);

#endif //INC_3DPCREG_UTILS_H
