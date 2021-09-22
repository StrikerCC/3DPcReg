//
// Created by cheng on 9/8/21.
//

#ifndef INC_3DPCREG_UTILS_H
#define INC_3DPCREG_UTILS_H

#include <iostream>
#include "Eigen/Eigen"



bool IsCloseEnough(float num1, float num2);

bool IsCloseEnough(double num1, double num2);

bool IsCloseEnough(int num1, int num2);

struct register_result {
    double error_rotation {0.0};
    double error_translation {0.0};
};
register_result ComputeRegError(const Eigen::Matrix4d &pose_1, const Eigen::Matrix4d &pose_2);

#endif //INC_3DPCREG_UTILS_H
