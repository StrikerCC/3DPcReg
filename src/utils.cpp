//
// Created by cheng on 9/8/21.
//
#include "utils.h"

bool IsCloseEnough(float num1, float num2) {
    return std::abs(num1-num2) < 0.001;
}

bool IsCloseEnough(double num1, double num2) {
    return IsCloseEnough((float) num1, (float) num2);
}

bool IsCloseEnough(int num1, int num2) {
    return IsCloseEnough((float) num1, (float) num2);
}

register_result ComputeRegError(const Eigen::Matrix4d &pose_1, const Eigen::Matrix4d &pose_2) {
    Eigen::Matrix3d rotation_error = pose_1.block<3, 3>(0, 0) * pose_2.block<3 ,3>(0, 0).transpose();
    Eigen::AngleAxisd rotation_error_vector;
    rotation_error_vector.fromRotationMatrix(rotation_error);

    Eigen::Vector3d translation_error = pose_1.block<3, 1>(0, 3) - pose_2.block<3, 1>(0, 3);

    register_result result;
    result.error_rotation = rotation_error_vector.angle();
    result.error_translation = translation_error.norm();
    return result;
}