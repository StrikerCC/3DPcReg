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
    result.error_rotation = rotation_error_vector.angle() * 180.0 / M_PI;
    result.error_translation = translation_error.norm();
    return result;
}

bool DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &transformation, const std::string &win_name) {
    if (source.points_.empty() or target.points_.empty()) {
        return false;
    }
    std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);

    *source_transformed_ptr = source;
    source_transformed_ptr->Transform(transformation);
    source_transformed_ptr->PaintUniformColor({0.0, 0.0, 1.0});

    *target_ptr = target;
    target_ptr->PaintUniformColor({0.0, 1.0, 0.0});

    open3d::visualization::DrawGeometries({source_transformed_ptr, target_ptr}, win_name);
    return true;
}

std::shared_ptr<open3d::geometry::PointCloud> FilterPointsOutBound(const open3d::geometry::PointCloud &pc,
                                                                   const Eigen::Vector3d& min_bound,
                                                                   const Eigen::Vector3d& max_bound) {
    open3d::geometry::AxisAlignedBoundingBox bbox(min_bound, max_bound);
    return pc.Crop(bbox);
}