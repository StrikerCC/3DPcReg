//
// Created by cheng on 9/16/21.
//

#include "SolveCorrespondenceAndRigidTransformation.h"

SolveCorrespondenceAndRigidTransformation::SolveCorrespondenceAndRigidTransformation() = default;


open3d::pipelines::registration::RegistrationResult SolveCorrespondenceAndRigidTransformation::solve_icp(std::shared_ptr<open3d::geometry::PointCloud> const& src,
                                                          std::shared_ptr<open3d::geometry::PointCloud> const& tgt,
                                                          double voxel_size,
                                                          const Eigen::Matrix4d& tf) {

    int max_iteration = 4000000, max_validation = 1000;
    double max_correspondence_dis = voxel_size * 0.5;
    auto source = *src, target = *tgt;

    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_dege_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(0.075);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
            0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_dege_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    correspondence_checker.emplace_back(correspondence_checker_normal);
    auto registration_result = open3d::pipelines::registration::RegistrationICP(source, target,
                                                                                max_correspondence_dis,
                                                                                tf,
                                                                                open3d::pipelines::registration::TransformationEstimationPointToPlane(),
//                                                                                open3d::pipelines::registration::TransformationEstimationPointToPoint(
//                                                                                        false),
                                                                                open3d::pipelines::registration::ICPConvergenceCriteria(
                                                                                        max_iteration,
                                                                                        max_validation));

    // overload old pose if this reg may valid
//    if (Registration_mix::IsRegValid(registration_result.transformation_)) {
//        Registration_mix::pose_current_local = registration_result.transformation_;
//        Registration_mix::pose_current = registration_result.transformation_;
//        return true;
//    } else {
//        return false;
//    }
//    open3d::pipelines::registration::RegistrationResult registration_result
    return registration_result;
}

open3d::pipelines::registration::RegistrationResult SolveCorrespondenceAndRigidTransformation::solve_ransac(std::shared_ptr<open3d::geometry::PointCloud> const& src,
                                                             std::shared_ptr<open3d::pipelines::registration::Feature> const& src_feature,
                                                             std::shared_ptr<open3d::geometry::PointCloud> const& tgt,
                                                             std::shared_ptr<open3d::pipelines::registration::Feature> const& tgt_feature,
                                                             double voxel_size,
                                                             const Eigen::Matrix4d& tf) {
    int max_iteration = 4000000, max_validation = 1000;
    double max_correspondence_dis = voxel_size, distance_threshold = voxel_size;

    auto source = *src, target = *tgt;
    auto source_feature = *src_feature, target_feature = *tgt_feature;

    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_dege_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
            0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_dege_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    correspondence_checker.emplace_back(correspondence_checker_normal);
//    auto registration_result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(source, target,
//                                                                                                         source_feature,
//                                                                                                         target_feature,
//                                                                                                         true,
//                                                                                                         max_correspondence_dis,
//                                                                                                         open3d::pipelines::registration::TransformationEstimationPointToPoint(
//                                                                                                                 false),
////                                                                                                         open3d::pipelines::registration::TransformationEstimationPointToPlane(),
//                                                                                                         3,
//                                                                                                         correspondence_checker,
//                                                                                                         open3d::pipelines::registration::RANSACConvergenceCriteria(
//                                                                                                                 max_iteration,
//                                                                                                                 max_validation));
    auto registration_result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(source, target,
                                                                                                         source_feature,
                                                                                                         target_feature,
                                                                                                         true,
                                                                                                         max_correspondence_dis,
                                                                                                         open3d::pipelines::registration::TransformationEstimationPointToPoint(
                                                                                                                 false));
    // overload old pose if this reg may valid
//    if (Registration_mix::IsRegValid(registration_result.transformation_)) {
//        Registration_mix::pose_current_global = registration_result.transformation_;
//        Registration_mix::pose_current = registration_result.transformation_;
//        return true;
//    } else {
//        return false;
//    }
    return registration_result;
}
