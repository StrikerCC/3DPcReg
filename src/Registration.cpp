////
//// Created by cheng on 9/17/21.
////

#include <Registration.h>

Registration_Statistics Registration::register_ransac_icp(const std::shared_ptr<open3d::geometry::PointCloud> &pc_ptr_src,
                                                          const std::shared_ptr<open3d::geometry::PointCloud> &pc_ptr_tgt) {
    auto pc_src = *pc_ptr_src, pc_tgt = *pc_ptr_tgt;

    /// prepare registration parameters
    float time_global_features{0.0}, time_local_features{0.0}, time_global_reg{0.0}, time_local_reg{0.0};

    std::vector<double> primary_voxel_size_global = Registration::voxel_size_global;
    std::vector<double> primary_voxel_size_local = Registration::voxel_size_local;

    open3d::pipelines::registration::RegistrationResult ransac_reg_result;
    open3d::pipelines::registration::RegistrationResult icp_reg_result;

    /// get global parameters and features ready
    std::chrono::steady_clock::time_point clock_global_feature_start = std::chrono::steady_clock::now();
    std::cout << "Compute global feature" << std::endl;
    FeatureCompute features = FeatureCompute();
    std::vector<PcNormalFpfh> features_global_src = features.getFPHFFeatures(pc_src, primary_voxel_size_global);
    std::vector<PcNormalFpfh> features_global_tgt = features.getFPHFFeatures(pc_tgt, primary_voxel_size_global);
    time_global_features = (float) (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - clock_global_feature_start).count());

    /// start global register
    std::cout << "Global Reg" << std::endl;
    std::chrono::steady_clock::time_point clock_global_reg_start = std::chrono::steady_clock::now();
    for (int i = 0; i < primary_voxel_size_global.size(); ++i) {
        double voxel_size = primary_voxel_size_global.at(i);
        auto pc_src_down = features_global_src.at(i).pc, pc_tgt_down = features_global_tgt.at(i).pc;
        auto feat_src = features_global_src.at(i).fpfh, feat_tgt = features_global_tgt.at(i).fpfh;

        ransac_reg_result = ransac(pc_src_down, feat_src, pc_tgt_down, feat_tgt, voxel_size);

        std::cout << "ransac " << voxel_size << std::endl;
        std::cout << "fitness " << ransac_reg_result.fitness_ << std::endl;
        std::cout << "inlier rmse " << ransac_reg_result.inlier_rmse_ << std::endl;

        auto corr_lines = GetCorrespoundenceLines(*pc_src_down, *pc_tgt_down, ransac_reg_result.correspondence_set_);
        corr_lines->PaintUniformColor(Eigen::Vector3d({1.0, 0.0, 0.0}));
        open3d::visualization::DrawGeometries({pc_src_down, pc_tgt_down, corr_lines});

//        open3d::visualization::Visualizer vis;
//        std::shared_ptr<open3d::geometry::LineSet> lines;
//        vis.AddGeometry(lines);
//        vis.AddGeometry(pc_ptr_src);
//        vis.AddGeometry(pc_ptr_tgt);

        // overload old pose if this reg may valid
//    if (Registration_mix::IsRegValid(registration_result.transformation_)) {
//        Registration_mix::pose_current_global = registration_result.transformation_;
//        Registration_mix::pose_current = registration_result.transformation_;
//        return true;
//    } else {
//        return false;
//    }
    }
    time_global_reg = (float) (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - clock_global_reg_start).count());

    /// get local features ready
    std::cout << "Compute local feature" << std::endl;
    std::chrono::steady_clock::time_point clock_local_feature_start = std::chrono::steady_clock::now();

    std::vector<PcNormal> src_features_local = features.getNormalFeature(pc_src, primary_voxel_size_local);
    std::vector<PcNormal> tgt_features_local = features.getNormalFeature(pc_tgt, primary_voxel_size_local);

    time_local_features = (float) (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - clock_local_feature_start).count());

    /// start local reg
    std::cout << "Local Reg" << std::endl;
    icp_reg_result = ransac_reg_result;     // overwrite icp result for first icp reg
    std::chrono::steady_clock::time_point clock_local_reg_start = std::chrono::steady_clock::now();

    for (int i = 0; i < primary_voxel_size_local.size(); ++i) {
        assert (primary_voxel_size_local.at(i) == src_features_local.at(i).voxel_size);
        assert (primary_voxel_size_local.at(i) == tgt_features_local.at(i).voxel_size);

        double voxel_size = primary_voxel_size_local.at(i);
        auto pc_src_down = src_features_local.at(i).pc, pc_tgt_down = tgt_features_local.at(i).pc;

        icp_reg_result = icp(pc_src_down, pc_tgt_down, voxel_size, icp_reg_result.transformation_);

        std::cout << i << " icp , voxel size: " << voxel_size << std::endl;
        std::cout << "fitness " << icp_reg_result.fitness_ << std::endl;
        std::cout << "inlier rmse " << icp_reg_result.inlier_rmse_ << std::endl;
//        DrawReg(*pc_src_down, *pc_tgt_down, icp_reg_result.transformation_, "icp" + std::to_string(voxel_size));

//        std::cout << "ICP iteration " << i << " voxel size " << voxel_size << ", reg in " << " seconds" << std::endl;
//        std::cout << "      Source has " << pc_src_down->points_.size() << " points" << std::endl;
//        std::cout << "      Target has " << pc_tgt_down->points_.size() << " points" << std::endl;
    }
    time_local_reg = (float) (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - clock_local_reg_start).count());

    double digits_round = 1000.0;
    Registration_Statistics reg_statistic = {
            std::make_shared<open3d::pipelines::registration::RegistrationResult>(ransac_reg_result),
            std::make_shared<open3d::pipelines::registration::RegistrationResult>(icp_reg_result),
                    nlohmann::json ({
                        {"time_global_features", round(time_global_features*digits_round)/ digits_round},
                        {"time_local_features", round(time_local_features*digits_round)/ digits_round},
                        {"time_global_reg", round(time_global_reg*digits_round)/ digits_round},
                        {"time_local_reg", round(time_local_reg*digits_round)/ digits_round}
                    })
    };
    return reg_statistic;
}


open3d::pipelines::registration::RegistrationResult Registration::ransac(
        const std::shared_ptr<open3d::geometry::PointCloud> &src,
        const std::shared_ptr<open3d::pipelines::registration::Feature> &src_feature,
        const std::shared_ptr<open3d::geometry::PointCloud> &tgt,
        const std::shared_ptr<open3d::pipelines::registration::Feature> &tgt_feature, double voxel_size) {

    int max_iteration = 10000000;
    float confidence = 0.999;
//    double max_correspondence_dis = voxel_size * 0.5;
    double max_correspondence_dis = 2.0;

    auto source = *src, target = *tgt;
    auto source_feature = *src_feature, target_feature = *tgt_feature;

    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_dege_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
//            0.9);
    double distance_threshold = voxel_size * 0.5;
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
            0.52359878);
//            0.52359878);
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
                                                                                                                 false) );
//                                                                                                         3,
//                                                                                                         correspondence_checker);
//                                                                                                         open3d::pipelines::registration::RANSACConvergenceCriteria(
//                                                                                                                 max_iteration,
//                                                                                                                 confidence));

    return registration_result;
}

open3d::pipelines::registration::RegistrationResult Registration::icp(
        const std::shared_ptr<open3d::geometry::PointCloud> &src,
        const std::shared_ptr<open3d::geometry::PointCloud> &tgt, double voxel_size,
        const Eigen::Matrix4d &tf) {
    int max_iteration = 4000000, max_validation = 1000;
    double max_correspondence_dis = voxel_size;
    auto source = *src, target = *tgt;
    auto tf_ = tf;
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_dege_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(0.075);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
            0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_dege_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    correspondence_checker.emplace_back(correspondence_checker_normal);
    open3d::pipelines::registration::RegistrationResult registration_result;
    std::vector<float> factors = {5.0, 2.5, 1.2};
    for (auto factor : factors) {
        auto max_correspondence_dis_ = max_correspondence_dis * factor;
        registration_result = open3d::pipelines::registration::RegistrationICP(source, target,
                                                                                    max_correspondence_dis_,
                                                                                    tf,
                                                                                    open3d::pipelines::registration::TransformationEstimationPointToPlane(),
//                                                                                open3d::pipelines::registration::TransformationEstimationPointToPoint(
//                                                                                        false),
                                                                                    open3d::pipelines::registration::ICPConvergenceCriteria(
                                                                                            max_iteration,
                                                                                            max_validation));
        tf_ = registration_result.transformation_;
    }
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

Registration::Registration() = default;

