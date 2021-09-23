//
// Created by cheng on 9/22/21.
//

#include "Evaluation.h"



verification::statistic_reg Evaluation::register_ransac_icp(const open3d::geometry::PointCloud& source,
                                                              const open3d::geometry::PointCloud& target,
                                                              const Eigen::Matrix4d& tf_gt) {
    // vis
    if (Evaluation::visualize) {
        DrawReg(source, target);
    }
    std::cout << "Original Source has " << source.points_.size() << " points" << std::endl;
    std::cout << "Original Target has " << target.points_.size() << " points" << std::endl;

    /// prepare registration parameters
    clock_t clock_start;
    float time_features_global {0.0}, time_features_local {0.0}, time_reg_global {0.0}, time_reg_local {0.0};
    std::vector<double> primary_voxel_size_global = Evaluation::voxel_size_global;
    std::vector<double> primary_voxel_size_local = Evaluation::voxel_size_local;
    SolveCorrespondenceAndRigidTransformation solver = SolveCorrespondenceAndRigidTransformation();
    verification::statistic_reg statistic_reg_;

    /// get features ready
    std::cout << "Compute global feature" << std::endl;
    clock_start = clock();

    FeatureCompute features = FeatureCompute();
    std::vector<PcNormalFpfh> src_features_global = features.getFPHFFeatures(source, primary_voxel_size_global);
    std::vector<PcNormalFpfh> tgt_features_global = features.getFPHFFeatures(target, primary_voxel_size_global);

    time_features_global = (float) (clock() - clock_start) / CLOCKS_PER_SEC;

    /// start global register
    std::cout << "Global Reg" << std::endl;
    clock_start = clock();

    Eigen::Matrix4d tf_global = Eigen::Matrix4d::Identity();
    for (int i = 0; i < primary_voxel_size_global.size(); ++i) {
        assert (primary_voxel_size_global.at(i) == src_features_global.at(i).voxel_size);
        assert (primary_voxel_size_global.at(i) == tgt_features_global.at(i).voxel_size);

        double voxel_size = primary_voxel_size_global.at(i);
        auto src_pc = src_features_global.at(i).pc, tgt_pc = tgt_features_global.at(i).pc;
        auto src_feature = src_features_global.at(i).fpfh,  tgt_feature = tgt_features_global.at(i).fpfh;
        verification::statistic statistic_;

        auto global_result = solver.solve_ransac(src_pc, src_feature, tgt_pc, tgt_feature, voxel_size, tf_global);
        tf_global = global_result.transformation_;
        auto error = ComputeRegError(tf_gt, tf_global);

        // statistic
        statistic_.method = "ransac";
        statistic_.voxel_size = voxel_size;
        statistic_.time = (float) (clock() - clock_start) / CLOCKS_PER_SEC;
        statistic_.error_r = error.error_rotation;
        statistic_.error_t = error.error_translation;
        statistic_reg_.statistics.push_back(statistic_);

        // vis
        if (Evaluation::visualize) {
            time_reg_global += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
            DrawReg(*src_pc, *tgt_pc, tf_global, "Global registration result #" + std::to_string(i) + " voxel size " + std::to_string(voxel_size));
            clock_start = clock();
        }
    }

    time_reg_global = (float) (clock() - clock_start) / CLOCKS_PER_SEC;
//    std::cout << tf_global << std::endl;

    // vis
    if (Evaluation::visualize) {
        DrawReg(source, target, tf_global, "Global registration final result");
    }

    /// get features ready
    std::cout << "Compute local feature" << std::endl;
    clock_start = clock();

    std::vector<PcNormal> src_features_local = features.getNormalFeature(source, primary_voxel_size_local);
    std::vector<PcNormal> tgt_features_local = features.getNormalFeature(target, primary_voxel_size_local);

    time_features_local = (float) (clock() - clock_start) / CLOCKS_PER_SEC;

    /// start local reg
    std::cout << "Local Reg" << std::endl;
    clock_start = clock();

    Eigen::Matrix4d tf_local = tf_global;
    for (int i = 0; i < primary_voxel_size_local.size(); ++i) {
        assert (primary_voxel_size_local.at(i) == src_features_local.at(i).voxel_size);
        assert (primary_voxel_size_local.at(i) == tgt_features_local.at(i).voxel_size);
        double voxel_size = primary_voxel_size_local.at(i);
        auto src_pc = src_features_local.at(i).pc, tgt_pc = tgt_features_local.at(i).pc;
        verification::statistic statistic_;

        auto icp_result = solver.solve_icp(src_pc, tgt_pc, voxel_size, tf_local);
        tf_local = icp_result.transformation_;
        auto error = ComputeRegError(tf_gt, tf_local);

        std::cout << "ICP iteration "  << i << " voxel size " << voxel_size << std::endl;
        std::cout << "      Source has " << src_pc->points_.size() << " points" << std::endl;
        std::cout << "      Target has " << tgt_pc->points_.size() << " points" << std::endl;


        // statistic
        statistic_.method = "ransac";
        statistic_.voxel_size = voxel_size;
        statistic_.time = (float) (clock() - clock_start) / CLOCKS_PER_SEC;
        statistic_.error_r = error.error_rotation;
        statistic_.error_t = error.error_translation;
        statistic_reg_.statistics.push_back(statistic_);

        // vis
        if (Evaluation::visualize) {
            time_reg_local += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
            DrawReg(*src_pc, *tgt_pc, tf_local, "Local registration #" + std::to_string(i) + " voxel size " + std::to_string(voxel_size));
            clock_start = clock();
        }
    }

    time_reg_local += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
//    std::cout << tf_local << std::endl;
//    std::cout << tf_gt << std::endl;

    // vis
    if (Evaluation::visualize) {
        DrawReg(source, target, tf_local, "Local registration final result");
    }
    auto error = ComputeRegError(tf_gt, tf_local);
    std::cout << "Time global feature computing " << time_features_global << std::endl;
    std::cout << "Time global registration      " << time_reg_global << std::endl;
    std::cout << "Time local feature computing  " << time_features_local << std::endl;
    std::cout << "Time local registration      " << time_reg_local << std::endl;
    std::cout << "Rotation statistic_reg            " << error.error_rotation << " degree" << std::endl;
    std::cout << "Translation statistic_reg         " << error.error_translation << "  mm" << std::endl;

    // compute statistic_reg if gt was given

//    std::cout << "Frame # " << Registration_mix::num_frame << " takes seconds" << time_global+time_local << "\n   global takes " << time_global << " seconds" << "\n   local takes " << time_local << " seconds" << std::endl;
//    if (pose_gt != nullptr) {
//        std::tuple<float, float> error_rotation_translation = Registration_mix::ComputeRegError(Registration_mix::pose_current_global, *pose_gt);
//        std::cout << "Error " << std::endl;
//        std::cout << " Rotation   " << std::to_string(std::get<0>(error_rotation_translation)) << std::endl;
//        std::cout << " Translation" << std::to_string(std::get<0>(error_rotation_translation)) << std::endl;
//        std::cout << std::endl;
//    }
    statistic_reg_.time_total = time_features_global + time_reg_global + time_features_local + time_reg_local;
    statistic_reg_.error_r_final = error.error_rotation;
    statistic_reg_.error_t_final = error.error_translation;
//    statistic_reg_.voxel_size_final = primary_voxel_size_local.at(primary_voxel_size_local.size()-1);
    statistic_reg_.voxel_size_final = primary_voxel_size_local[-1];
    return statistic_reg_;
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


bool Evaluation::recordError(verification::statistic_reg statistic_reg_) {
    this->statistics.push_back(statistic_reg_);
    return false;
}

bool Evaluation::addAvgStddev() {

    // copy the map structure from statistic_reg, using the first statistic
    verification::statistic_reg statistics_avg;
    verification::statistic_reg statistics_stddev;

    /// compute mean first
    statistics_avg.src = "mean";
    for (const auto& statistic : this->statistics) {
        statistics_avg.time_total += statistic.time_total;
        statistics_avg.error_r_final += statistic.error_r_final;
        statistics_avg.error_t_final += statistic.error_t_final;
        // TODO
    }
    statistics_avg.time_total /= (double) this->statistics.size();
    statistics_avg.error_r_final /= (double) this->statistics.size();
    statistics_avg.error_t_final /= (double) this->statistics.size();

    // compute std dev then
    statistics_stddev.src = "stddev";
    for (const auto& statistic : this->statistics) {
        statistics_stddev.time_total += pow(statistic.time_total-statistics_avg.time_total, 2);
        statistics_stddev.error_r_final += pow(statistic.error_r_final-statistics_avg.error_r_final, 2);
        statistics_stddev.error_t_final += pow(statistic.error_t_final-statistics_avg.error_t_final, 2);
        // TODO
    }
    statistics_stddev.time_total /= (double) this->statistics.size();
    statistics_stddev.error_r_final /= (double) this->statistics.size();
    statistics_stddev.error_t_final /= (double) this->statistics.size();


    /// add avg and stddev to statistic_reg
    this->statistics.push_back(statistics_avg);
    this->statistics.push_back(statistics_stddev);
    return true;
}

bool Evaluation::save(const std::string& output_json_path) {
    // compute mean and std dev
    this->addAvgStddev();

    /// write prettified JSON to output file
    // struct statistic to json
    std::vector<nlohmann::json> vector_json;
    for (verification::statistic_reg statistic : this->statistics) {
        nlohmann::json error_json(statistic);
        vector_json.push_back(error_json);
    }

    nlohmann::json error_json(vector_json);
    std::ofstream o(output_json_path);
    o << error_json << std::endl;
    return false;
}

Evaluation::Evaluation() = default;
