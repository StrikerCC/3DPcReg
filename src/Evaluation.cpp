//
// Created by cheng on 9/22/21.
//

#include "Evaluation.h"



statistics Evaluation::register_ransac_icp(const open3d::geometry::PointCloud& source,
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
    statistics statistic_reg_;

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
        statistics statistic_;

        auto global_result = solver.solve_ransac(src_pc, src_feature, tgt_pc, tgt_feature, voxel_size, tf_global);
        tf_global = global_result.transformation_;
        auto error = ComputeRegError(tf_gt, tf_global);

        // statistic
        statistic_.method = "ransac";
        statistic_.voxel_size = voxel_size;
        statistic_.time = (float) (clock() - clock_start) / CLOCKS_PER_SEC;
        statistic_.error_r = error.error_rotation;
        statistic_.error_t = error.error_translation;
        statistic_reg_.substatistics.push_back(statistic_);

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
        statistics statistic_;

        auto icp_result = solver.solve_icp(src_pc, tgt_pc, voxel_size, tf_local);
        tf_local = icp_result.transformation_;
        auto error = ComputeRegError(tf_gt, tf_local);

        std::cout << "ICP iteration "  << i << " voxel size " << voxel_size << std::endl;
        std::cout << "      Source has " << src_pc->points_.size() << " points" << std::endl;
        std::cout << "      Target has " << tgt_pc->points_.size() << " points" << std::endl;


        // statistic
        statistic_.method = "icp";
        statistic_.voxel_size = voxel_size;
        statistic_.time = (float) (clock() - clock_start) / CLOCKS_PER_SEC;
        statistic_.error_r = error.error_rotation;
        statistic_.error_t = error.error_translation;
        statistic_reg_.substatistics.push_back(statistic_);

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

    statistic_reg_.method = "ransac_icp";
//    statistic_reg_.src = ;
    statistic_reg_.voxel_size = primary_voxel_size_local.at(primary_voxel_size_local.size()-1);
    statistic_reg_.time = time_features_global + time_reg_global + time_features_local + time_reg_local;
    statistic_reg_.error_r = error.error_rotation;
    statistic_reg_.error_t = error.error_translation;

    return statistic_reg_;
}

bool Evaluation::recordError(const statistics& statistic_reg_) {
    this->statistics_eval.push_back(statistic_reg_);
    return true;
}

bool Evaluation::addOrgAvgStddev() {

    // copy the map structure from statistic_reg, using the first statistic
    int num_success_case = 0;
    statistics statistics_avg_org;
    statistics statistics_avg_success_case;
    statistics statistics_stddev_org;
    statistics statistics_stddev_success_case;

    /// compute mean first
    statistics_avg_org.src = "mean";
    for (const auto& statistic : this->statistics_eval) {
        // take final error and time into account
        statistics_avg_org.time += statistic.time;
        statistics_avg_org.error_r += statistic.error_r;
        statistics_avg_org.error_t += statistic.error_t;
        // take error and time in each step into account
        for (int i = 0; i < statistic.substatistics.size(); i++) {
            if (statistics_avg_org.substatistics.empty()) {
                statistics_avg_org.substatistics = std::vector<statistics> (statistic.substatistics.size());
            }
            statistics_avg_org.substatistics[i].error_r += statistic.substatistics[i].error_r;
            statistics_avg_org.substatistics[i].error_t += statistic.substatistics[i].error_t;
            statistics_avg_org.substatistics[i].time += statistic.substatistics[i].time;
            statistics_avg_org.substatistics[i].voxel_size += statistic.substatistics[i].voxel_size;
        }

        // if error below threshold, take this case into success account
        if (statistic.error_r <  error_r_threshold && statistic.error_t < error_t_threshold) {
            num_success_case += 1;
//            statistics_avg_success_case = statistics_avg_org;
        }
    }
    statistics_avg_org.time /= (double) this->statistics_eval.size();
    statistics_avg_org.error_r /= (double) this->statistics_eval.size();
    statistics_avg_org.error_t /= (double) this->statistics_eval.size();
    for (int i = 0; i < this->statistics_eval[0].substatistics.size(); i++) {
        statistics_avg_org.substatistics[i].error_r /= (double) this->statistics_eval.size();
        statistics_avg_org.substatistics[i].error_t /= (double) this->statistics_eval.size();
        statistics_avg_org.substatistics[i].time /= (double) this->statistics_eval.size();
        statistics_avg_org.substatistics[i].voxel_size /= (double) this->statistics_eval.size();
        statistics_avg_org.substatistics[i].method = this->statistics_eval[0].substatistics[i].method;
    }

    // compute std dev then
    statistics_stddev_org.src = "stddev";
    for (const auto& statistic : this->statistics_eval) {
        statistics_stddev_org.time += pow(statistic.time - statistics_avg_org.time, 2);
        statistics_stddev_org.error_r += pow(statistic.error_r - statistics_avg_org.error_r, 2);
        statistics_stddev_org.error_t += pow(statistic.error_t - statistics_avg_org.error_t, 2);
        // add step statistic stddev
        for (int i = 0; i < statistic.substatistics.size(); i++) {
            if (statistics_stddev_org.substatistics.empty()) statistics_stddev_org.substatistics = std::vector<statistics> (statistic.substatistics.size());
            statistics_stddev_org.substatistics[i].error_r += pow(statistic.substatistics[i].error_r - statistics_avg_org.substatistics[i].error_r, 2);
            statistics_stddev_org.substatistics[i].error_t +=  pow(statistic.substatistics[i].error_t - statistics_avg_org.substatistics[i].error_t, 2);
            statistics_stddev_org.substatistics[i].time +=  pow(statistic.substatistics[i].time - statistics_avg_org.substatistics[i].time, 2);
            statistics_stddev_org.substatistics[i].voxel_size +=  pow(statistic.substatistics[i].voxel_size - statistics_avg_org.substatistics[i].voxel_size, 2);
        }
    }
    statistics_stddev_org.time = sqrt(statistics_stddev_org.time / (double) this->statistics_eval.size());
    statistics_stddev_org.error_r = sqrt(statistics_stddev_org.error_r / (double) this->statistics_eval.size());
    statistics_stddev_org.error_t = sqrt(statistics_stddev_org.error_t / (double) this->statistics_eval.size());
    for (int i = 0; i < this->statistics_eval[0].substatistics.size(); i++) {
        statistics_stddev_org.substatistics[i].error_r = sqrt(statistics_stddev_org.substatistics[i].error_r / (double) this->statistics_eval.size());
        statistics_stddev_org.substatistics[i].error_t = sqrt(statistics_stddev_org.substatistics[i].error_t / (double) this->statistics_eval.size());
        statistics_stddev_org.substatistics[i].time = sqrt(statistics_stddev_org.substatistics[i].time / (double) this->statistics_eval.size());
        statistics_stddev_org.substatistics[i].voxel_size = sqrt(statistics_stddev_org.substatistics[i].voxel_size / (double) this->statistics_eval.size());
        statistics_stddev_org.substatistics[i].method = this->statistics_eval[0].substatistics[i].method;
    }

    /// add avg and stddev to statistic_reg
    this->statistics_eval.push_back(statistics_avg_org);
    this->statistics_eval.push_back(statistics_stddev_org);
    return true;
}

bool Evaluation::addSuccessCaseAvgStddev() {
    return false;
}

bool Evaluation::save(const std::string& output_dir) {
    std::string output_dir_ = output_dir;
    if (output_dir_.back() != '/') {
        output_dir_ += + "/";
    }

    /// determine output json name
    std::string output_json_path;
    for (int i = 1; i < 1000; i++) {
        output_json_path = output_dir_;
        for (int j = 0; j < 5-int(log10((double)i))-1; j++) {   // add 0 in front
            output_json_path += '0';
        }
        output_json_path += std::to_string(i) + ".json";    // final output path
        if (access(output_json_path.data(), 0) == -1) {
            break;
        }
    }
    std::cout << "Saving to " << output_json_path << std::endl;

    // compute mean and std dev
    this->addOrgAvgStddev();
    this->addSuccessCaseAvgStddev();
    /// write prettified JSON to output file
    // struct statistic to json
    std::vector<nlohmann::json> vector_json;
    for (statistics statistic : this->statistics_eval) {
        vector_json.push_back(statistic.to_json());
    }

    nlohmann::json error_json(vector_json);
    std::ofstream o(output_json_path);
    o << error_json << std::endl;
    std::cout << "Saved to " << output_json_path << std::endl;
    return true;
}

Evaluation::Evaluation() = default;
