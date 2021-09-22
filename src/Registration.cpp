//
// Created by cheng on 9/17/21.
//

#include "Registration.h"

Registration::Registration() = default;


float Registration::register_ransac_icp(const open3d::geometry::PointCloud& source,
                                       const open3d::geometry::PointCloud& target,
                                       const Eigen::Matrix4d& tf_gt) {
    // vis
    if (Registration::visualize) {
        Registration::DrawReg(source, target);
    }
    std::cout << "Original Source has " << source.points_.size() << " points" << std::endl;
    std::cout << "Original Target has " << target.points_.size() << " points" << std::endl;

    /// prepare registration parameters
    clock_t clock_start;
    float time_features_global {0.0}, time_features_local {0.0}, time_reg_global {0.0}, time_reg_local {0.0};
    std::vector<double> primary_voxel_size_global = Registration::voxel_size_global;
    std::vector<double> primary_voxel_size_local = Registration::voxel_size_local;
    SolveCorrespondenceAndRigidTransformation solver = SolveCorrespondenceAndRigidTransformation();

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
        tf_global = solver.solve_ransac(src_pc, src_feature, tgt_pc, tgt_feature, voxel_size, tf_global);

        // vis
        if (Registration::visualize) {
            time_reg_global += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
            Registration::DrawReg(*src_pc, *tgt_pc, tf_global, "Global registration result #" + std::to_string(i) + " voxel size " + std::to_string(voxel_size));
            clock_start = clock();
        }
    }

    time_reg_global += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
//    std::cout << tf_global << std::endl;

    // vis
    if (Registration::visualize) {
        Registration::DrawReg(source, target, tf_global, "Global registration final result");
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
        tf_local = solver.solve_icp(src_pc, tgt_pc, voxel_size, tf_local);

        std::cout << "ICP iteration "  << i << " voxel size " << voxel_size << std::endl;
        std::cout << "      Source has " << src_pc->points_.size() << " points" << std::endl;
        std::cout << "      Target has " << tgt_pc->points_.size() << " points" << std::endl;

        // vis
        if (Registration::visualize) {
            time_reg_local += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
            Registration::DrawReg(*src_pc, *tgt_pc, tf_local, "Local registration #" + std::to_string(i) + " voxel size " + std::to_string(voxel_size));
            clock_start = clock();
        }
    }

    time_reg_local += (float) (clock() - clock_start) / CLOCKS_PER_SEC;
//    std::cout << tf_local << std::endl;
//    std::cout << tf_gt << std::endl;

    // vis
    if (Registration::visualize) {
        Registration::DrawReg(source, target, tf_local, "Local registration final result");
    }
    auto error = ComputeRegError(tf_gt, tf_local);
    std::cout << "Time global feature computing " << time_features_global << std::endl;
    std::cout << "Time global registration      " << time_reg_global << std::endl;
    std::cout << "Time local feature computing  " << time_features_local << std::endl;
    std::cout << "Time global registration      " << time_reg_local << std::endl;
    std::cout << "Rotation error            " << error.error_rotation << " degree" << std::endl;
    std::cout << "Translation error         " << error.error_translation << "  mm" << std::endl;

    // compute error if gt was given

//    std::cout << "Frame # " << Registration_mix::num_frame << " takes seconds" << time_global+time_local << "\n   global takes " << time_global << " seconds" << "\n   local takes " << time_local << " seconds" << std::endl;
//    if (pose_gt != nullptr) {
//        std::tuple<float, float> error_rotation_translation = Registration_mix::ComputeRegError(Registration_mix::pose_current_global, *pose_gt);
//        std::cout << "Error " << std::endl;
//        std::cout << " Rotation   " << std::to_string(std::get<0>(error_rotation_translation)) << std::endl;
//        std::cout << " Translation" << std::to_string(std::get<0>(error_rotation_translation)) << std::endl;
//        std::cout << std::endl;
//    }

    return time_features_global + time_features_local + time_reg_global + time_reg_local;
}

bool Registration::DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                           const Eigen::Matrix4d &transformation, const std::string &win_name) const {
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

//bool Registration::DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
//                           const std::string &win_name) const {
//    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
//    Registration::DrawReg(source, target, transformation, win_name) ;
//    return true;
//}
