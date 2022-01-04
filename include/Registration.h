//
// Created by cheng on 9/17/21.
//

#ifndef INC_3DPCREG_REGISTRATION_H
#define INC_3DPCREG_REGISTRATION_H

#include <cmath>
//#include <iostream>
//#include <fstream>
#include <vector>
//#include <unordered_map>
#include "nlohmann/json.hpp"
#include <Eigen/Core>

#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
#include "FeatureCompute.h"
//#include "SolveCorrespondenceAndRigidTransformation.h"
#include "utils.h"

struct Registration_Statistics{
    std::shared_ptr<open3d::pipelines::registration::RegistrationResult> global_reg_result;
    std::shared_ptr<open3d::pipelines::registration::RegistrationResult> local_reg_result;
    nlohmann::json time;
};

class Registration {
public:
//    void register_ransac_icp(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target, double voxel_size,
//                                                    const Eigen::Matrix4d& tf_gt=Eigen::Matrix4d::Identity());


    Registration();

    Registration_Statistics register_ransac_icp(const std::shared_ptr<open3d::geometry::PointCloud> &pc_ptr_src,
                             const std::shared_ptr<open3d::geometry::PointCloud> &pc_ptr_tgt);

private:
    bool visualize = true;

    std::vector<double> voxel_size_global {6};
    std::vector<double> voxel_size_local {5, 3, 1, 0.5};
//    std::vector<verification::statistic_reg> statistics;


    open3d::pipelines::registration::RegistrationResult ransac (
            std::shared_ptr<open3d::geometry::PointCloud> const& src,
            std::shared_ptr<open3d::pipelines::registration::Feature> const& src_feature,
            std::shared_ptr<open3d::geometry::PointCloud> const& tgt,
            std::shared_ptr<open3d::pipelines::registration::Feature> const& tgt_feature,
            double voxel_size);

    open3d::pipelines::registration::RegistrationResult icp(std::shared_ptr<open3d::geometry::PointCloud> const& src,
                                                            std::shared_ptr<open3d::geometry::PointCloud> const& tgt,
                                                            double voxel_size,
                                                            const Eigen::Matrix4d& tf = Eigen::Matrix4d::Identity());

};


#endif //INC_3DPCREG_REGISTRATION_H
