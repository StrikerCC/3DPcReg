//
// Created by cheng on 9/22/21.
//

#ifndef INC_3DPCREG_EVALUATION_H
#define INC_3DPCREG_EVALUATION_H

#pragma once
#include "nlohmann/json.hpp"
#include <unistd.h>
//#include "Registration.h"
#include "PointCloudReader.h"
#include "SolveCorrespondenceAndRigidTransformation.h"
#include "utils.h"
#include "FeatureCompute.h"
#include "statistics.h"

class Evaluation {
public:
    Evaluation();
    statistics register_ransac_icp(const open3d::geometry::PointCloud& source,
                        const open3d::geometry::PointCloud& target,
                        const Eigen::Matrix4d& tf_gt);

    bool recordError(const statistics& statistic_reg_);
    bool save(const std::string& output_json_path = "");

private:

    bool visualize = false;
    std::vector<double> voxel_size_global {4};
    std::vector<double> voxel_size_local {5, 3, 1, 0.4};
    double error_r_threshold = 3.0, error_t_threshold = 5.0;
    std::vector<statistics> statistics_eval;

    bool addOrgAvgStddev();
    bool addSuccessCaseAvgStddev();
};





#endif //INC_3DPCREG_EVALUATION_H
