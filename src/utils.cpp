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

namespace verification {

    nlohmann::json to_json_helper_(const statistic& statistic_) {
        nlohmann::json j = nlohmann::json{
                {"method", statistic_.method},
                {"voxel_size", statistic_.voxel_size},
                {"time", statistic_.time},
                {"error_r", statistic_.error_r},
                {"error_t", statistic_.error_t},
        };
        return j;
    }

    void to_json(nlohmann::json &j, const statistic_reg &statistics_) {
        j = nlohmann::json{
                {"src",           statistics_.src},
                {"time_total",    statistics_.time_total},
                {"error_r_final", statistics_.error_r_final},
                {"error_t_final", statistics_.error_t_final},
        };

        std::vector<nlohmann::json> statistics_json;
        for (const verification::statistic& statistic_ : statistics_.statistics) {
            statistics_json.push_back(verification::to_json_helper_(statistic_));
        }
        j["statistics"] =statistics_json;
    }
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
