//
// Created by cheng on 9/24/21.
//

#include "statistics.h"

statistics::statistics() = default;

//nlohmann::json statistics::to_json_helper_(const statistic& statistic_) {
//    nlohmann::json j = nlohmann::json{
//            {"method", statistic_.method},
//            {"voxel_size", statistic_.voxel_size},
//            {"time", statistic_.time},
//            {"error_r", statistic_.error_r},
//            {"error_t", statistic_.error_t},
//    };
//    return j;
//}

nlohmann::json statistics::to_json() {
    nlohmann::json j;
//    nlohmann::json j = nlohmann::json{
//            {"method",  this->method},
//            {"src",     this->src},
//            {"time",    this->time},
//            {"error_r", this->error_r},
//            {"error_t", this->error_t},
//    };
    this->to_json_helper(j);
    return j;
}

void statistics::to_json_helper(nlohmann::json &j) {
    nlohmann::json j_add = nlohmann::json{
            {"method",      this->method},
            {"src",         this->src},
            {"voxel_size",  this->voxel_size},
            {"time",        this->time},
            {"error_r",     this->error_r},
            {"error_t",     this->error_t},
            {"statistics", std::vector<nlohmann::json>()}
    };
    std::vector<nlohmann::json> j_sub;
    for (statistics statistic_ : this->substatistics) {
        statistic_.to_json_helper(j_add);
    }
    if (j.empty()) {
        j = j_add;
    } else {
        j["statistics"].push_back(j_add);
    }
}
