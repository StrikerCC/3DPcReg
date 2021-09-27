//
// Created by cheng on 9/24/21.
//

#include "statistics.h"

statistics::statistics() = default;

nlohmann::json statistics::to_json() {
    nlohmann::json j;
    this->to_json_helper(j);
    return j;
}

void statistics::to_json_helper(nlohmann::json &j) {
    nlohmann::json j_add = nlohmann::json{
            {"method",      this->method},
            {"src",         this->src},
            {"num_points_src", this->num_points_src},
            {"num_points_tgt", this->num_points_tgt},
            {"voxel_size",  this->voxel_size},
            {"noise_src",   this->noise_src},
            {"time",        this->time},
            {"error_r",     this->error_r},
            {"error_t",     this->error_t},
            {"statistics_step", std::vector<nlohmann::json>()}
    };

    // add children json node
    for (statistics statistic_ : this->substatistics) {
        statistic_.to_json_helper(j_add);
    }
    if (j.empty()) {    // root json node
        j = j_add;      // root is current json node
    } else {            // non-root json node
        j["statistics_step"].push_back(j_add);   // append current json node to children list
    }
}
