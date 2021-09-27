//
// Created by cheng on 9/24/21.
//

#ifndef INC_3DPCREG_STATISTICS_H
#define INC_3DPCREG_STATISTICS_H
#include <iostream>
#include <string>
#include <vector>
#include "nlohmann/json.hpp"

class statistics {
public:
    nlohmann::json to_json();

    std::string method;
    std::string src;
    double voxel_size = 0.0;
    double time = 0.0;
    double error_r = 0.0;
    double error_t = 0.0;
    std::vector<statistics> substatistics;

    statistics();

private:

    void to_json_helper(nlohmann::json &j_add);
};


#endif //INC_3DPCREG_STATISTICS_H
