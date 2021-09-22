//
// Created by cheng on 9/16/21.
//

#ifndef INC_3DPCREG_FEATURECOMPUTE_H
#define INC_3DPCREG_FEATURECOMPUTE_H

#include "open3d/Open3D.h"

struct PcNormalFpfh {
    double voxel_size;
    std::shared_ptr<open3d::geometry::PointCloud> pc;
    std::shared_ptr<open3d::pipelines::registration::Feature> fpfh;
};

struct PcNormal {
    double voxel_size;
    std::shared_ptr<open3d::geometry::PointCloud> pc;
};

class FeatureCompute {
public:
    FeatureCompute();
    std::vector<PcNormalFpfh> getFPHFFeatures(const open3d::geometry::PointCloud& pc, const std::vector<double>& voxel_sizes) const;
    std::vector<PcNormal> getNormalFeature(const open3d::geometry::PointCloud& pc, const std::vector<double>& voxel_sizes) const;

private:
//    std::vector<double> voxel_sizes_down_sampling {{5, 3, 1}};
    const int max_nn_normal = 30;
    const int max_nn_fpfh = 100;
};


#endif //INC_3DPCREG_FEATURECOMPUTE_H
