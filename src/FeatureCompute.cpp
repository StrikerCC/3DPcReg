//
// Created by cheng on 9/16/21.
//

#include "FeatureCompute.h"

FeatureCompute::FeatureCompute() = default;

std::vector<PcNormalFpfh> FeatureCompute::getFPHFFeatures(const open3d::geometry::PointCloud &pc, const std::vector<double>& voxel_sizes) const {
    std::vector<PcNormalFpfh> features = std::vector<PcNormalFpfh>();
    if (pc.points_.empty()) {
        return features;
    }
    // TODO: use getNormalFeature instead
    for (double voxel_size : voxel_sizes) {
        double radius_normal = voxel_size * 2, radius_fpfh = voxel_size * 5;
        auto pc_down = pc.VoxelDownSample(voxel_size);
        pc_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, FeatureCompute::max_nn_normal));
        auto pc_down_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pc_down,
                                                                                open3d::geometry::KDTreeSearchParamHybrid(
                                                                                        radius_fpfh,
                                                                                        FeatureCompute::max_nn_fpfh));
        PcNormalFpfh feature;
        feature.voxel_size = voxel_size;
        feature.pc = pc_down;
        feature.fpfh = pc_down_fpfh;
        features.push_back(feature);
    }
    return features;
}

std::vector<PcNormal> FeatureCompute::getNormalFeature(const open3d::geometry::PointCloud &pc, const std::vector<double> &voxel_sizes) const {
    std::vector<PcNormal> features;
    if (pc.points_.empty()) {
        return features;
    }
    for (double voxel_size : voxel_sizes) {
        double radius_normal = voxel_size * 2;
        auto pc_down = pc.VoxelDownSample(voxel_size);
        pc_down->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, FeatureCompute::max_nn_normal));
        PcNormal feature;
        feature.voxel_size = voxel_size;
        feature.pc = pc_down;
        features.push_back(feature);
    }
    return features;
}
