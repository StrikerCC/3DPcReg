//
// Created by cheng on 9/16/21.
//

#include "FeatureCompute.h"

FeatureCompute::FeatureCompute() = default;

std::vector<PcNormalFpfh> FeatureCompute::getFPHFFeatures(const open3d::geometry::PointCloud &pc, const std::vector<float>& voxel_sizes) const {
    std::shared_ptr<open3d::geometry::PointCloud> pc_down = std::make_shared<open3d::geometry::PointCloud>(pc);
    std::vector<PcNormalFpfh> features = std::vector<PcNormalFpfh>();

    if (pc.points_.empty()) {
        return features;
    }

    for (auto voxel_size_iter = voxel_sizes.crbegin(); voxel_size_iter != voxel_sizes.crend(); ++voxel_size_iter) { // down-sampling from fine to coarse
        double voxel_size = *voxel_size_iter;
        double radius_normal = voxel_size * 2, radius_fpfh = voxel_size * 5;
        auto pc_down_ = pc_down->VoxelDownSample(voxel_size);
        pc_down_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, FeatureCompute::max_nn_normal));
        auto pc_down_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pc_down_,
                                                                                open3d::geometry::KDTreeSearchParamHybrid(
                                                                                        radius_fpfh,
                                                                                        FeatureCompute::max_nn_fpfh));
        PcNormalFpfh feature;
        feature.voxel_size = voxel_size;
        feature.pc = pc_down_;
        feature.fpfh = pc_down_fpfh;
        features.push_back(feature);
        pc_down = pc_down_;
    }
    std::reverse(std::begin(features), std::end(features)); // return feature from coarse to fine for registration
    return features;
}

std::vector<PcNormal> FeatureCompute::getNormalFeature(const open3d::geometry::PointCloud &pc, const std::vector<float> &voxel_sizes) const {
    std::shared_ptr<open3d::geometry::PointCloud> pc_down = std::make_shared<open3d::geometry::PointCloud>(pc);

    std::vector<PcNormal> features;
    if (pc.points_.empty()) {
        return features;
    }

    for (auto voxel_size_iter = voxel_sizes.crbegin(); voxel_size_iter != voxel_sizes.crend(); voxel_size_iter++) {// down-sampling from fine to coarse
        double voxel_size = *voxel_size_iter;
        double radius_normal = voxel_size * 2;
        auto pc_down_ = pc_down->VoxelDownSample(voxel_size);
        pc_down_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, FeatureCompute::max_nn_normal));
        PcNormal feature;
        feature.voxel_size = voxel_size;
        feature.pc = pc_down_;
        features.push_back(feature);
        pc_down = pc_down_;
    }
    std::reverse(std::begin(features), std::end(features)); // return feature from coarse to fine for registration
    return features;
}
