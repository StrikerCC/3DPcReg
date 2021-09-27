//
// Created by cheng on 9/16/21.
//

#ifndef INC_3DPCREG_SOLVECORRESPONDENCEANDRIGIDTRANSFORMATION_H
#define INC_3DPCREG_SOLVECORRESPONDENCEANDRIGIDTRANSFORMATION_H

#include "Eigen/Eigen"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
#include "utils.h"
#include "features.h"

class SolveCorrespondenceAndRigidTransformation {
public:
    SolveCorrespondenceAndRigidTransformation();
    open3d::pipelines::registration::RegistrationResult solve_icp(std::shared_ptr<open3d::geometry::PointCloud> const& src,
                   std::shared_ptr<open3d::geometry::PointCloud> const& tgt,
                   double voxel_size,
                   const Eigen::Matrix4d& tf=Eigen::Matrix4d::Identity());

    open3d::pipelines::registration::RegistrationResult solve_ransac(std::shared_ptr<open3d::geometry::PointCloud> const& src,
                      std::shared_ptr<open3d::pipelines::registration::Feature> const&  src_feature,
                      std::shared_ptr<open3d::geometry::PointCloud> const& tgt,
                      std::shared_ptr<open3d::pipelines::registration::Feature> const& tgt_feature,
                      float voxel_size,
                      const Eigen::Matrix4d& tf=Eigen::Matrix4d::Identity());

private:

};


#endif //INC_3DPCREG_SOLVECORRESPONDENCEANDRIGIDTRANSFORMATION_H
