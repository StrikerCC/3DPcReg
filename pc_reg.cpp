#include "pc_reg.h"

pc_reg::pc_reg(/* args */) {
    pc_reg::visualize = true;
    pc_reg::num_frame = 0;
    pc_reg::voxel_size_read_pc = 0.5;
    pc_reg::voxel_size_global_reg = 5;
    pc_reg::voxel_size_local_reg = 0.8;

    pc_reg::model_global.voxel_size=voxel_size_global_reg;
    pc_reg::model_local.voxel_size=voxel_size_local_reg;
    pc_reg::frame_current_global.voxel_size=voxel_size_global_reg;
    pc_reg::frame_current_local.voxel_size=voxel_size_local_reg;
}

pc_reg::~pc_reg() = default;

bool pc_reg::Register(const Eigen::Matrix4d* pose_gt) {
//    Eigen::Matrix4d tf_global = Eigen::Matrix4d::Identity();

    // vis
    if (pc_reg::visualize) {
        pc_reg::DrawReg("Global registration result");
    }

    clock_t start_time_global, start_time_local;
    float time_global {0.0}, time_local {0.0};
    start_time_global = clock();
    if (pc_reg::GlobalRegister()) {
        time_global = (float) (clock() - start_time_global)/CLOCKS_PER_SEC;
        // vis
        if (pc_reg::visualize) {
            pc_reg::DrawReg("Global registration result");
        }
    } else {
        return false;
    }

    start_time_local = clock();
    if (pc_reg::LocalRegister()) {
        time_local = (float) (clock() - start_time_local)/CLOCKS_PER_SEC;
        // vis
        if (pc_reg::visualize) {
            pc_reg::DrawReg("Global registration result");
        }
    } else {
        return false;
    }

    // compute error if gt was given
//    std::cout << "Frame # " << pc_reg::num_frame << " takes seconds" << time_global+time_local << "\n   global takes " << time_global << " seconds" << "\n   local takes " << time_local << " seconds" << std::endl;
    if (pose_gt != nullptr) {
        std::tuple<float, float> error_rotation_translation = pc_reg::ComputeRegError(pc_reg::pose_current_global, *pose_gt);
        std::cout << "Error " << std::endl;
        std::cout << " Rotation   " << std::to_string(std::get<0>(error_rotation_translation)) << std::endl;
        std::cout << " Translation" << std::to_string(std::get<0>(error_rotation_translation)) << std::endl;
        std::cout << std::endl;
    }
    return true;
}

bool pc_reg::GlobalRegister() {
//    int max_iteration = 4000000, max_validation = 1000;
    int max_iteration = 4000000;
    float max_validation = 0.999;
    auto max_correspondence_dis = pc_reg::model_global.voxel_size;
    auto distance_threshold = pc_reg::model_global.voxel_size;

    auto source = *pc_reg::model_global.pc, target = *pc_reg::frame_current_global.pc;
    auto source_fpfh = *pc_reg::model_global.fpfh, target_fpfh = *pc_reg::frame_current_global.fpfh;

    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_dege_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
            0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_dege_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    correspondence_checker.emplace_back(correspondence_checker_normal);
    auto registration_result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(source, target,
                                                                                                         source_fpfh,
                                                                                                         target_fpfh,
                                                                                                         max_correspondence_dis,
                                                                                                         open3d::pipelines::registration::TransformationEstimationPointToPoint(
                                                                                                                 false),
                                                                                                         3,
                                                                                                         correspondence_checker,
                                                                                                         open3d::pipelines::registration::RANSACConvergenceCriteria(
                                                                                                                 max_iteration,
                                                                                                                 max_validation));
    // overload old pose if this reg may valid
    if (pc_reg::IsRegValid(registration_result.transformation_)) {
        pc_reg::pose_current_global = registration_result.transformation_;
        pc_reg::pose_current = registration_result.transformation_;
        return true;
    } else {
        return false;
    }
}

bool pc_reg::LocalRegister() {
    Eigen::Matrix4d tf_global = pc_reg::pose_current_global;
    int max_iteration = 4000000, max_validation = 1000;
    auto max_correspondence_dis = pc_reg::model_global.voxel_size * 1.5;
    auto source = *pc_reg::model_global.pc, target = *pc_reg::frame_current_global.pc;
    auto source_fpfh = *pc_reg::model_global.fpfh, target_fpfh = *pc_reg::frame_current_global.fpfh;

    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_dege_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
            0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(0.075);
    auto correspondence_checker_normal = open3d::pipelines::registration::CorrespondenceCheckerBasedOnNormal(
            0.52359878);
    correspondence_checker.emplace_back(correspondence_checker_dege_length);
    correspondence_checker.emplace_back(correspondence_checker_distance);
    correspondence_checker.emplace_back(correspondence_checker_normal);
    auto registration_result = open3d::pipelines::registration::RegistrationICP(source, target,
                                                                                max_correspondence_dis,
                                                                                tf_global,
                                                                                open3d::pipelines::registration::TransformationEstimationPointToPoint(
                                                                                        false),
                                                                                open3d::pipelines::registration::ICPConvergenceCriteria(
                                                                                        max_iteration,
                                                                                        max_validation));

    // overload old pose if this reg may valid
    if (pc_reg::IsRegValid(registration_result.transformation_)) {
        pc_reg::pose_current_local = registration_result.transformation_;
        pc_reg::pose_current = registration_result.transformation_;
        return true;
    } else {
        return false;
    }
}

bool pc_reg::LoadModel(const char *file_name) {
    /* global feature parameters */
    float radius_normal_global = pc_reg::model_global.voxel_size * 2, radius_fpfh_global = pc_reg::model_global.voxel_size * 5;
    int max_nn_normal_global = 30, max_nn_fpfh_local = 100;

    /* local feature parameters */
    float radius_normal_local = pc_reg::model_local.voxel_size * 2;
    int max_nn_normal_local = 30;

    /* read point cloud from file */
    open3d::geometry::PointCloud pc = open3d::geometry::PointCloud();
    open3d::io::ReadPointCloud(file_name, pc);
//    pc.Scale(1000.0, pc.GetCenter());

    if (!pc.points_.empty()) {
        /* processing point cloud from file */
        auto pc_down = pc.VoxelDownSample(pc_reg::voxel_size_read_pc);

        /* processing point cloud for global reg */
        auto pc_down_global = pc.VoxelDownSample(pc_reg::model_global.voxel_size);
        pc_down_global->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(radius_normal_global, max_nn_normal_global));
        auto pc_down_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pc_down_global,
                                                                                open3d::geometry::KDTreeSearchParamHybrid(
                                                                                        radius_fpfh_global,
                                                                                        max_nn_fpfh_local));
        pc_reg::model_global.pc = pc_down_global;
        pc_reg::model_global.fpfh = pc_down_fpfh;

        /* processing point cloud for local reg */
        auto pc_down_local = pc.VoxelDownSample(pc_reg::model_local.voxel_size);
        pc_down_local->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(radius_normal_local, max_nn_normal_local));
        pc_reg::model_local.pc = pc_down_local;

        /* cout */
        std::cout << "Model point cloud: " << std::endl;
        std::cout << "  Read " << std::to_string(pc.points_.size()) << " points" << std::endl;
        std::cout << "  Initial down to " << std::to_string(pc_down->points_.size()) << " points" << std::endl;
        std::cout << "  Global down to " << std::to_string(pc_down_global->points_.size()) << " points" << std::endl;
        std::cout << "  Local down to " << std::to_string(pc_down_local->points_.size()) << " points" << std::endl;

        /* vis */
//        open3d::visualization::DrawGeometries({std::make_shared<open3d::geometry::PointCloud>(pc)});
        open3d::visualization::DrawGeometries({pc_reg::model_global.pc});
        return !pc_down->points_.empty() and !pc_down_global->points_.empty() and !pc_down_local->points_.empty();
    } else {
        return false;
    }
}

bool pc_reg::TakeNewFrame(const char *file_name) {
    /* global feature parameters */
    float radius_normal_global = pc_reg::frame_current_global.voxel_size * 2, radius_fpfh_global = pc_reg::frame_current_global.voxel_size * 5;
    int max_nn_normal_global = 30, max_nn_fpfh_local = 100;

    /* local feature parameters */
    float radius_normal_local = pc_reg::frame_current_local.voxel_size * 2;
    int max_nn_normal_local = 30;

    /* read point cloud from file */
    float time_read {0.0};
    clock_t clock_start = clock();

    open3d::geometry::PointCloud pc = open3d::geometry::PointCloud();
    open3d::io::ReadPointCloud(file_name, pc);
    time_read = ((float) (clock()-clock_start)) / ((float) CLOCKS_PER_SEC);
    std::cout << "      read " << time_read << std::endl;

    if (!pc.points_.empty()) {
        clock_start = clock();

        /* processing point cloud from file */
//        auto pc_down = pc.VoxelDownSample(pc_reg::voxel_size_read_pc);
//        auto pc_down = pc;

        /* processing point cloud for global reg */
        auto pc_down_global = pc.VoxelDownSample(pc_reg::frame_current_global.voxel_size);

        pc_down_global->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(radius_normal_global, max_nn_normal_global));
        auto pc_down_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pc_down_global,
                                                                                open3d::geometry::KDTreeSearchParamHybrid(
                                                                                        radius_fpfh_global,
                                                                                        max_nn_fpfh_local));
        pc_reg::frame_current_global.pc = pc_down_global;
        pc_reg::frame_current_global.fpfh = pc_down_fpfh;

        time_read = ((float) (clock()-clock_start)) / ((float) CLOCKS_PER_SEC);
        std::cout << "      FPFH process " << time_read << std::endl;
        clock_start = clock();

        /* processing point cloud for local reg */
        auto pc_down_local = pc_down_global->VoxelDownSample(pc_reg::frame_current_local.voxel_size);
        pc_down_local->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(radius_normal_local, max_nn_normal_local));
        pc_reg::frame_current_local.pc = pc_down_local;

        time_read = ((float) (clock()-clock_start)) / ((float) CLOCKS_PER_SEC);
        std::cout << "      XYZNORMAL process " << time_read << std::endl;

        /* cout */
        std::cout << "Frame point cloud: " << std::endl;
        std::cout << "  Read " << std::to_string(pc.points_.size()) << " points" << std::endl;
//        std::cout << "  Initial down to " << std::to_string(pc_down->points_.size()) << " points" << std::endl;
        std::cout << "  Global down to " << std::to_string(pc_down_global->points_.size()) << " points" << std::endl;
        std::cout << "  Local down to " << std::to_string(pc_down_local->points_.size()) << " points" << std::endl;
//        if (!pc_down->points_.empty() and !pc_down_global->points_.empty() and !pc_down_local->points_.empty()) {
        if (!pc_down_global->points_.empty() and !pc_down_local->points_.empty()) {
            num_frame++;
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}
bool pc_reg::IsRegValid(const Eigen::Matrix4d &transformation) const {
    return CloseEnough(transformation.block<3, 3>(0, 0).determinant(), 1.0);
}

bool pc_reg::DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                     const Eigen::Matrix4d &transformation, const std::string &win_name) const {
    if (source.points_.empty() or target.points_.empty()) {
        return false;
    }
    std::shared_ptr<open3d::geometry::PointCloud> source_transformed_ptr(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_ptr(new open3d::geometry::PointCloud);
    *source_transformed_ptr = source;
    source_transformed_ptr->Transform(transformation);
    source_transformed_ptr->PaintUniformColor({0.0, 0.0, 1.0});
    *target_ptr = target;
    open3d::visualization::DrawGeometries({source_transformed_ptr, target_ptr}, win_name);
    return true;
}

bool pc_reg::DrawReg(const std::string &win_name) const {
    return pc_reg::DrawReg(*pc_reg::model_global.pc, *pc_reg::frame_current_global.pc, pc_reg::pose_current, win_name);
}

bool pc_reg::showBall() {
    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    sphere->ComputeVertexNormals();
    sphere->PaintUniformColor({0.0, 1.0, 0.0});
    open3d::visualization::DrawGeometries({sphere, sphere});
    return true;
}

std::tuple<float, float> pc_reg::ComputeRegError(const Eigen::Matrix4d &pose_1, const Eigen::Matrix4d &pose_2) const {
    Eigen::Matrix3d rotation_error = pose_1.block<3, 3>(0, 0) * pose_2.block<3 ,3>(0, 0).transpose();
    Eigen::Vector3d translation_error = pose_1.block<3, 1>(0, 3) - pose_2.block<3, 1>(0, 3);
    return std::make_tuple(rotation_error.norm(), translation_error.norm());
}

int pc_reg::get_num_frame() {
    return pc_reg::num_frame;
}
