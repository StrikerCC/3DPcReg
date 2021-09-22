//
// Created by cheng on 9/16/21.
//

#include "PointCloudReader.h"

PointCloudReader::PointCloudReader() = default;

//PointCloudReader::~PointCloudReader() {}

bool PointCloudReader::loadModel(std::string file_path) {
    if (PointCloudReader::safe_model) {
        auto pc_down = PointCloudReader::readPointCloud(file_path, PointCloudReader::voxel_size_read_pc);
        if (!pc_down || pc_down->IsEmpty()) {
            return false;
        }
    }
    PointCloudReader::model_paths.push_back(file_path);
    return true;
}

bool PointCloudReader::loadOneFrame(std::string file_path) {
    if (PointCloudReader::safe_model) {
        auto pc_down = PointCloudReader::readPointCloud(file_path, PointCloudReader::voxel_size_read_pc);
        if (!pc_down || pc_down->IsEmpty()) {
            return false;
        }
    }
    PointCloudReader::frame_paths.push_back(file_path);
    return true;
}

bool PointCloudReader::loadFramesAndPoseFromDir(std::string frames_dir_path, std::string poses_dir_path) {
    // TODO
    return false;
}

sourceTargetAndPose PointCloudReader::getModelAndOneFrame(int index_frame) {
    assert(index_frame < PointCloudReader::frame_paths.size());

    if (PointCloudReader::model_paths.empty()) {
        std::cout << "No model loaded" << std::endl;
        return sourceTargetAndPose{};
    } else if (PointCloudReader::frame_paths.empty()) {
        std::cout << "No frames loaded" << std::endl;
        return sourceTargetAndPose{};
    } else {
        sourceTargetAndPose output;
        output.src = PointCloudReader::readPointCloud(PointCloudReader::model_paths.at(index_frame), PointCloudReader::voxel_size_read_pc);
        output.tgt = PointCloudReader::readPointCloud(PointCloudReader::frame_paths.at(index_frame), PointCloudReader::voxel_size_read_pc);
        output.pose = !PointCloudReader::poses.empty() ? PointCloudReader::poses.at(index_frame) : Eigen::Matrix4d::Identity();
        return output;
    }
}

std::shared_ptr<open3d::geometry::PointCloud> PointCloudReader::readPointCloud(std::string file_path,
                                                                               double voxel_size_downsampling) {
    /* loadModel point cloud from file */
    std::shared_ptr<open3d::geometry::PointCloud> pc_down;
    open3d::geometry::PointCloud pc = open3d::geometry::PointCloud();
    open3d::io::ReadPointCloud(file_path, pc);
    //    pc.Scale(1000.0, pc.GetCenter());
    if (!pc.points_.empty()) {
        /* processing point cloud from file */
        pc_down = pc.VoxelDownSample(voxel_size_downsampling);
    } else {
        return nullptr;
    }
    return pc_down;
}

bool PointCloudReader::loadOnePose(std::string pose_file_path) {
    std::ifstream in(pose_file_path, std::ios::in);
    if (!in) return false;
    Eigen::Matrix4d pose = Eigen::Matrix4d();
    for (int i=0;  i<4; ++i) {
        for (int j=0; j<4; ++j) {
            float num;
            in >> num;
            pose(i, j) = num;
        }
    }
    PointCloudReader::poses.push_back(pose);
    return true;
}

Eigen::Matrix4d PointCloudReader::readPose(std::string pose_file_path) {
    Eigen::Matrix4d pose = Eigen::Matrix4d();
    return pose;
}

int PointCloudReader::get_length() {
    return int(PointCloudReader::frame_paths.size());
}

//PointCloudReader::~PointCloudReader() {}

///////////////////////////////////////////////////////////////////////////

PointCloudReaderFromJson::PointCloudReaderFromJson() = default;

//PointCloudReaderFromJson::~PointCloudReaderFromJson() {}

bool PointCloudReaderFromJson::loadJson(std::string file_path) {
    std::ifstream input_file(file_path);
    input_file >> PointCloudReaderFromJson::sources;

    // load source, target, pose accordingly
    for (auto iter = PointCloudReaderFromJson::sources.cbegin(); iter < PointCloudReaderFromJson::sources.cend(); ++iter) {
//        std::cout << *iter << std::endl;

        std::string src_path = std::string (iter->at("pc_artificial"));
//        src_path = PointCloudReaderFromJson::root_path + src_path.substr(1);
        src_path = PointCloudReaderFromJson::root_path + src_path;

        std::string tgt_path = std::string (iter->at("pc_model"));
        tgt_path = PointCloudReaderFromJson::root_path + tgt_path.substr(1);

        PointCloudReaderFromJson::frame_paths.push_back(src_path);
        PointCloudReaderFromJson::model_paths.push_back(tgt_path);

        auto pose = iter->at("pose");
        Eigen::Matrix4d pose_matrix = Eigen::Matrix4d();
//        std::vector<std::vector<float>> pose_matrix;

        for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++) {
                float num;
                pose[i][j].get_to(num);
                pose_matrix(i, j) = num;
            }
        }
        PointCloudReaderFromJson::poses.push_back(pose_matrix);
    }
    return true;
}
