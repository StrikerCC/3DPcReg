#include <string>
#include <ctime>
//#include <boost/smart_ptr.hpp>
#include "Eigen/Core"
#include "open3d/Open3D.h"
#include "open3d/geometry/PointCloud.h"
#include "utils.h"

struct PcFpfh {
    float voxel_size;
    std::shared_ptr<open3d::geometry::PointCloud> pc;
    std::shared_ptr<open3d::pipelines::registration::Feature> fpfh;
};

struct Pc {
    float voxel_size;
    std::shared_ptr<open3d::geometry::PointCloud> pc;
};

class pc_reg {
private:
    /* data */
    bool visualize;

    int num_frame;
    float voxel_size_read_pc;
    float voxel_size_global_reg;
    float voxel_size_local_reg;

    struct PcFpfh model_global;
    struct Pc model_local;

    struct PcFpfh frame_current_global;
    struct Pc frame_current_local;

public:
    pc_reg(/* args */);
    ~pc_reg();
    bool showBall();
    bool DrawReg(const open3d::geometry::PointCloud &source, const open3d::geometry::PointCloud &target,
                 const Eigen::Matrix4d &transformation, const std::string& win_name="pc_reg") const;
    bool DrawReg(const std::string& win_name="Current Registration Result") const;

    bool LoadModel(const char* file_name);
    bool TakeNewFrame(const char *file_name);

    bool Register(const Eigen::Matrix4d* pose_gt=nullptr);
    bool GlobalRegister();
    bool LocalRegister();

    std::tuple<float, float> ComputeRegError(const Eigen::Matrix4d &pose_1, const Eigen::Matrix4d &pose_2) const;

    Eigen::Matrix4d pose_current_global = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_current_local = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_current = Eigen::Matrix4d::Identity();

    bool IsRegValid(const Eigen::Matrix4d &transformation) const;
    int get_num_frame();
};


