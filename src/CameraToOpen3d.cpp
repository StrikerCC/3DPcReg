//
// Created by cheng on 10/26/21.
//

#include "CameraToOpen3d.h"

CameraToOpen3d::CameraToOpen3d() {
    /// set extensional properties
    PropertyExtension value;

    value.depthRange.min = CameraToOpen3d::depthRange_min_;      // PROPERTY_EXT_DEPTH_RANGE
    value.depthRange.max = CameraToOpen3d::depthRange_max_;
    value.algorithmContrast = CameraToOpen3d::algorithm_contrast_;    // the threshold of algorithm

    float exposure = CameraToOpen3d::exposure_;          // exposure_ time (micro second)
    float gain = CameraToOpen3d::gain_;                 // sensor gain_ (intensity of inferred laser ?)

    ERROR_CODE ret;                 // camera error code, help debug camera

    /// get CameraToOpen3d pointer and connect a valid CameraToOpen3d
    CameraToOpen3d::camera = cs::getCameraPtr();
    ret = CameraToOpen3d::camera->connect();
    if (ret == SUCCESS) {
        std::cout << "Camera on line" << std::endl;
    } else {
        printf("CameraToOpen3d connect failed(%d)! Aborting\n", ret);
        CameraToOpen3d::working_normal_ = false;
        return;
    }

    /// get camera info and display
    CameraInfo info;
    ret = camera->getInfo(info);
    if (ret == SUCCESS) {
        // display informations of camera
        printf("%20s  :  %s\n", "name", info.name);
        printf("%20s  :  %s\n", "serial", info.serial);
        printf("%20s  :  %s\n", "unique id", info.uniqueId);
        printf("%20s  :  %s\n", "firmware version", info.firmwareVersion);
        printf("%20s  :  %s\n", "algorithm version", info.algorithmVersion);
        printf("\n");
    } else {
        printf("camera get info failed(%d)! Aborting\n", ret);
        CameraToOpen3d::working_normal_ = false;
        return;
    }

    /// get depth stream info and check
    std::vector<StreamInfo> streamInfos;
    ret = CameraToOpen3d::camera->getStreamInfos(STREAM_TYPE_DEPTH, streamInfos);
    if (ret == SUCCESS) {
        // display information of depth-stream
        for (auto streamInfo : streamInfos) {
            printf("depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
        }
        printf("\n");
    } else {
        printf("CameraToOpen3d get depth stream failed(%d)! Aborting\n", ret);
        CameraToOpen3d::working_normal_ = false;
        return;
    }

    /// start depth stream
    for (auto streamInfo : streamInfos) {
        if (streamInfo.format == STREAM_FORMAT_Z16) {
            //start stream
            ret = CameraToOpen3d::camera->startStream(STREAM_TYPE_DEPTH, streamInfo);
            if (ret == SUCCESS) {
                printf("start depth format:%2d, width:%4d, height:%4d, fps:%2.1f\n", streamInfo.format, streamInfo.width, streamInfo.height, streamInfo.fps);
            } else {
                printf("camera start depth stream failed(%d)! Aborting\n", ret);
                return;
            }
            break;
        }
    }

    /// set exposure_ and double check
    ret = CameraToOpen3d::SetExposure(exposure);
    if (ret != SUCCESS) {
        printf("set exposure_ time to %f failed(%d)! Aborting\n", exposure, ret);
        CameraToOpen3d::working_normal_ = false;
        return;
    }

    /// set gain_ and double check
    ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, gain);

    /// set depth range
    ret = CameraToOpen3d::camera->setPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, value);

    /// set min contrast
    ret = CameraToOpen3d::camera->setPropertyExtension(PROPERTY_EXT_CONTRAST_MIN, value);

    CameraToOpen3d::working_normal_ = true;
}

CameraToOpen3d::~CameraToOpen3d() {
    // disconnect CameraToOpen3d
    CameraToOpen3d::camera->stopStream(STREAM_TYPE_DEPTH);
    ERROR_CODE ret = CameraToOpen3d::camera->disconnect();
    if(ret != SUCCESS)
    {
        printf("CameraToOpen3d disconnect failed(%d)!\n", ret);
        return;
    }
}

std::shared_ptr<open3d::geometry::PointCloud> CameraToOpen3d::GetNewFrame(const std::string& file_path_temp) const{
    assert(CameraToOpen3d::WorkingProperly());
    assert(file_path_temp.substr(-4, 4) == ".ply");
    std::string file_path_temp_ = file_path_temp.empty() ? "./pc_temp.ply" : file_path_temp;
    open3d::geometry::PointCloud pc = open3d::geometry::PointCloud();
    if (CameraToOpen3d::SaveNewFrame(file_path_temp_) == 1) {
        open3d::io::ReadPointCloud(file_path_temp_, pc);
    } else {
        std::cout << "Couldn't get new frame" << std::endl;
    }

    /// filter to proper range
    return FilterPointsOutBound(pc, CameraToOpen3d::min_bound_pc_tgt, CameraToOpen3d::max_bound_pc_tgt);
}

int CameraToOpen3d::SaveNewFrame(const std::string& file_path) const {
    if (!CameraToOpen3d::WorkingProperly()) return 0;

    /// save a frame
    bool success = false;
    int count = 5;

    /// loop some times to make sure newest frame received
    for (int i = 0; i < count; i++) {
        std::cout << i << " ";
        std::remove(file_path.data());

        /// try to get a new frame
        Intrinsics intr;
        CameraToOpen3d::camera->getIntrinsics(STREAM_TYPE_DEPTH, intr);
//        if (SUCCESS != CameraToOpen3d::camera->getIntrinsics(STREAM_TYPE_DEPTH, intr)) {
//            printf("camera->getIntrinsics: No Frame Receive!\n");
//            continue;
//        }
        cs::IFramePtr frameDepth;

        if (SUCCESS != CameraToOpen3d::camera->getFrame(STREAM_TYPE_DEPTH, frameDepth)) {
            printf("camera->getFrame: No Frame Receive!\n");
            continue;
        }
        printf("Get a new Frame!\n");

        /// extract point cloud from the new frame
        cs::Pointcloud pc;
        float scale = 0.1f;
        PropertyExtension value;
        if (SUCCESS == CameraToOpen3d::camera->getPropertyExtension(PROPERTY_EXT_DEPTH_SCALE, value)) {
            scale = value.depthScale;
        }
        // generate point cloud without color
        pc.generatePoints((unsigned short *)frameDepth->getData(), frameDepth->getWidth(), frameDepth->getHeight(), scale, &intr, nullptr, nullptr);
        pc.exportToFile(file_path, nullptr, 0, 0, true);
        success = true;
    }
    return success;
}

ERROR_CODE CameraToOpen3d::SetExposure(float exposure_) {
    ERROR_CODE ret;
    float targetExposure = exposure_;
    //read the frame time(the maximum exposure_ time)
    float exposureMax = 7000;
    float exposure;
    ret = camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, exposureMax);
    if (ret == SUCCESS) {
        //If you need to set the exposure_ time which is greater than current frame time,
        //you should set frame time first
        if (targetExposure > exposureMax) {
            camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, targetExposure);
        }
        //set exposure_ value
        ret = camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, targetExposure);
        if (ret == SUCCESS) {
            camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, exposure);
            printf("set exposure_ time to %f success\n", exposure);
        }
    }
    return ret;
}

//ERROR_CODE CameraToOpen3d::SetGain(float gain_) {
//    ERROR_CODE ret = CameraToOpen3d::camera->setProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, gain_);
//    if (ret == SUCCESS) {
//        camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, gain_);
//        printf("set gain_ to %f success\n", gain_);
//    } else {
//        printf("set gain_ to %f failed(%d)! Aborting\n", gain_, ret);
//        CameraToOpen3d::working_normal_ = false;
//    }
//    return ret;
//}


