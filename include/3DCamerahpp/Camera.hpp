 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     Camera.3DCamerahpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <vector>
#include <memory>
#include "Frame.hpp"
#include "APIExport.hpp"

namespace cs
{
class ICamera;

/**
* @~chinese
* \defgroup Camera 相机操作
* @brief 提供相机连接，打开数据流，设置属性，读取参数等功能
* @{
* @~english
* \defgroup Camera Camera operations
* @brief Provide functions for CameraToOpen3d connection, start stream, set properties, read parameters and other functions
* @{
*/

/**
* @~chinese
* @brief 获得帧数据回调函数
* @~english
* @brief callback of get frame
*/
typedef void (*FrameCallback)(IFramePtr frame, void *usrData);

/**
* @~chinese
* @brief 相机对象的共享指针
* @~english
* @brief the shared pointer of CameraToOpen3d
*/
typedef std::shared_ptr<ICamera> ICameraPtr;
	
/*!\class ICamera
* @~chinese
* @brief 相机接口
* @~english
* @brief Camera interface
*/
class CS_API ICamera
{
public:

	virtual ~ICamera() {};

	/**
	* @~chinese
	* @brief     连接任意一个相机
	* @return    成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief     Connect to any CameraToOpen3d
	* @return    success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE connect() = 0;
	
	/**
	* @~chinese
	* @brief     连接指定信息的相机
	* @param[in] info			:指定的相机信息，可通过ISystem::queryCameras接口获得信息列表
	* @return    成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief     Connect to the specified CameraToOpen3d
	* @param[in] info			:the information of specified CameraToOpen3d，you can get the infomations by ISystem::queryCameras
	* @return    success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE connect(CameraInfo info) = 0;

    /**
	* @~chinese
	* @brief	  获取当前连接相机的信息，包括序列号，版本信息，标识Id
	* @param[out] info			:接收返回的相机信息
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief	  Get the information of 3DCamera,include serial number, version, unique id
	* @param[out] info			:information of connected CameraToOpen3d
	* @return     success:return SUCCESS, fail:other error code
	**/
	virtual ERROR_CODE getInfo(CameraInfo &info) = 0;

	/** 
	* @~chinese 
	* @brief	  断开相机连接
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english 
	* @brief	  Disconnect to CameraToOpen3d
	* @return     success:return SUCCESS, fail:other error code
	*/
    virtual ERROR_CODE disconnect() = 0;
    
    /**
	* @~chinese
	* @brief      获取指定类型的数据流所支持的流格式信息列表
	* @param[in]  streamType		：流类型
	* @param[out] streamInfos		：返回的流格式信息列表
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english 
	* @brief      Get all supported stream informations of the specified stream 
	* @param[in]  streamType		：the type of stream
	* @param[out] streamInfos		：return the list of supported stream informations
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getStreamInfos(STREAM_TYPE streamType, std::vector<StreamInfo> &streamInfos) = 0;

    /**
	* @~chinese
	* @brief      打开数据流并通过回调函数返回帧数据
	* @param[in]  streamType		：需要打开的流类型， 见STREAM_TYPE
	* @param[in]  info				：需要打开的流格式信息, 可由getStreamInfos返回
	* @param[in]  callback          : 返回帧数据的回调函数
	* @param[in]  userData          : 用户数据
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Start stream and return frame by callback
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[out] info      		：stream information, returned by getStreamInfos
	* @param[in]  callback          : frame callback
	* @param[in]  userData          : the user data
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE startStream(STREAM_TYPE streamType, StreamInfo info, FrameCallback callback, void *userData) = 0;

	/**
	* @~chinese
	* @brief      打开数据流并通过getFrame主动获取帧数据
	* @param[in]  streamType		：需要打开的流类型， 见STREAM_TYPE
	* @param[in]  info				：需要打开的流格式信息, 可由getStreamInfos返回
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Start stream without callback, you should get frame by getFrame
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[out] info      		：stream information, returned by getStreamInfos
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE startStream(STREAM_TYPE streamType, StreamInfo info) = 0;

    /**
	* @~chinese
	* @brief      停止数据流
	* @param[in]  streamType		：需要停止的流类型， 见STREAM_TYPE
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Stop stream
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE stopStream(STREAM_TYPE streamType) = 0;

	/**
	* @~chinese
	* @brief      主动获取当前流输出的帧数据
	* @param[in]  streamType		：需要获取的流类型
	* @param[out] frame				：返回帧数据
	* @param[in]  timeout_ms		：超时时间，单位为毫秒
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get frame manually
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[out] frame				：return the captured frame
	* @param[in]  timeout_ms		：timeout in millisecond
	* @return success:return SUCCESS, fail:other error code
	**/   
    virtual ERROR_CODE getFrame(STREAM_TYPE streamType, IFramePtr &frame, int timeout_ms = 5000) = 0;

	/**
	* @~chinese
	* @brief      软触发模式下触发N帧（暂时只支持1帧）
	* @param[in]  count				：触发帧数
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Trigger N frames in software trigger mode (only one frame is supported temporarily)
	* @param[in]  count				：the count of trigger times
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE softTrigger(int count = 1) = 0;

	/**
	* @~chinese
	* @brief      获取指定流的指定属性范围
	* @param[in]  streamType		：指定的数据流类型
	* @param[in]  propertyType		：属性类型
	* @param[out] min				：属性的最小值
	* @param[out] max				：属性的最大值
	* @param[out] step				：属性调整时的建议调节步进
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get the value range of property
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[in]  propertyType		：property type, @see PROPERTY_TYPE
	* @param[out] min				：the minimum of the property
	* @param[out] max				：the minimum of the property
	* @param[out] step				：the step of the property
	* @return success:return SUCCESS, fail:other error code
	**/
	virtual ERROR_CODE getPropertyRange(STREAM_TYPE streamType, PROPERTY_TYPE propertyType, float &min, float &max, float &step) = 0;	

	/**
	* @~chinese
	* @brief      获取指定流的属性值
	* @param[in]  streamType		：指定的数据流类型
	* @param[in]  propertyType		：属性类型
	* @param[out] value				：属性当前值
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get the value of property
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[in]  propertyType		：property type, @see PROPERTY_TYPE
	* @param[out] value				：the value of the property
	* @return success:return SUCCESS, fail:other error code
	**/
	virtual ERROR_CODE getProperty(STREAM_TYPE streamType, PROPERTY_TYPE propertyType, float &value) = 0;

	/**
	* @~chinese
	* @brief      修改指定流的属性值
	* @param[in]  streamType		：指定的数据流类型
	* @param[in]  propertyType		：属性类型
	* @param[in]  value				：需要设置的属性值
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Set the value of property
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[in]  propertyType		：property type, @see PROPERTY_TYPE
	* @param[in]  value				：the value of property
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE setProperty(STREAM_TYPE streamType, PROPERTY_TYPE propertyType, float value) = 0;	

	/**
	* @~chinese
	* @brief      修改扩展属性值
	* @param[in]  propertyType		：属性类型
	* @param[in]  value				：需要设置的属性值
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Set the value of extensional property
	* @param[in]  propertyType		：property type, @see PROPERTY_TYPE_EXTENSION
	* @param[in]  value				：the value of extensional property
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE setPropertyExtension(PROPERTY_TYPE_EXTENSION propertyType, PropertyExtension value) = 0;

	/**
	* @~chinese
	* @brief      获取扩展属性的当前值
	* @param[in]  propertyType		：属性类型
	* @param[out] value				：返回的值
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get the value of extensional property
	* @param[in]  propertyType		：property type, @see PROPERTY_TYPE_EXTENSION
	* @param[out] value				：return the value of extensional property
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getPropertyExtension(PROPERTY_TYPE_EXTENSION propertyType, PropertyExtension &value) = 0;
    
	/**
	* @~chinese
	* @brief      获取指定类型数据流的内参
	* @param[in]  streamType		：指定的数据流类型
	* @param[out] intrinsics		：返回内参
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get the intrinsic of specified stream
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[out] intrinsics		：return the intrinsic of specified stream
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getIntrinsics(STREAM_TYPE streamType, Intrinsics &intrinsics) = 0;
    
	/**
	* @~chinese
	* @brief      获取从深度流到RGB流的旋转平移参数
	* @param[out] extrinsics		：返回外参
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get the extrinsics from depth stream to RGB stream
	* @param[out] extrinsics		：return the extrinsics
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getExtrinsics(Extrinsics &extrinsics) = 0;
    
	/**
	* @~chinese
	* @brief      获取指定类型数据流的畸变参数
	* @param[in]  streamType		：指定的数据流类型
	* @param[out] distort			：返回内参
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Get the distort of the specified stream
	* @param[in]  streamType		：stream type, @see STREAM_TYPE
	* @param[out] distort			：return the distort of the specified stream
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getDistort(STREAM_TYPE streamType, Distort &distort) = 0;

	/**
	* @~chinese
	* @brief	  重启相机
	* @return 成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief	  reboot CameraToOpen3d
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE restart() = 0;

	/**
	* @~chinese
	* @brief      写入用户自定义数据，长度必须要小于1024字节
	* @param[in]  userData			：写入数据指针
	* @param[in]  length			：写入数据长度
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Write the user defined data to CameraToOpen3d, at most 1024 bytes
	* @param[in]  userData			：the pointer of user defined data
	* @param[in]  length			：the length of user defined data
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE setUserData(char *userData, int length) = 0;

	/**
	* @~chinese
	* @brief      读取用户自定义数据，长度必须要小于1024字节
	* @param[in]  userData			：保存数据区的指针
	* @param[out] length			：返回数据长度
	* @return     成功:SUCCESS, 失败:其他错误码
	* @~english
	* @brief      Write the user defined data to CameraToOpen3d, at most 1024 bytes
	* @param[in]  userData			：the pointer of buffer to save user defined data
	* @param[in]  length			：return the length of user defined data
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getUserData(char *userData, int &length) = 0;

};

CS_API ICameraPtr getCameraPtr();
/*@} */
}
#endif