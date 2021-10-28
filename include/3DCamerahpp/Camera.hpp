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
* \defgroup Camera �������
* @brief �ṩ������ӣ������������������ԣ���ȡ�����ȹ���
* @{
* @~english
* \defgroup Camera Camera operations
* @brief Provide functions for CameraToOpen3d connection, start stream, set properties, read parameters and other functions
* @{
*/

/**
* @~chinese
* @brief ���֡���ݻص�����
* @~english
* @brief callback of get frame
*/
typedef void (*FrameCallback)(IFramePtr frame, void *usrData);

/**
* @~chinese
* @brief �������Ĺ���ָ��
* @~english
* @brief the shared pointer of CameraToOpen3d
*/
typedef std::shared_ptr<ICamera> ICameraPtr;
	
/*!\class ICamera
* @~chinese
* @brief ����ӿ�
* @~english
* @brief Camera interface
*/
class CS_API ICamera
{
public:

	virtual ~ICamera() {};

	/**
	* @~chinese
	* @brief     ��������һ�����
	* @return    �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief     Connect to any CameraToOpen3d
	* @return    success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE connect() = 0;
	
	/**
	* @~chinese
	* @brief     ����ָ����Ϣ�����
	* @param[in] info			:ָ���������Ϣ����ͨ��ISystem::queryCameras�ӿڻ����Ϣ�б�
	* @return    �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief     Connect to the specified CameraToOpen3d
	* @param[in] info			:the information of specified CameraToOpen3d��you can get the infomations by ISystem::queryCameras
	* @return    success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE connect(CameraInfo info) = 0;

    /**
	* @~chinese
	* @brief	  ��ȡ��ǰ�����������Ϣ���������кţ��汾��Ϣ����ʶId
	* @param[out] info			:���շ��ص������Ϣ
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief	  Get the information of 3DCamera,include serial number, version, unique id
	* @param[out] info			:information of connected CameraToOpen3d
	* @return     success:return SUCCESS, fail:other error code
	**/
	virtual ERROR_CODE getInfo(CameraInfo &info) = 0;

	/** 
	* @~chinese 
	* @brief	  �Ͽ��������
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english 
	* @brief	  Disconnect to CameraToOpen3d
	* @return     success:return SUCCESS, fail:other error code
	*/
    virtual ERROR_CODE disconnect() = 0;
    
    /**
	* @~chinese
	* @brief      ��ȡָ�����͵���������֧�ֵ�����ʽ��Ϣ�б�
	* @param[in]  streamType		��������
	* @param[out] streamInfos		�����ص�����ʽ��Ϣ�б�
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english 
	* @brief      Get all supported stream informations of the specified stream 
	* @param[in]  streamType		��the type of stream
	* @param[out] streamInfos		��return the list of supported stream informations
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getStreamInfos(STREAM_TYPE streamType, std::vector<StreamInfo> &streamInfos) = 0;

    /**
	* @~chinese
	* @brief      ����������ͨ���ص���������֡����
	* @param[in]  streamType		����Ҫ�򿪵������ͣ� ��STREAM_TYPE
	* @param[in]  info				����Ҫ�򿪵�����ʽ��Ϣ, ����getStreamInfos����
	* @param[in]  callback          : ����֡���ݵĻص�����
	* @param[in]  userData          : �û�����
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Start stream and return frame by callback
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[out] info      		��stream information, returned by getStreamInfos
	* @param[in]  callback          : frame callback
	* @param[in]  userData          : the user data
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE startStream(STREAM_TYPE streamType, StreamInfo info, FrameCallback callback, void *userData) = 0;

	/**
	* @~chinese
	* @brief      ����������ͨ��getFrame������ȡ֡����
	* @param[in]  streamType		����Ҫ�򿪵������ͣ� ��STREAM_TYPE
	* @param[in]  info				����Ҫ�򿪵�����ʽ��Ϣ, ����getStreamInfos����
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Start stream without callback, you should get frame by getFrame
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[out] info      		��stream information, returned by getStreamInfos
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE startStream(STREAM_TYPE streamType, StreamInfo info) = 0;

    /**
	* @~chinese
	* @brief      ֹͣ������
	* @param[in]  streamType		����Ҫֹͣ�������ͣ� ��STREAM_TYPE
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Stop stream
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE stopStream(STREAM_TYPE streamType) = 0;

	/**
	* @~chinese
	* @brief      ������ȡ��ǰ�������֡����
	* @param[in]  streamType		����Ҫ��ȡ��������
	* @param[out] frame				������֡����
	* @param[in]  timeout_ms		����ʱʱ�䣬��λΪ����
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get frame manually
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[out] frame				��return the captured frame
	* @param[in]  timeout_ms		��timeout in millisecond
	* @return success:return SUCCESS, fail:other error code
	**/   
    virtual ERROR_CODE getFrame(STREAM_TYPE streamType, IFramePtr &frame, int timeout_ms = 5000) = 0;

	/**
	* @~chinese
	* @brief      ����ģʽ�´���N֡����ʱֻ֧��1֡��
	* @param[in]  count				������֡��
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Trigger N frames in software trigger mode (only one frame is supported temporarily)
	* @param[in]  count				��the count of trigger times
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE softTrigger(int count = 1) = 0;

	/**
	* @~chinese
	* @brief      ��ȡָ������ָ�����Է�Χ
	* @param[in]  streamType		��ָ��������������
	* @param[in]  propertyType		����������
	* @param[out] min				�����Ե���Сֵ
	* @param[out] max				�����Ե����ֵ
	* @param[out] step				�����Ե���ʱ�Ľ�����ڲ���
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get the value range of property
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[in]  propertyType		��property type, @see PROPERTY_TYPE
	* @param[out] min				��the minimum of the property
	* @param[out] max				��the minimum of the property
	* @param[out] step				��the step of the property
	* @return success:return SUCCESS, fail:other error code
	**/
	virtual ERROR_CODE getPropertyRange(STREAM_TYPE streamType, PROPERTY_TYPE propertyType, float &min, float &max, float &step) = 0;	

	/**
	* @~chinese
	* @brief      ��ȡָ����������ֵ
	* @param[in]  streamType		��ָ��������������
	* @param[in]  propertyType		����������
	* @param[out] value				�����Ե�ǰֵ
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get the value of property
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[in]  propertyType		��property type, @see PROPERTY_TYPE
	* @param[out] value				��the value of the property
	* @return success:return SUCCESS, fail:other error code
	**/
	virtual ERROR_CODE getProperty(STREAM_TYPE streamType, PROPERTY_TYPE propertyType, float &value) = 0;

	/**
	* @~chinese
	* @brief      �޸�ָ����������ֵ
	* @param[in]  streamType		��ָ��������������
	* @param[in]  propertyType		����������
	* @param[in]  value				����Ҫ���õ�����ֵ
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Set the value of property
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[in]  propertyType		��property type, @see PROPERTY_TYPE
	* @param[in]  value				��the value of property
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE setProperty(STREAM_TYPE streamType, PROPERTY_TYPE propertyType, float value) = 0;	

	/**
	* @~chinese
	* @brief      �޸���չ����ֵ
	* @param[in]  propertyType		����������
	* @param[in]  value				����Ҫ���õ�����ֵ
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Set the value of extensional property
	* @param[in]  propertyType		��property type, @see PROPERTY_TYPE_EXTENSION
	* @param[in]  value				��the value of extensional property
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE setPropertyExtension(PROPERTY_TYPE_EXTENSION propertyType, PropertyExtension value) = 0;

	/**
	* @~chinese
	* @brief      ��ȡ��չ���Եĵ�ǰֵ
	* @param[in]  propertyType		����������
	* @param[out] value				�����ص�ֵ
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get the value of extensional property
	* @param[in]  propertyType		��property type, @see PROPERTY_TYPE_EXTENSION
	* @param[out] value				��return the value of extensional property
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getPropertyExtension(PROPERTY_TYPE_EXTENSION propertyType, PropertyExtension &value) = 0;
    
	/**
	* @~chinese
	* @brief      ��ȡָ���������������ڲ�
	* @param[in]  streamType		��ָ��������������
	* @param[out] intrinsics		�������ڲ�
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get the intrinsic of specified stream
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[out] intrinsics		��return the intrinsic of specified stream
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getIntrinsics(STREAM_TYPE streamType, Intrinsics &intrinsics) = 0;
    
	/**
	* @~chinese
	* @brief      ��ȡ���������RGB������תƽ�Ʋ���
	* @param[out] extrinsics		���������
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get the extrinsics from depth stream to RGB stream
	* @param[out] extrinsics		��return the extrinsics
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getExtrinsics(Extrinsics &extrinsics) = 0;
    
	/**
	* @~chinese
	* @brief      ��ȡָ�������������Ļ������
	* @param[in]  streamType		��ָ��������������
	* @param[out] distort			�������ڲ�
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Get the distort of the specified stream
	* @param[in]  streamType		��stream type, @see STREAM_TYPE
	* @param[out] distort			��return the distort of the specified stream
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getDistort(STREAM_TYPE streamType, Distort &distort) = 0;

	/**
	* @~chinese
	* @brief	  �������
	* @return �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief	  reboot CameraToOpen3d
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE restart() = 0;

	/**
	* @~chinese
	* @brief      д���û��Զ������ݣ����ȱ���ҪС��1024�ֽ�
	* @param[in]  userData			��д������ָ��
	* @param[in]  length			��д�����ݳ���
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Write the user defined data to CameraToOpen3d, at most 1024 bytes
	* @param[in]  userData			��the pointer of user defined data
	* @param[in]  length			��the length of user defined data
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE setUserData(char *userData, int length) = 0;

	/**
	* @~chinese
	* @brief      ��ȡ�û��Զ������ݣ����ȱ���ҪС��1024�ֽ�
	* @param[in]  userData			��������������ָ��
	* @param[out] length			���������ݳ���
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Write the user defined data to CameraToOpen3d, at most 1024 bytes
	* @param[in]  userData			��the pointer of buffer to save user defined data
	* @param[in]  length			��return the length of user defined data
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE getUserData(char *userData, int &length) = 0;

};

CS_API ICameraPtr getCameraPtr();
/*@} */
}
#endif