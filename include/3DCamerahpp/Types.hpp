 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     Types.3DCamerahpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __TYPES_HPP__
#define __TYPES_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#define FRAMERATE_ANY 0

/** 
* @~chinese
* ö��: ���صĴ�����
* @~english
* enumeration: returned error code
**/
typedef enum ERROR_CODE
{
	SUCCESS = 0,					/**< @~chinese �ɹ�				@~english success*/ 
	ERROR_PARAM,					/**< @~chinese �����������		@~english param input error*/
	ERROR_DEVICE_NOT_FOUND,			/**< @~chinese δ�ҵ��豸		@~english device not found*/
	ERROR_DEVICE_NOT_CONNECT,		/**< @~chinese �豸δ����		@~english device not connected*/
	ERROR_DEVICE_BUSY,				/**< @~chinese �豸æ			@~english device busy*/
	ERROR_STREAM_NOT_START,			/**< @~chinese ����δ��		@~english stream not start*/
	ERROR_STREAM_BUSY,				/**< @~chinese ���Ѵ�			@~english stream had started*/
	ERROR_FRAME_TIMEOUT,			/**< @~chinese ��ȡ֡����ʧ��		@~english get frame failed*/
	ERROR_NOT_SUPPORT,				/**< @~chinese �в�֧��			@~english not support*/
	ERROR_PROPERTY_GET_FAILED,		/**< @~chinese ��ȡ����ʧ��		@~english get property failed*/
	ERROR_PROPERTY_SET_FAILED		/**< @~chinese ��������ʧ��		@~english set property failed*/
}ERROR_CODE;

/**
* @~chinese
* ö��: ���������
* @~english
* enumeration: stream type
**/
typedef enum STREAM_TYPE
{
	STREAM_TYPE_DEPTH	= 0, /**<@~chinese �����	@~english Depth CameraToOpen3d stream */
    STREAM_TYPE_RGB		= 1, /**<@~chinese RGB��		@~english RGB CameraToOpen3d stream */
	STREAM_TYPE_COUNT
}STREAM_TYPE;


/// \~chinese
/// \defgroup StreamFormat ��������ʽ
/// \brief �������RGB����֧�ֵ����и�ʽ
/// @{
/// \~english
/// \defgroup StreamFormat Stream format
/// \brief Format of depth stream and RGB stream
/// @{
/**
* @~chinese
* ö��: �����ݸ�ʽ
* @~english
* enumeration: stream format
**/
typedef enum STREAM_FORMAT
{
	STREAM_FORMAT_MJPG		= 0x00,		 /**< @~chinese RGB����MJPGѹ��������			
											  @~english MJPG compressed data*/ 
	STREAM_FORMAT_RGB8		= 0x01,		 /**< @~chinese RGB����8λ��,��,��3ͨ������			
											  @~english 8-bit red, green and blue channels*/ 
	STREAM_FORMAT_Z16		= 0x02,		 /**< @~chinese ����������ͼ��ʽ, ÿһ�����ֵ��unsigned short��ʾ
											  @~english 16-bit unsigned short depth values. The depth in millimeters is equal to depth scale * pixel value. */ 
	STREAM_FORMAT_Z16Y8Y8	= 0x03,		 /**< @~chinese ����������ͼ+����ͼ��ϸ�ʽ,	
														ͨ��FRAME_DATA_FORMAT_Z16���������ݣ�
														ͨ��FRAME_DATA_FORMAT_IR_LEFT��������ͼ, 
														ͨ��FRAME_DATA_FORMAT_IR_RIGHT����Һ���ͼ
											  @~english output depth map and infrared, 
														get depth map by FRAME_DATA_FORMAT_Z16
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/ 
	STREAM_FORMAT_PAIR		= 0x04,		 /**< @~chinese ������ĺ���ͼ��ʽ��
														ͨ��FRAME_DATA_FORMAT_IR_LEFT��������ͼ, 
														ͨ��FRAME_DATA_FORMAT_IR_RIGHT����Һ���ͼ	
											  @~english output infrared��
														get left infrared by FRAME_DATA_FORMAT_IR_LEFT,
														get right infrared by FRAME_DATA_FORMAT_IR_RIGHT*/ 
}STREAM_FORMAT;

/**
* @~chinese
* ö��: ֡���ݸ�ʽ�����ڻ�ȡ�����������е�ָ����ʽ������ʼ��ַ
* @~english
* enumeration: format of frame data, used for get specified data in a composite frame
**/
typedef enum FRAME_DATA_FORMAT
{
	FRAME_DATA_FORMAT_Z16				= 0x00,		/**< @~chinese ����������ͼ��ʽ, ÿһ�����ֵ��unsigned short��ʾ
														 @~english 16-bit unsigned short depth values. The depth in millimeters is equal to depth scale * pixel value. */ 
	FRAME_DATA_FORMAT_IR_LEFT			= 0x01,		/**< @~chinese �����ͼ���ݣ� 8-bit unsigned char��ʾһ���Ҷ�ֵ				
														 @~english 8-bit unsigned char gray level of left infrared*/
	FRAME_DATA_FORMAT_IR_RIGHT			= 0x02,		/**< @~chinese �Һ���ͼ���ݣ� unsigned char��ʾһ���Ҷ�ֵ				
														 @~english 8-bit unsigned char gray level of right infrared*/
}FRAME_DATA_FORMAT;
/// @}


/// \~chinese
/// \defgroup PropertyType ��������
/// \brief �о����п����õĻ�������
/// @{
/// \~english
/// \defgroup PropertyType Basic property
/// \brief List basic properties
/// @{

/**
* @~chinese
* ö��: ����Ļ�������
* @~english
* enumeration: basic property of CameraToOpen3d
**/
typedef enum PROPERTY_TYPE
{
	PROPERTY_GAIN						= 0x00,	/**<@~chinese ����				@~english gain_ of depth CameraToOpen3d or RGB CameraToOpen3d*/
	PROPERTY_EXPOSURE					= 0x01,	/**<@~chinese �ع�ֵ				@~english Controls exposure_ time of depth CameraToOpen3d or RGB CameraToOpen3d*/
	PROPERTY_FRAMETIME					= 0x02,	/**<@~chinese ֡ʱ��				@~english Frame time of depth CameraToOpen3d */
	PROPERTY_FOCUS						= 0x03,	/**<@~chinese ����				@~english Focus of RGB CameraToOpen3d*/
	PROPERTY_ENABLE_AUTO_FOCUS			= 0x04,	/**<@~chinese �Ƿ��Զ��Խ�		@~english Enable / disable auto-focus of RGB CameraToOpen3d*/
	PROPERTY_ENABLE_AUTO_EXPOSURE		= 0x05, /**<@~chinese �Ƿ��Զ��ع�		@~english Enable / disable auto-exposure_ of RGB CameraToOpen3d*/
	PROPERTY_ENABLE_AUTO_WHITEBALANCE	= 0x06, /**<@~chinese �Ƿ��Զ���ƽ��		@~english White balance of RGB CameraToOpen3d*/
	PROPERTY_WHITEBALANCE				= 0x07,	/**<@~chinese ��ƽ��ֵ			@~english adjust white balance of RGB CameraToOpen3d*/
	PROPERTY_WHITEBALANCE_R				= 0x08,	/**<@~chinese ��ƽ��Rͨ��		@~english Channel r of RGB CameraToOpen3d, adjust white balance*/
	PROPERTY_WHITEBALANCE_B				= 0x09,	/**<@~chinese ��ƽ��Bͨ��		@~english Channel b of RGB CameraToOpen3d, adjust white balance*/
	PROPERTY_WHITEBALANCE_G				= 0x10,	/**<@~chinese ��ƽ��Gͨ��		@~english Channel g of RGB CameraToOpen3d, adjust white balance*/
} PROPERTY_TYPE;
/// @}

/**
* @~chinese
* ö��: �������ģʽ 
* @~english
* enumeration: trigger mode
**/
typedef enum TRIGGER_MODE
{
	TRIGGER_MODE_OFF		= 0, /**< @~chinese �رմ���ģʽ��������������	
									  @~english output depth map continuously*/ 
	TRIGGER_MODE_HARDWAER	= 1, /**< @~chinese �ⴥ��ģʽ����Ҫ�ڴ���������Ӳ���źŲ��ܳ�ͼ
									  @~english external trigger mode��you should input hardware pulse to get depth frame*/
	TRIGGER_MODE_SOFTWAER	= 2, /**< @~chinese ����ģʽ����Ҫ����cs::ICamera::softTrigger���ܳ����ͼ
									  @~english software trigger mode��you should call cs::ICamera::softTrigger to get depth frame*/
}TRIGGER_MODE;

/**
* @~chinese
* ö��: �߶�̬��ģʽ
* @~english
* enumeration: mode of HDR
**/
typedef enum HDR_MODE
{
	HDR_MODE_OFF			= 0,	/**< @~chinese �ر�				@~english HDR off*/ 
	HDR_MODE_HIGH_RELECT	= 1,	/**< @~chinese �����ڲ�߷�����	@~english suitable for shiny object*/
	HDR_MODE_LOW_RELECT		= 2,	/**< @~chinese �����ڲ���ɫ����	@~english suitable for dark object*/
	HDR_MODE_ALL_RELECT		= 3		/**< @~chinese �����ڲ⸴�ϱ���	@~english suitable for composite object*/
}HDR_MODE;

/**
* @~chinese
* @brief ö��: �Զ��ع�ģʽ
* @~english
* @brief enumeration: mode of auto exposure_
**/
typedef enum AUTO_EXPOSURE_MODE
{
	AUTO_EXPOSURE_MODE_CLOSE = 0,			/**< @~chinese �ر�				@~english off*/
	AUTO_EXPOSURE_MODE_FIX_FRAMETIME = 1,	/**< @~chinese �ڲ��ı�֡�ʵ�ǰ�����Զ������ع�ʱ��
											@~english adjust exposure_ automatically and keep frame time unchanged*/
	AUTO_EXPOSURE_MODE_HIGH_QUALITY = 2		/**< @~chinese �Զ������ع�ʱ�䣬�ᰴ��ı�֡��
											@~english adjust exposure_ and frame time automatically*/
}AUTO_EXPOSURE_MODE;

/**
* @~chinese
* @brief ö��: ���紫��ѹ����ʽ
* @~english
* @brief enumeration: mode of compress
**/
typedef enum NETWORK_COMPRESS_MODE
{
	NETWORK_COMPRESS_MODE_CLOSE = 0,		/**< @~chinese �ر�				@~english off*/
	NETWORK_COMPRESS_MODE_ZIP	= 1,		/**< @~chinese ZIP(Ĭ������)     @~english ZIP(Default)*/
}NETWORK_COMPRESS_MODE;


/**
* @~chinese
* @brief ������ȷ�Χ��������Χ��ֵ��������
* @~english
* @brief range of depth�� value out of range will be set to zero
**/
typedef struct DepthRange
{
	int min;		/**< @~chinese �����Сֵ		@~english minimum of depth*/ 
	int max;		/**< @~chinese ������ֵ		@~english maximum of depth*/ 
}DepthRange;

/**
* @~chinese
* @brief ��������ʱ�豸��IP���ã���autoEnable����Ϊtrueʱ����������ipFourthByte
* @~english
* @brief IP setting��when autoEnable is true, there is no need to set ipFourthByte
**/
typedef struct IpSetting
{
	unsigned int autoEnable;	/**< @~chinese �Ƿ���DHCP		@~english enable/disable DHCP*/ 
	unsigned char ipFourthByte;	/**< @~chinese IP��ַ�ĵ���λ		@~english the fourth byte of ip*/ 
}IpSetting;

/**
* @~chinese
* @brief HDR�Զ�ģʽʱ�ع⼶���������ع�֮��ı�������
* @~english
* @brief exposure_ times and interstage scale of HDR
**/
typedef struct HdrScaleSetting
{
	unsigned int highReflectModeCount;	/**< @~chinese �߷�ģʽ�ع⼶��		@~english exposure_ times of high-reflective mode*/
	unsigned int highReflectModeScale;	/**< @~chinese �߷�ģʽ�����䱶��		@~english interstage scale of high-reflective mode*/ 
	unsigned int lowReflectModeCount;	/**< @~chinese ��ɫģʽ�ع⼶��		@~english exposure_ times of low-reflective mode*/
	unsigned int lowReflectModeScale;	/**< @~chinese ��ɫģʽ�����䱶��		@~english interstage scale of low-reflective mode*/ 
}HdrScaleSetting;

#pragma pack(push, 1)

/**
* @~chinese
* @brief HDRĳһ���ع�Ĳ���
* @~english
* @brief exposure_ param of HDR
**/
typedef struct HdrExposureParam
{
	unsigned int  exposure;	/**< @~chinese �ع�ʱ��			@~english exposure_ time*/
	unsigned char gain;		/**< @~chinese ����				@~english gain_*/
}HdrExposureParam;

/**
* @~chinese
* @brief HDR�ع����
* @~english
* @brief all exposure_ params of HDR
**/
typedef struct HdrExposureSetting
{
	unsigned char count;			/**< @~chinese ���ع⼶��		@~english total exposure_ times of HDR*/
	HdrExposureParam param[11];		/**< @~chinese �����ع����		@~english all params of HDR*/ 
}HdrExposureSetting;

#pragma pack(pop)


/// \~chinese
/// \defgroup PropertyExtensionType ��չ����
/// \brief �о����п����õ���չ����
/// @{
/// \~english
/// \defgroup PropertyExtensionType Extensional property
/// \brief List extensional properties
/// @{

/**
* @~chinese
* @brief ö��: ��չ����
* @~english
* @brief enumeration: extensional of property
**/
typedef enum PROPERTY_TYPE_EXTENSION
{
	PROPERTY_EXT_IP_SETTING				= 0x4,	 /**< @~chinese IP����				@~english IP setting*/
	PROPERTY_EXT_DEPTH_RANGE			= 0x707, /**< @~chinese ��ȷ�Χ				@~english depth range of CameraToOpen3d*/
	PROPERTY_EXT_HDR_MODE				= 0x914, /**< @~chinese HDRģʽ				@~english HDR mode*/
	PROPERTY_EXT_NETWORK_COMPRESS		= 0x5,	 /**< @~chinese ���紫��ʱ���Ƿ�ѹ��
													  @~english whether the stream compresses when transmited by network*/

	PROPERTY_EXT_HDR_SCALE_SETTING		= 0x915, /**< @~chinese HDR�Զ�ģʽ������		@~english setting of auto-HDR*/
	PROPERTY_EXT_HDR_EXPOSURE			= 0x916, /**< @~chinese HDR��������			@~english all params of HDR*/
	PROPERTY_EXT_AUTO_EXPOSURE_MODE		= 0x912, /**< @~chinese �������Զ��ع�ģʽ	@~english auto exposure_ mode of depth CameraToOpen3d*/
	PROPERTY_EXT_DEPTH_SCALE			= 0x0,	 /**< @~chinese ���ֵ����ϵ��		@~english depth unit for real distance */
	PROPERTY_EXT_TRIGGER_MODE 			= 0x1,	 /**< @~chinese ����ģʽ				@~english/PROPERTY_EXT_TRIGGER_OUT_MODE set trigger mode ,normal or trigge mode, value 1 stands for software trigger mode, value 2 stands for hardware trigger mode, other stands for trigger off(default)*/
	PROPERTY_EXT_CONTRAST_MIN			= 0x705, /**< @~chinese �Աȶ���ֵ			@~english remove where fringe contrast below this value*/
	PROPERTY_EXT_LED_ON_OFF			    = 0xb00, /**< @~chinese �Ƿ��LED��		@~english turn on/off led*/
} PROPERTY_TYPE_EXTENSION;
/// @}

/**
* @~chinese
* @brief ��չ����ֵ���������ʾ�����úͻ�ȡʱֻȡָ�����Զ�Ӧ���ֶμ���
* @~english
* @brief union of extensional property
**/
typedef union PropertyExtension
{
	float depthScale;							/**< @~chinese ��ӦPROPERTY_EXT_DEPTH_SCALE			@~english corresponding PROPERTY_EXT_DEPTH_SCALE			*/
	TRIGGER_MODE triggerMode;					/**< @~chinese ��ӦPROPERTY_EXT_TRIGGER_MODE			@~english corresponding PROPERTY_EXT_TRIGGER_MODE			*/
	int algorithmContrast;						/**< @~chinese ��ӦPROPERTY_EXT_CONTRAST_MIN			@~english corresponding PROPERTY_EXT_CONTRAST_MIN			*/
	AUTO_EXPOSURE_MODE autoExposureMode;		/**< @~chinese ��ӦPROPERTY_EXT_AUTO_EXPOSURE_MODE	@~english corresponding PROPERTY_EXT_AUTO_EXPOSURE_MODE	*/
	HdrScaleSetting hdrScaleSetting;			/**< @~chinese ��ӦPROPERTY_EXT_HDR_SCALE_SETTING	@~english corresponding PROPERTY_EXT_HDR_SCALE_SETTING	*/
	HdrExposureSetting hdrExposureSetting;		/**< @~chinese ��ӦPROPERTY_EXT_HDR_EXPOSURE			@~english corresponding PROPERTY_EXT_HDR_EXPOSURE			*/
	int ledOnOff;								/**< @~chinese ��ӦPROPERTY_EXT_LED_ON_OFF			@~english corresponding PROPERTY_EXT_LED_ON_OFF			*/
	HDR_MODE hdrMode;							/**< @~chinese ��ӦPROPERTY_EXT_HDR_MODE				@~english corresponding PROPERTY_EXT_HDR_MODE				*/
	DepthRange depthRange;						/**< @~chinese ��ӦPROPERTY_EXT_DEPTH_RANGE			@~english corresponding PROPERTY_EXT_DEPTH_RANGE			*/
	IpSetting ipSetting;						/**< @~chinese ��Ӧ��ӦPROPERTY_EXT_IP_SETTING		  @~english corresponding PROPERTY_EXT_IP_SETTING*/
	NETWORK_COMPRESS_MODE networkCompressMode;	/**< @~chinese ��ӦPROPERTY_EXT_NETWORK_COMPRESS		@~english corresponding PROPERTY_EXT_NETWORK_COMPRESS	*/
	int reserved[15];							/**< @~chinese Ԥ��									@~english reserved */
}PropertyExtension;								

/**
* @~chinese
* @brief ����Ϣ��ϣ����ڴ���ʱʹ�ã���ͨ��ICamera::getStreamInfos���
* @~english
* @brief stream information, returned by ICamera::getStreamInfos
**/
typedef struct StreamInfo
{
	STREAM_FORMAT format;	/**< @~chinese ����Ϣ		@~english stream format*/ 
	int width;				/**< @~chinese ���			@~english stream width*/
	int height;				/**< @~chinese �߶�			@~english stream height*/
	float fps;				/**< @~chinese ֡��			@~english stream framerate*/
}StreamInfo;

/**
* @~chinese
* @brief �����Ϣ����ͨ��ICamera::getInfo��ISystem::queryCameras���
* @~english
* @brief CameraToOpen3d informations, returned by ICamera::getStreamInfos or ISystem::queryCameras
**/
typedef struct CameraInfo
{
	char name[32];					/**< @~chinese �������			@~english type of CameraToOpen3d*/
	char serial[32];				/**< @~chinese ���к�			@~english serial number of CameraToOpen3d*/
	char uniqueId[32];				/**< @~chinese �����ʶ			@~english unique Id of CameraToOpen3d*/
	char firmwareVersion[32];		/**< @~chinese �̼��汾			@~english version of firmware*/
	char algorithmVersion[32];		/**< @~chinese �㷨�汾			@~english version of algorithm*/
}CameraInfo;

/**
* @~chinese
* @brief ����ڲ�
* @~english
* @brief Intrinsics of depth CameraToOpen3d or RGB CameraToOpen3d
**/
typedef struct Intrinsics
{
	short width;	/**< @~chinese �궨�ֱ���-���		@~english calibration resolution-width*/ 
	short height;	/**< @~chinese �궨�ֱ���-�߶�		@~english calibration resolution-height*/ 
	float fx;
	float zero01;
	float cx;
	float zeor10;
	float fy;
	float cy;
	float zeor20;
	float zero21;
	float one22;
}Intrinsics;

/**
* @~chinese
* @brief ��������RGB��������תƽ����Ϣ
* @~english
* @brief Rotation and translation offrom depth CameraToOpen3d to RGB CameraToOpen3d
**/
typedef struct Extrinsics
{
	float rotation[9];                           /**<@~chinese 3x3��ת����		@~english column-major 3x3 rotation matrix */
	float translation[3];                        /**<@~chinese 3Ԫ�ص�ƽ�ƾ���	@~english three-element translation vector */
}Extrinsics;

/**
* @~chinese
* @brief ��������RGB����������
* @~english
* @brief Distort of depth CameraToOpen3d or RGB CameraToOpen3d
**/
typedef struct Distort
{
	float k1;
	float k2;
	float k3;
	float k4;
	float k5;
}Distort;

#ifdef __cplusplus
}
#endif

#endif
