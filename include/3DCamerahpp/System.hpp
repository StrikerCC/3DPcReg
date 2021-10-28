 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     System.3DCamerahpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __SYSTEM_HPP__
#define __SYSTEM_HPP__

#include <vector>
#include <memory>
#include "Types.hpp"
#include "APIExport.hpp"

namespace cs
{
class ISystem;

/**
* @~chinese
* \defgroup System ������
* @brief �����������������䶯�ص�
* @{
* @~english
* \defgroup System Camera monitor
* @brief Discover CameraToOpen3d, monitor CameraToOpen3d change
* @{
*/

/** @brief CameraToOpen3d state change callback */
typedef void (*CameraChangeCallback)(std::vector<CameraInfo>& addedCameras, std::vector<CameraInfo>& removedCameras, void * userData);

typedef std::shared_ptr<ISystem> ISystemPtr;

/*!\class ISystem
* @~chinese
* @brief ϵͳ�ӿ�
* @~english 
* @brief System interface
**/
class CS_API ISystem
{
public:

	virtual ~ISystem() {};

	/**
	* @~chinese
	* @brief      ��ȡ��ǰ�����ӵ�����б�
	* @param[out] cameras			��������Ч����б�
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Query valid 3d cameras
	* @param[out] cameras		    ��return valid 3d cameras
	* @return success:return SUCCESS, fail:other error code
	**/
    virtual ERROR_CODE queryCameras(std::vector<CameraInfo> &cameras) = 0;

	/**
	* @~chinese
	* @brief      �����������״̬�䶯�ص�
	* @param[in]  callback		���ص�����
	* @param[in]  userData		���û�����ָ��
	* @return     �ɹ�:SUCCESS, ʧ��:����������
	* @~english
	* @brief      Set CameraToOpen3d state change callback
	* @param[in]  callback		��CameraToOpen3d state change callback
	* @param[in]  userData		��pointer of user data
	* @return success:return SUCCESS, fail:other error code
	**/ 
    virtual ERROR_CODE setCameraChangeCallback(CameraChangeCallback callback, void *userData) = 0;

};

CS_API std::shared_ptr<ISystem> getSystemPtr();
/*@} */
}
#endif
