 /*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     Frame.3DCamerahpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2019 / 08 / 17
*
*****************************************************************************/
#ifndef __FRAME_HPP__
#define __FRAME_HPP__

#include <vector>
#include <memory>
#include "Types.hpp"
#include "APIExport.hpp"

namespace cs
{
class IFrame;
/**
* @~chinese
* \defgroup Frame ����֡���� 
* @brief ����֡����ӿ�
* @{
* @~english
* \defgroup Frame Frame Operations
* @brief Frame object interface
* @{
*/
typedef std::shared_ptr<IFrame> IFramePtr;

/*!\class IFrame
* @~chinese
* @brief ����֡�ӿ�
* @~english
* @brief Frame interface
*/
class CS_API IFrame
{
public:

	virtual ~IFrame() {};

	/** 
	* @~chinese		
	* @brief		�ж�֡�Ƿ�Ϊ��
	* @ return ���֡Ϊ�շ���true, ���򷵻�false
	* @~english
	* @brief		Check frame is valid
	* @return true if frame is empty��otherwise return false
	*/
	virtual const bool empty() const = 0;

	/**
	* @~chinese
	* @brief      ��ȡ��֡���ݵ�ʱ�������λΪ����
	* @~english
	* @brief      Retrieve the time at which the frame was captured in milliseconds
	**/
	virtual const double getTimeStamp() const = 0;

	/**
	* @~chinese
	* @brief      ��ȡ��֡���ݵ��׵�ַָ��
	* @~english
	* @brief      Retrieve the pointer to the start of the frame data
	**/
	virtual const char *getData() const = 0;

	/**
	* @~chinese
	* @brief      ��ȡ��֡����ָ�����ֵ���ʼָ��
	* @~english
	* @brief      Retrieve the pointer to the start of the specified format data
	**/
	virtual const char *getData(FRAME_DATA_FORMAT format) const = 0;
	
	/**
	* @~chinese
	* @brief      ��ȡ��֡���ݵ��ֽ���
	* @~english
	* @brief      Retrieve size of frame in bytes
	**/
	virtual const int getSize() const = 0;

	/**
	* @~chinese
	* @brief      ��ȡ��֡���
	* @~english
	* @brief      Retrieve frame width in pixels
	**/
	virtual const int getWidth() const = 0;

	/**
	* @~chinese
	* @brief      ��ȡ��֡�߶�
	* @~english
	* @brief      Retrieve frame height in pixels
	**/
	virtual const int getHeight() const = 0;

	/**
	* @~chinese
	* @brief      ��ȡ֡�ĸ�ʽ
	* @~english
	* @brief      Retrieve format of frame
	**/
	virtual STREAM_FORMAT getFormat() const = 0;
};
/*@} */
}
#endif