/*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     HandEyeInput.3DCamerahpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2020 / 05 / 09
*
*****************************************************************************/
#ifndef __HANDEYEINPUT_HPP__
#define __HANDEYEINPUT_HPP__

#include <vector>
#include "hpp/APIExport.hpp"

namespace cs
{
/**
* \ingroup HandEye
* @{
*/

/**
* @~chinese
* @brief ö��: ��ǰλ�˷���ı�ʾ����
* @~english
* @brief enumeration: The type of direction
**/
typedef enum RobotPoseType
{
	POSE_TYPE_EULER_ZYZ = 0,	/**< @~chinese ŷ��ZYZ		@~english EULER ZYZ*/ 
	POSE_TYPE_EULER_XYZ = 1,	/**< @~chinese ŷ��XYZ		@~english EULER XYZ*/ 
	POSE_TYPE_EULER_ZYX = 2,	/**< @~chinese ŷ��ZYX		@~english EULER ZYX*/ 
	POSE_TYPE_RPY		= 3,	/**< @~chinese RPY			@~english RPY*/ 
	POSE_TYPE_QUATERNION = 4,	/**< @~chinese ��Ԫ����ʾ��	@~english Quaternion*/
	POSE_TYPE_XYZAB		= 5,	/**< @~chinese �����������	@~english used by 5-axis machine*/
}RobotPoseType;

/**
* @~chinese
* @brief ö��: ��ǰλ�˷���ı�ʾ��λ
* @~english
* @brief enumeration: The unit of direction
**/
typedef enum RobotPoseUnit
{
	POSE_UNIT_RADIAN = 0,	/**<@~chinese ����	@~english in radian */
	POSE_UNIT_DEGREE		/**<@~chinese �Ƕ�	@~english in degree */
}RobotPoseUnit;

/**
* @~chinese
* @brief ������λ��
* @~english
* @brief pose in hand
**/
typedef struct RobotPose
{
	float x;        /**<@~chinese ��λ����	@~english unit mm */
	float y;        /**<@~chinese ��λ����	@~english unit mm */
	float z;        /**<@~chinese ��λ����	@~english unit mm */
	float alfa;     /**<@~chinese ��̬��ʾԪ��1	@~english pose representation element 1 */
	float beta;     /**<@~chinese ��̬��ʾԪ��2	@~english pose representation element 2 */
	float gamma;    /**<@~chinese ��̬��ʾԪ��3	@~english pose representation element 3 */
	float theta;    /**<@~chinese ��̬��ʾԪ��4	@~english pose representation element 4 */
}RobotPose;

typedef struct Point3f {
	float x;
	float y;
	float z;
}Point3f;

typedef struct Point2f {
	float x;
	float y;
}Point2f;

/*!\class RobotPoseMatrix4f
* @~chinese
* @brief ������λ�˵�4x4�����ʾ
* @~english
* @brief pose of robot in matrix 4x4
**/
class CS_API RobotPoseMatrix4f
{
public:
	RobotPoseMatrix4f();
	/**
	* @~chinese
	* @brief      ͨ�������˵�ǰ��λ�˼�λ�˱�ʾ���͹���4x4����
	* @param[in]  pose				�����뵱ǰλ��
	* @param[in]  type				��λ�˱�ʾ����
	* @param[in]  unit				: λ�˱�ʾ�Ƕ�/����
	* @~english
	* @brief      Start stream and return frame by callback
	* @param[in]  pose				��input pose
	* @param[in]  type				��type of pose
	* @param[in]  unit				: unit of direction(radian/degree)
	**/
	RobotPoseMatrix4f(RobotPose pose, RobotPoseType type, RobotPoseUnit unit);
	float r00;
	float r01;
	float r02;
	float tx;
	float r10;
	float r11;
	float r12;
	float ty;
	float r20;
	float r21;
	float r22;
	float tz;
	float zero0;
	float zero1;
	float zero2;
	float one;
};

/**
* @~chinese
* @brief ���۱궨����
* @~english
* @brief eye to hand matrix
**/
typedef struct HandEyeMatrix
{
	float r00;
	float r01;
	float r02;
	float tx;
	float r10;
	float r11;
	float r12;
	float ty;
	float r20;
	float r21;
	float r22;
	float tz;
	float zero0;
	float zero1;
	float zero2;
	float one;
}HandEyeMatrix;

/*!\class HandEyeCalibrationInput
* @~chinese
* @brief ���۱궨������
* @~english
* @brief calibration input
**/
class CS_API HandEyeCalibrationInput
{
public:
	RobotPoseMatrix4f m_pose;
	std::vector<Point3f> m_points;
public:
	HandEyeCalibrationInput(HandEyeCalibrationInput&& other);

	HandEyeCalibrationInput& operator=(HandEyeCalibrationInput other);

	HandEyeCalibrationInput(const HandEyeCalibrationInput& other);

	~HandEyeCalibrationInput();
	/**
	* @~chinese
	* @brief      ͨ�������˵�ǰ��λ�˹������۱궨����
	* @param[in]  pose				: 4x4�����ʾ�ĵ�ǰλ��
	* @param[in]  points			: �������ϵ�±궨��ǵ�����
	* @~english
	* @brief      construction HandEyeCalibrationInput from current hand pose and chessboard points in eye
	* @param[in]  pose				: current hand pose in 4x4 matrix
	* @param[in]  points			: chessboard points in eye
	**/
	HandEyeCalibrationInput(RobotPoseMatrix4f pose, std::vector<Point3f>& points);
};
/*@} */
}
#endif
