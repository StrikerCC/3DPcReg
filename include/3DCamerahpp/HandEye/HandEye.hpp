/*****************************************************************************
*  3DCamera SDK header
*
*
*  @file     HandEye.3DCamerahpp
*  @brief    3DCamera sdk header
*
*  @version  1.0
*  @date     2020 / 05 / 09
*
*****************************************************************************/
#ifndef __HANDEYE_HPP__
#define __HANDEYE_HPP__

#include <vector>
#include "hpp/HandEye/HandEyeInput.hpp"
#include "hpp/Camera.hpp"
#include "hpp/Frame.hpp"
#include "hpp/APIExport.hpp"

namespace cs
{
/**
* @~chinese
* \defgroup HandEye ���۱궨
* @brief �ṩ���۱궨���ݲɼ����������㣬��ʹ�ñ궨�����������ϵ�µĵ��������ת�����ֵ�����ϵ��
* @{
* @~english
* \defgroup HandEye HandEye Calibration
* @brief Provide functions for eye-hand calibration
* @{
*/

/**
* @~chinese
* @brief ö��: ���߷����������������
* @~english
* @brief enumeration: The baseline direction
**/
typedef enum
{
	BASELINE_HORIZONTAL  = 0x00,	/**< @~chinese ˮƽ����		@~english baseline is horizontal*/ 
	BASELINE_VERTICAL    = 0x01,	/**< @~chinese ��ֱ����		@~english baseline is vertical*/ 
}BASELINE_TYPE;

/**
* @~chinese
* @brief         �ؽ��궨������
* @param[in]     camera                      : �����ӳɹ������ָ��
* @param[in]     pairFrame                   : ��ʽ��STREAM_FORMAT_PAIR��֡����
* @param[out]    points                      : �ؽ��ı궨��ǵ����
* @param[in]     chessboardHorizontalPoints  : ����궨��ǵ���
* @param[in]     chessboardVerticalPoints    : ����궨��ǵ���
* @param[in]     type                        : ���߷���, @�μ� BASELINE_TYPE
* @return     �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         reconstruct chessboard points
* @param[in]     camera                      : camera pointer, @see ICameraPtr
* @param[in]     pairFrame                   : the frame which format is STREAM_FORMAT_PAIR
* @param[out]    points                      : output the reconstructed chessboard points
* @param[in]     chessboardHorizontalPoints	 : number of horizontal chessboard points
* @param[in]     chessboardVerticalPoints    : number of vertical chessboard points
* @param[in]     type                        : baseline type, @see BASELINE_TYPE
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE reconstructChessboardPoints(ICameraPtr camera, IFramePtr pairFrame, std::vector<Point3f> &points, int chessboardHorizontalPoints = 9, int chessboardVerticalPoints = 6, BASELINE_TYPE type = BASELINE_HORIZONTAL);

/**
* @~chinese
* @brief         �ؽ��궨������
* @param[in]     camera                      : �����ӳɹ������ָ��
* @param[in]     imageLeft                   : �����ͼ��
* @param[in]     imageRight                  : �����ͼ��
* @param[in]     width	                     : ͼ����
* @param[in]     height						 : ͼ��߶�
* @param[out]    points                      : �ؽ��ı궨��ǵ����
* @param[in]     chessboardHorizontalPoints  : ����궨��ǵ���
* @param[in]     chessboardVerticalPoints    : ����궨��ǵ���
* @param[in]     type                        : ���߷���, @�μ� BASELINE_TYPE
* @return     �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         reconstruct chessboard points
* @param[in]     camera                      : camera pointer, @see ICameraPtr
* @param[in]     imageLeft                   : left image
* @param[in]     imageRight                  : right image
* @param[in]     width	                     : the width of image
* @param[in]     height						 : the height of image
* @param[out]    points                      : output the reconstructed chessboard points
* @param[in]     chessboardHorizontalPoints	 : number of horizontal chessboard points
* @param[in]     chessboardVerticalPoints    : number of vertical chessboard points
* @param[in]     type                        : baseline type, @see BASELINE_TYPE
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE reconstructChessboardPoints(ICameraPtr camera, const unsigned char* imageLeft, const unsigned char* imageRight, int width, int height, std::vector<cs::Point3f> &points, int chessboardHorizontalPoints = 9, int chessboardVerticalPoints = 6, BASELINE_TYPE type = BASELINE_HORIZONTAL);

/**
* @~chinese
* @brief         ͨ���������ͼƬ����ȡ��������������㵽��ά������
* @param[in]     camera                      : �����ӳɹ������ָ��
* @param[in]     leftPoints                  : �����ͼƬ����ȡ������������
* @param[in]     rightPoints                 : �����ͼƬ����ȡ������������
* @param[out]    points                      : �ؽ��������ά������
* @param[in]     type                        : ��������
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         reconstruct feature points
* @param[in]     camera                      : camera pointer, @see ICameraPtr
* @param[in]     leftPoints                  : left picture feature points
* @param[in]     rightPoints                 : right picture feature points
* @param[out]    points                      : reconstructed points
* @param[in]     type                        : baseline type, @see BASELINE_TYPE
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE reconstructPoints(ICameraPtr camera, std::vector<Point2f> leftPoints, std::vector<Point2f> rightPoints, std::vector<Point3f> &points, BASELINE_TYPE type = BASELINE_HORIZONTAL);

/**
* @~chinese
* @brief         ����ڻ������ⲿʱ������תƽ�ƾ���
* @param[in]     input           : ����ڻ������ⲿʱ�ı궨����
* @param[out]    matrix          : ������۵��־���
* @param[out]    error           : ������
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         calculate eye to hand matrix
* @param[in]     input           : eye to hand calibration input
* @param[out]    matrix          : eye to hand matrix
* @param[out]    error           : convergence error
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE calibrateEyeToHand(std::vector<HandEyeCalibrationInput>& input, HandEyeMatrix& matrix, double* error = NULL);

/**
* @~chinese
* @brief         ����̶��ڻ�������ʱ������תƽ�ƾ���
* @param[in]     input           : ����̶��ڻ�������ʱ�ı궨����
* @param[out]    matrix          : ������۵��־���
* @param[out]    error           : ������
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         calculate eye in hand matrix
* @param[in]     input           : eye in hand calibration input
* @param[out]    matrix          : eye to hand matrix
* @param[out]    error           : convergence error
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE calibrateEyeInHand(std::vector<HandEyeCalibrationInput>& input, HandEyeMatrix& matrix, double* error = NULL);

/**
* @~chinese
* @brief         ���������ϵ�µĵ�ת������������ϵ
* @param[in]     pose           : �����˵�ǰλ��
* @param[in]     inputPoint     : �������ϵ�µĵ�
* @param[in]     matrix         : �۵��־���calibrateEyeToHand/calibrateEyeInHand����õ�
* @param[out]    outPoint       : ���������������ϵ�µĵ�
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         transform point to the coordinate system of hand
* @param[in]     pose           : the pose of robot
* @param[in]     inputPoint     : the input point in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[out]    outPoint       : output the point in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformPointFromEyeToHand(RobotPoseMatrix4f pose, Point3f inputPoint, HandEyeMatrix matrix, Point3f& outPoint);

/**
* @~chinese
* @brief         ���������ϵ�µ�����ת������������ϵ
* @param[in]     pose           : �����˵�ǰλ��
* @param[in]     inputVector    : �������ϵ�µ�����
* @param[in]     matrix         : �۵��־���calibrateEyeToHand/calibrateEyeInHand����õ�
* @param[out]    outVector      : ���������������ϵ�µ�����
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         transform vector to the coordinate system of hand
* @param[in]     pose           : the pose of robot
* @param[in]     inputVector    : the input vector in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[out]    outVector      : output the vector in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformVectorFromEyeToHand(RobotPoseMatrix4f pose, Point3f inputVector, HandEyeMatrix matrix, Point3f& outVector);

/**
* @~chinese
* @brief         ���������ϵ�µ�λ��ת������������ϵ
* @param[in]     toolPose       : �����˵�ǰλ��
* @param[in]     poseInEye      : �������ϵ�µ�λ��
* @param[in]     matrix         : �۵��־���calibrateEyeToHand/calibrateEyeInHand����õ�
* @param[in]	 poseType		�����λ�˱�ʾ����
* @param[in]	 poseUnit		: �����̬��ʾ��λ�Ƕ�/����
* @param[out]    outPoseInHand  : ���λ��
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         transform point to the coordinate system of hand
* @param[in]     toolPose       : the pose of robot
* @param[in]     poseInEye      : the input point in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[in]	 poseType		��type of output pose
* @param[in]	 poseUnit		: unit of output direction(radian/degree)
* @param[out]    outPoseInHand  : the output pose in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformPoseFromEyeToHand(RobotPoseMatrix4f toolPose, RobotPoseMatrix4f poseInEye, HandEyeMatrix matrix,
											  RobotPoseType poseType, RobotPoseUnit poseUnit, RobotPose& outPoseInHand);

/**
* @~chinese
* @brief         ���������ϵ�µ�λ��ת������������ϵ
* @param[in]     toolPose       : �����˵�ǰλ��
* @param[in]     poseInEye      : �������ϵ�µ�λ��
* @param[in]     matrix         : �۵��־���calibrateEyeToHand/calibrateEyeInHand����õ�
* @param[out]    outMatrixInHand  : ���λ��
* @return �ɹ�:SUCCESS, ʧ��:����������
* @~english
* @brief         transform point to the coordinate system of hand
* @param[in]     toolPose       : the pose of robot
* @param[in]     poseInEye      : the input point in eye
* @param[in]     matrix         : the matrix eye to hand, returned by calibrateEyeToHand/calibrateEyeInHand
* @param[out]    outMatrixInHand  : the output matrix in hand
* @return success:return SUCCESS, fail:other error code
**/
CS_API ERROR_CODE transformPoseFromEyeToHand(RobotPoseMatrix4f toolPose, RobotPoseMatrix4f poseInEye, HandEyeMatrix matrix,
	RobotPoseMatrix4f outMatrixInHand);

/*@} */
}
#endif
