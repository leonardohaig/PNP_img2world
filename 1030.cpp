#include<opencv2/opencv.hpp>
#include<vector>
#include <iostream>
#include "opencvLibs.h"


//计算图像坐标系到地面坐标系的转换矩阵,地面坐标系到图像坐标系的转换矩阵
//输入参数:cameraParam[4]----相机高度,俯仰角,偏航角,翻滚角;单位:度°
//输出参数:无(转换矩阵是类成员变量)
//返回值:无
//Note:函数中求平移矩阵时,目前只考虑了相机高度,之后可以考虑加上相机的其他偏移信息,
//     使得输出数据可以直接运用,即地面坐标系的位置不在左相机之下,而是其他情况
void CalImg2GroundMatrix(float cameraParam[4], 
	cv::Mat& m_ground2cameraMatrix, cv::Mat& m_camera2groundMatrix,
	cv::Mat& RMatrix,cv::Mat& TMatrix)
{
	//计算旋转矩阵,地面(世界)坐标系到相机坐标系
	// rotation matrix for pitch
	float pitch = cameraParam[1] * CV_PI * 1.0 / 180.0;//转换为弧度
	cv::Mat t_pitch_matrix(3, 3, CV_32FC1);
	t_pitch_matrix.at<float>(0, 0) = 1;
	t_pitch_matrix.at<float>(0, 1) = 0;
	t_pitch_matrix.at<float>(0, 2) = 0;

	t_pitch_matrix.at<float>(1, 0) = 0;
	t_pitch_matrix.at<float>(1, 1) = cos(pitch);
	t_pitch_matrix.at<float>(1, 2) = -sin(pitch);

	t_pitch_matrix.at<float>(2, 0) = 0;
	t_pitch_matrix.at<float>(2, 1) = sin(pitch);
	t_pitch_matrix.at<float>(2, 2) = cos(pitch);

	// rotation matrix for yaw
	float yaw = cameraParam[2] * CV_PI * 1.0 / 180.0;
	cv::Mat t_yaw_matrix(3, 3, CV_32FC1);

	t_yaw_matrix.at<float>(0, 0) = cos(yaw);
	t_yaw_matrix.at<float>(0, 1) = 0;
	t_yaw_matrix.at<float>(0, 2) = sin(yaw);

	t_yaw_matrix.at<float>(1, 0) = 0;
	t_yaw_matrix.at<float>(1, 1) = 1;
	t_yaw_matrix.at<float>(1, 2) = 0;

	t_yaw_matrix.at<float>(2, 0) = -sin(yaw);
	t_yaw_matrix.at<float>(2, 1) = 0;
	t_yaw_matrix.at<float>(2, 2) = cos(yaw);

	// rotation matrix for roll
	float roll = cameraParam[3] * CV_PI * 1.0 / 180.0;
	cv::Mat t_roll_matrix(3, 3, CV_32FC1);

	t_roll_matrix.at<float>(0, 0) = cos(roll);
	t_roll_matrix.at<float>(0, 1) = -sin(roll);
	t_roll_matrix.at<float>(0, 2) = 0;

	t_roll_matrix.at<float>(1, 0) = sin(roll);
	t_roll_matrix.at<float>(1, 1) = cos(roll);
	t_roll_matrix.at<float>(1, 2) = 0;

	t_roll_matrix.at<float>(2, 0) = 0;
	t_roll_matrix.at<float>(2, 1) = 0;
	t_roll_matrix.at<float>(2, 2) = 1;

	//cv::Mat RMatrix(3, 3, CV_32FC1);//旋转矩阵
	RMatrix.create(3, 3, CV_32FC1);
	//RMatrix = t_pitch_matrix * t_yaw_matrix * t_roll_matrix;
	RMatrix = t_roll_matrix * t_yaw_matrix * t_pitch_matrix;

	//cv::Mat TMatrix(3, 1, CV_32FC1);//平移矩阵
	TMatrix.create(3, 1, CV_32FC1);
	TMatrix.at<float>(0, 0) = 0;
	TMatrix.at<float>(1, 0) = 0 /*cameraParam[0]*/;//单位决定了后面坐标转换的单位
	TMatrix.at<float>(2, 0) = 0;

	//地面坐标系到相机空间坐标系转换矩阵[R,T
	//                            0,1] 4x4
	//cv::Mat ground2CameraMatrix;

	m_ground2cameraMatrix = cv::Mat::zeros(4, 4, CV_32FC1);
	RMatrix.copyTo(m_ground2cameraMatrix.rowRange(0, 3).colRange(0, 3));
	TMatrix.copyTo(m_ground2cameraMatrix.rowRange(0, 3).colRange(3, 4));
	m_ground2cameraMatrix.at<float>(3, 3) = 1;

	m_camera2groundMatrix = m_ground2cameraMatrix.inv();

}



int main()
{
		cv::FileStorage fs;
		fs.open("../cameraParam.xml", cv::FileStorage::READ);
		if (!fs.isOpened()) // failed
		{
			std::cerr << "Open File Failed !" << std::endl;
			return false;
		}
	
		//初始化相机参数Opencv
		cv::Mat camera_matrix;
		fs["CameraMatrix"] >> camera_matrix;

		//畸变参数
		cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_32FC1);
		fs["CameraDistCoeff"] >> distortion_coefficients;

		float cameraHeight = 0.0f;

		float cameraPitch = 0.0f;
		float cameraYaw = 0.0f;
		float cameraRoll = 0.0f;

		float tX_W = 0.0f;
		float tY_W = 0.0f;
		float tZ_W = 0.0f;
		
		cv::Mat RMatrix, TMatrix;

		fs["CameraHeight"] >> cameraHeight;

		fs["CameraPitch"] >> cameraPitch;
		fs["CameraYaw"] >> cameraYaw;
		fs["CameraRoll"] >> cameraRoll;

		fs["translationWX"] >> tX_W;
		fs["translationWY"] >> tY_W;
		fs["translationWZ"] >> tZ_W;
		
		fs["RMatrix"] >> RMatrix;
		fs["TMatrix"] >> TMatrix;

		fs.release();

		cv::Mat m_ground2cameraMatrix,m_camera2groundMatrix;
		float param[4] = { cameraHeight, cameraPitch, cameraYaw, cameraRoll };

		//CalImg2GroundMatrix(param, m_ground2cameraMatrix, m_camera2groundMatrix, RMatrix, TMatrix);

		RMatrix.convertTo(RMatrix, CV_32FC1);
		TMatrix.convertTo(TMatrix, CV_32FC1);

		m_ground2cameraMatrix = cv::Mat::zeros(4, 4, CV_32FC1);
		RMatrix.copyTo(m_ground2cameraMatrix.rowRange(0, 3).colRange(0, 3));
		TMatrix.copyTo(m_ground2cameraMatrix.rowRange(0, 3).colRange(3, 4));
		m_ground2cameraMatrix.at<float>(3, 3) = 1;

		m_camera2groundMatrix = m_ground2cameraMatrix.inv();

		//===============开始计算====================//

		//内参的逆矩阵
		cv::Mat inerMatInv = camera_matrix.inv();

		
		
		cv::Mat inPointM(3, 1, CV_32FC1);
		inPointM.at<float>(0, 0) = 985;
		inPointM.at<float>(1, 0) = 640;
		inPointM.at<float>(2, 0) = 1;

		//[Xc/Zc Yc/Zc 1]^T = M^-1 * [u v 1]^T
		cv::Mat XYZMat = inerMatInv * inPointM;

		float Zc = 0.0f;

		
		//提取分量。。相机到世界。。得到Zw
		float r11 = m_camera2groundMatrix.ptr<float>(0)[0];
		float r12 = m_camera2groundMatrix.ptr<float>(0)[1];
		float r13 = m_camera2groundMatrix.ptr<float>(0)[2];
		float t1 =  m_camera2groundMatrix.ptr<float>(0)[3];

		float r21 = m_camera2groundMatrix.ptr<float>(1)[0];
		float r22 = m_camera2groundMatrix.ptr<float>(1)[1];
		float r23 = m_camera2groundMatrix.ptr<float>(1)[2];
		float t2 =  m_camera2groundMatrix.ptr<float>(1)[3];

		float r31 = m_camera2groundMatrix.ptr<float>(2)[0];
		float r32 = m_camera2groundMatrix.ptr<float>(2)[1];
		float r33 = m_camera2groundMatrix.ptr<float>(2)[2];
		float t3 =  m_camera2groundMatrix.ptr<float>(2)[3];

		

		float a = XYZMat.at<float>(0, 0);
		float b = XYZMat.at<float>(1, 0);
		Zc = (-1.0*cameraHeight - t3) / (a*r31 + b*r32 + r33);

		float _a, _b;
		_a = (inPointM.at<float>(0, 0) - camera_matrix.at<float>(0, 2)) / camera_matrix.at<float>(0, 0);
		_b = (inPointM.at<float>(1, 0) - camera_matrix.at<float>(1, 2)) / camera_matrix.at<float>(1, 1);


		float Xc = a*Zc;
		float Yc = b*Zc;

		cv::Mat pointInCameraM(4, 1, CV_32FC1);
		pointInCameraM.at<float>(0, 0) = Xc;
		pointInCameraM.at<float>(1, 0) = Yc;
		pointInCameraM.at<float>(2, 0) = Zc;
		pointInCameraM.at<float>(3, 0) = 1;

		cv::Mat groundM;
		
		groundM = m_camera2groundMatrix * pointInCameraM;

		float _valueX = r11* Xc + r12*Yc + r13*Zc + t1;
		float _valueY = r21* Xc + r22*Yc + r23*Zc + t2;
		float _valueZ = r31* Xc + r32*Yc + r33*Zc + t3;
		
		std::cout << groundM << std::endl;
		
		system("pause");
	return 0;
}