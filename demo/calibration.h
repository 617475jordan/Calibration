#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "calibration.h"
#include <opencv_all.h>
class calibration
{
public:
	void run();
	void addChessboardPoints();
	void currentCameraCorner(Mat input_image);//实时显示
	void calibrate();
	Mat calibrationRemap(Mat image, int flag, int flag_0);
	void countError();
	void resultPrint();
	void saveData();
	void printfVectorMat(vector<Mat> umatrix);
	void printfMatrix(Mat umatrix);
	void readData();//读取保存的相机，以及校正数据
private:
	const int n_images = 10;//图片总数
	const int width = 320 * 2;
	const int height = 240 * 2;
	const Size photoSize = Size(width,height);
	const double    square_width = 17.2;//格子边长
	const int chessboardWidth = 9;//设置图像中X方向上的角点个数
	const int chessboardHeigth = 6;//设置图像中Y方向上的角点个数
	const Size chessboardSize = Size(chessboardWidth, chessboardHeigth);
	const int n_points = chessboardWidth*chessboardHeigth;//单张图片中角点总数


	vector<Point2f> currentCorners;//实时角点数
	Mat calibrationRemapImage;
	vector<vector<Point2f>> image_points;//检测到角点在图像中的二重二维坐标
	vector<vector<Point3f>> object_points;//检测到角点在实际中的二重三维坐标
	int imageCount0 = 0;//当前找到图片总数
	// 输出矩阵
	Mat cameraMatrix;
	Mat distCoeffs;
	vector<Mat> rvecs, tvecs;
	Mat map1, map2;// 用于图像去畸变
	bool mustInitUndistort; // 是否重新进行去畸变
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 40, 0.001);

	// 棋盘上的点的两种坐标
	vector<Point2f> imageCorners;
	vector<Point3f> objectCorners;

	double totalErrorValue = 0;
	double errorValue[8];
	double totalAveError;

	FILE *outputFile;
#define Window_Name_0 "摄像头帧截取窗口"
};


#endif