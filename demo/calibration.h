#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "calibration.h"
#include <opencv_all.h>
class calibration
{
public:
	void run();
	void addChessboardPoints();
	void currentCameraCorner(Mat input_image);//ʵʱ��ʾ
	void calibrate();
	Mat calibrationRemap(Mat image, int flag, int flag_0);
	void countError();
	void resultPrint();
	void saveData();
	void printfVectorMat(vector<Mat> umatrix);
	void printfMatrix(Mat umatrix);
	void readData();//��ȡ�����������Լ�У������
private:
	const int n_images = 10;//ͼƬ����
	const int width = 320 * 2;
	const int height = 240 * 2;
	const Size photoSize = Size(width,height);
	const double    square_width = 17.2;//���ӱ߳�
	const int chessboardWidth = 9;//����ͼ����X�����ϵĽǵ����
	const int chessboardHeigth = 6;//����ͼ����Y�����ϵĽǵ����
	const Size chessboardSize = Size(chessboardWidth, chessboardHeigth);
	const int n_points = chessboardWidth*chessboardHeigth;//����ͼƬ�нǵ�����


	vector<Point2f> currentCorners;//ʵʱ�ǵ���
	Mat calibrationRemapImage;
	vector<vector<Point2f>> image_points;//��⵽�ǵ���ͼ���еĶ��ض�ά����
	vector<vector<Point3f>> object_points;//��⵽�ǵ���ʵ���еĶ�����ά����
	int imageCount0 = 0;//��ǰ�ҵ�ͼƬ����
	// �������
	Mat cameraMatrix;
	Mat distCoeffs;
	vector<Mat> rvecs, tvecs;
	Mat map1, map2;// ����ͼ��ȥ����
	bool mustInitUndistort; // �Ƿ����½���ȥ����
	TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 40, 0.001);

	// �����ϵĵ����������
	vector<Point2f> imageCorners;
	vector<Point3f> objectCorners;

	double totalErrorValue = 0;
	double errorValue[8];
	double totalAveError;

	FILE *outputFile;
#define Window_Name_0 "����ͷ֡��ȡ����"
};


#endif