#include "calibration.h"


void calibration::run()
{
	addChessboardPoints();// ������ͼ����ȡ�ǵ�
	calibrate();// ����궨
	countError();
	// ͼ��ȥ������ʾ
	for (int i = 1; i <= n_images; i++)
	{
		char filename[20];
		sprintf_s(filename, "..\\data\\%d.jpg", i);
		Mat image = imread(filename);
		calibrationRemap(image, i,0);

		image.release();
	}
	resultPrint();//�����ӡ���
	saveData();

}
// ������ͼ����ȡ�ǵ�
void calibration::addChessboardPoints()
{
	for (int i = 0; i < chessboardHeigth; i++)
	{
		for (int j = 0; j < chessboardWidth; j++)
		{
			objectCorners.push_back(Point3f(i, j, 0.0f));
		}
	}
	// ѭ������ͼƬ
	int current_image_count = 1;//���㵱ǰ��ȡͼƬ��Ŀ����ʼ��
	for (current_image_count; current_image_count <= n_images; current_image_count++)
	{
		Mat input_image; // ������������ͼ��
		Mat input_gray_image;
		char filename[20];
		bool find_corners_result;// �õ����̽ǵ�
		int success = 0;//��Ǹ���ͼƬ�Ƿ��ʺϱ궨
		sprintf(filename, "..\\data\\%d.jpg", current_image_count);
		input_image = imread(filename, 1);
		if (input_image.cols != width || input_image.rows != height)
		{
			printf("..\\data\\%d.jpgͼ��ֱ��ʲ�����\n",current_image_count);
			continue;
		}
		else
		{

			cvtColor(input_image, input_gray_image, CV_BGR2GRAY);
			find_corners_result = findChessboardCorners(input_image,
				chessboardSize,
				imageCorners,
				3);

			// ����ǵ���Ŀ����ҪҪ����ô������������
			if (imageCorners.size() == chessboardSize.area())
			{
				// ��ȡ�����ؾ���,���õ�������߾���
				cornerSubPix(input_gray_image,
					imageCorners,
					chessboardSize, // �������ڵ�һ���С
					Size(-1, -1), // ������һ���С��(-1, -1)��ʾû������
					criteria);
				// ���һ���ӽ��е�ͼ��㼰������
				image_points.push_back(imageCorners);//����ʱ�ǵ����������ͼ��ǵ�Ķ��ض�ά������
				object_points.push_back(objectCorners);//����ʱ�ǵ����������ʵ�ʽǵ�Ķ�����ά������
				imageCount0++;
				success = 1;
			}
			// ���ƽǵ�,find_corners_result �Ѿ��ҵ��ǵ�
			drawChessboardCorners(input_image,
				chessboardSize,
				imageCorners,
				find_corners_result);

			// ��ʾ
			if (success == 1)
			{
				printf("��ǰ��ʾͼƬΪ%d.jpg,�ɽ��б궨\n", current_image_count);
			}
			if (success == 0)
			{
				printf("��ǰ��ʾͼƬΪ%d.jpg,���ɽ��б궨\n", current_image_count);
			}
			namedWindow(filename, WINDOW_AUTOSIZE);
			imshow(filename, input_image);
			waitKey(1000);
			input_image.release();
			destroyWindow(filename);
		}
	}
}

void calibration::currentCameraCorner(Mat input_image)//ʵʱ��ʾ
{
	bool find_corners_result;
	Mat input_gray_image;
	cvtColor(input_image, input_gray_image, CV_BGR2GRAY);
	find_corners_result = findChessboardCorners(
		input_image,
		chessboardSize,
		currentCorners,
		CV_CALIB_CB_ADAPTIVE_THRESH);
	// ����ǵ���Ŀ����ҪҪ����ô������������
	if (imageCorners.size() == chessboardSize.area())
	{
		// ��ȡ�����ؾ���,���õ�������߾���
		cornerSubPix(input_gray_image,
			imageCorners,
			chessboardSize, // �������ڵ�һ���С
			Size(-1, -1), // ������һ���С��(-1, -1)��ʾû������
			criteria);
	}
	namedWindow(Window_Name_0, WINDOW_AUTOSIZE);
	imshow(Window_Name_0, input_image);
}

// ���б궨��������ͶӰ���
// ����������ڲξ���(camera_matrix)
// �����˻���ϵ��(dist_coeffs)
// ��������ת����(rvecs)
// ������ƽ������(tvecs)
void calibration::calibrate()
{

	if (imageCount0 == 0)
	{
		printf("δ���ҵ�Cornersֵ������ʧ��\n");
		cvWaitKey(2000);
		exit(-1);
	}
	else
	{
		// �������½���ȥ����
		mustInitUndistort = true;
		printf("һ��ʹ��%d��ͼƬ���б궨\n", imageCount0);
		printf("��ʼ��������궨\n");
		// ��ʼ�궨
		calibrateCamera(object_points, // 3D��
			image_points,  // ͼ���
			photoSize,    // ͼ��ߴ�
			cameraMatrix, // ������������
			distCoeffs,   // ����ϵ��
			rvecs, tvecs); // ��ת��ƽ��
	//	Save_Data();//����ϵ������
		printf("����궨����\n");
	}
}
//�������
void calibration::countError()
{
	int flag = 0;
	
	for (int i = 0; i < imageCount0; i++)
	{
		if (imageCorners.size() == chessboardSize.area())
		{
			vector<Point2f> image_points2;
			//������ת����,ƽ���������ڲξ�����������������ʵ�ʽǵ���ͼ���е�����
			projectPoints(object_points[i],//�ǵ�ʵ������
				rvecs[i],//��ת����
				tvecs[i],//ƽ������
				cameraMatrix,//�ڲξ���
				distCoeffs,//����������
				image_points2);//�洢ͼ��ʵ������
			Mat tempImagePointMat = Mat(1, image_points[i].size(), CV_32FC2);//�������ڴ���ǵ��ͼ������ľ���
			Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);//�������ڴ���ǵ��ͼ����������ֵ�ľ���  

			for (int j = 0; j < (int)image_points[i].size(); j++)
			{
				image_points2Mat.at<Vec2f>(0, j) = Vec2d(image_points2[j].x, image_points2[j].y);
				tempImagePointMat.at<Vec2f>(0, j) = Vec2d(image_points[i][j].x, image_points[i][j].y);
			}
			errorValue[i] = norm(image_points2Mat, tempImagePointMat, NORM_L2);//����ֵ�������Թ�һ��;
			totalErrorValue += errorValue[i] /= image_points[i].size();
			cout << "��" << flag + 1 << "��ͼ���ƽ�����:" << errorValue[i] << "����" << endl;
			flag++;
		}
	}
	totalAveError = totalAveError / flag;
	cout << "����ƽ�����:" << totalAveError << "����" << endl;
	printf("�����Ϊ:%f\n", totalErrorValue);
	printf("%f\n", totalAveError);
}
// �궨��ȥ��ͼ���еĻ���
Mat calibration::calibrationRemap(Mat image, int flag, int flag_0)
{
	Mat undistorted;
	double undis_ave;
	if (flag_0 == 0)
		undis_ave = 1 - totalAveError;
	if (flag_0 == 1)
	{
		readData();
		undis_ave = 1 - totalAveError;
		mustInitUndistort = true;
	}
	double undis_width = undis_ave*width;
	double undis_height = undis_ave*height;
	int undis_int_width = (int)undis_width;
	int undis_int_height = (int)undis_height;
	Size undis = Size(undis_int_width, undis_int_height);//У����ͼ��ֱ���
	if (mustInitUndistort)   // ÿ�α궨ֻ��Ҫ��ʼ��һ��
	{
		initUndistortRectifyMap(
			cameraMatrix,  // CalibrateCamera�м���õ����������
			distCoeffs,    // CalibrateCamera�м���õ��Ļ������
			Mat(),     // ��ѡ��У������(�˴�Ϊ��)
			Mat(),     // ��������undistorted������������
			undis, // image.size(), undistorted����ĳߴ�
			CV_32FC1,      // ���ӳ��ͼ�������
			map1, map2);   // x�����y����ӳ�亯��
		mustInitUndistort = false;
	}
	double border_larger_width,border_lager_height;
	border_larger_width = image.cols / undis_ave - image.cols;
	border_lager_height = image.rows / undis_ave - image.rows;
	Mat temp;
	copyMakeBorder(image, temp,
		0,
		border_larger_width/2,
		border_lager_height/2,
		border_lager_height/2,
		BORDER_CONSTANT, 0);
	//imshow(" ", temp);	
	// Ӧ��ӳ�亯��
	undistort( temp,undistorted, cameraMatrix, distCoeffs);
	/*remap( undistorted,temp, map1, map2,
		INTER_LINEAR);*/
	
    resize( undistorted, image,photoSize,INTER_LINEAR);//��ͼƬ�ָ���ԭʼ��С
	//imshow(" ", undistorted);
	//��ʾУ����ͼƬ
	char filename[20];
	if (flag_0 == 0)
	{
		sprintf(filename, "..\\out\\%d.bmp", flag);
		imwrite(filename, image);
		printf("��ǰУ��ͼƬΪ%d.bmp\n", flag);
		//Show_Current_Time(flag);
		imshow(filename, image);
		cvWaitKey(1000);
		undistorted.release();
	//	image.release();
		destroyWindow(filename);
		return image;
	}
	//ʵ�����ͼƬ
    if (flag_0 == 1)
	{
		namedWindow(Window_Name_0, WINDOW_AUTOSIZE);
	/*	vector<Mat> channels;
		split(undistorted, channels);
		imshow(" ", undistorted);
		for (int i = 0; i < channels.size(); i++)
		{
			equalizeHist(channels.at(i), channels.at(i));
		}
		merge(channels, undistorted);*/
		return image;
	//	Brightness_And_Contrast_Init(undistorted,fps); �ԱȶȺ����ȵ���
	}
}

//��Ҫ����������
void calibration::resultPrint()
{
	printf("����ڲξ���\n");
	printfMatrix(cameraMatrix);                                         //����̨��ʾ����ڲξ���  

	printf("����������\n");
	printfMatrix(distCoeffs);                                        //����̨��ʾ�������  

	printf("��ת�����飺\n");
	printfVectorMat(rvecs);                                                 //����̨��ʾ��ת������  

	printf("ƽ�������飺\n");
	printfVectorMat(tvecs);                                                //����̨��ʾƽ�������� 
	
}
//�ڲκͻ���������
void calibration::printfMatrix(Mat umatrix)
{
	for (int i = 0; i < umatrix.rows; ++i)
	{
		for (int j = 0; j < umatrix.cols; ++j)
		{
				printf("%lf ", umatrix.at<double>(i, j));
		}
		printf("\n");
	}
	printf("\n");
}
//��ת��ƽ�Ƽ������
void calibration::printfVectorMat(vector<Mat> umatrix)
{
	for (int i = 0; i < (int)umatrix.size(); ++i)
	{
		for (int j = 0; j < umatrix[i].rows; ++j)
		{
			for (int k = 0; k < umatrix[i].cols; ++k)
			{
					printf("  %lf", umatrix[i].at<double>(j, k));
			}
		}
		printf("\n");
	}
}

void calibration::readData()
{
	FileStorage fs_0;
	fs_0.open("Camera_Calibration_Data.xml", FileStorage::READ);
	if (!fs_0.isOpened())
	{
		printf("��������������궨\n");
		printf("�˳�\n");
		exit(-1);
	}
	else
	{
		fs_0["camera_matrix"] >> cameraMatrix;
		fs_0["dist_coeffs"] >> distCoeffs;
		fs_0["total_ave_error"] >> totalAveError;
		//fs_0["map1"] >> map1;
		//fs_0["map2"] > map2;
		fs_0.release();
	}
}

//���������xml�ļ���
void calibration::saveData()
{
	FileStorage fs_0;
	fs_0.open("Camera_Calibration_Data.xml", FileStorage::WRITE);
	fs_0 << "camera_matrix" << cameraMatrix;//��������ھ���
	fs_0 << "dist_coeffs" << distCoeffs;//�������ϵ��
	fs_0 << "total_ave_error" << totalAveError;//�����������
	//fs_0 << "map1" << map1;
	//fs_0 << "map2" << map2;
	fs_0.release();

	FileStorage fs_1("Camera_Calibration_Data_0.xml", FileStorage::WRITE);
	//д����תʸ����
	fs_1 << "rvecs" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		stringstream cvter;
		string name_str = "The_RvecsValue_of_", name_num;//����ַ���
		cvter << i;
		cvter >> name_num;
		name_str.append(name_num);
		name_str.append("The_Photo");
		fs_1 << "{:" << name_str << rvecs[i] << "}";
	}
	fs_1 << "]";
	//д��ƽ��ʸ����
	fs_1 << "tvecs" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		stringstream cvter;
		string name_str = "The_TvecsValue_of_", name_num;//����ַ���
		cvter << i;
		cvter >> name_num;
		name_str.append(name_num);
		name_str.append("The_Photo");
		fs_1 << "{:" << name_str << tvecs[i] << "}";
	}
	fs_1 << "]";
	//д��ÿ��ͼ���
	fs_1 << "Error_of_Per_Image" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		stringstream cvter;
		string name_str = "The_ErrorValue_of_", name_num;//����ַ���
		cvter << i;
		cvter >> name_num;
		name_str.append(name_num);
		name_str.append("The_Photo");
		fs_1 << "{:" << name_str << errorValue[i] << "}";
	}
	fs_1 << "]";
	fs_1.release();
	//����
	FileStorage fs_2("Camera_Calibration_Data_1.xml", FileStorage::WRITE);
	//д��ǵ�ͼ������
	fs_2 << "Image_Corners_Coordinates" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		fs_2 << "{:" << "Photo_num" << i << "coordinates" << "[";
		for (int j = 0; j < (int)imageCorners.size(); j++)
		{
			fs_2 << image_points[i][j];
		}
		fs_2 << "]" << "}";
	}
	fs_2 << "]";
	fs_2 << "Object_Corners_Coordinates" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		fs_2 << "{:" << "Photo_num" << i << "coordinates" << "[";
		for (int j = 0; j < (int)imageCorners.size(); j++)
		{
			fs_2 << object_points[i][j];
		}
		fs_2 << "]" << "}";
	}
	fs_2 << "]";
	fs_2.release();
}
