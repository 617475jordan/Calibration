#include "calibration.h"


void calibration::run()
{
	addChessboardPoints();// 打开棋盘图像并提取角点
	calibrate();// 相机标定
	countError();
	// 图像去畸变显示
	for (int i = 1; i <= n_images; i++)
	{
		char filename[20];
		sprintf_s(filename, "..\\data\\%d.jpg", i);
		Mat image = imread(filename);
		calibrationRemap(image, i,0);

		image.release();
	}
	resultPrint();//结果打印输出
	saveData();

}
// 打开棋盘图像并提取角点
void calibration::addChessboardPoints()
{
	for (int i = 0; i < chessboardHeigth; i++)
	{
		for (int j = 0; j < chessboardWidth; j++)
		{
			objectCorners.push_back(Point3f(i, j, 0.0f));
		}
	}
	// 循环所有图片
	int current_image_count = 1;//计算当前获取图片数目，初始化
	for (current_image_count; current_image_count <= n_images; current_image_count++)
	{
		Mat input_image; // 用来保存棋盘图像
		Mat input_gray_image;
		char filename[20];
		bool find_corners_result;// 得到棋盘角点
		int success = 0;//标记该张图片是否适合标定
		sprintf(filename, "..\\data\\%d.jpg", current_image_count);
		input_image = imread(filename, 1);
		if (input_image.cols != width || input_image.rows != height)
		{
			printf("..\\data\\%d.jpg图像分辨率不符合\n",current_image_count);
			continue;
		}
		else
		{

			cvtColor(input_image, input_gray_image, CV_BGR2GRAY);
			find_corners_result = findChessboardCorners(input_image,
				chessboardSize,
				imageCorners,
				3);

			// 如果角点数目满足要要求，那么将它加入数据
			if (imageCorners.size() == chessboardSize.area())
			{
				// 获取亚像素精度,利用迭代法提高精度
				cornerSubPix(input_gray_image,
					imageCorners,
					chessboardSize, // 搜索窗口的一半大小
					Size(-1, -1), // 死区的一半大小，(-1, -1)表示没有死区
					criteria);
				// 添加一个视角中的图像点及场景点
				image_points.push_back(imageCorners);//将临时角点坐标组存入图像角点的二重二维坐标组
				object_points.push_back(objectCorners);//将临时角点坐标组存入实际角点的二重三维坐标组
				imageCount0++;
				success = 1;
			}
			// 绘制角点,find_corners_result 已经找到角点
			drawChessboardCorners(input_image,
				chessboardSize,
				imageCorners,
				find_corners_result);

			// 显示
			if (success == 1)
			{
				printf("当前显示图片为%d.jpg,可进行标定\n", current_image_count);
			}
			if (success == 0)
			{
				printf("当前显示图片为%d.jpg,不可进行标定\n", current_image_count);
			}
			namedWindow(filename, WINDOW_AUTOSIZE);
			imshow(filename, input_image);
			waitKey(1000);
			input_image.release();
			destroyWindow(filename);
		}
	}
}

void calibration::currentCameraCorner(Mat input_image)//实时显示
{
	bool find_corners_result;
	Mat input_gray_image;
	cvtColor(input_image, input_gray_image, CV_BGR2GRAY);
	find_corners_result = findChessboardCorners(
		input_image,
		chessboardSize,
		currentCorners,
		CV_CALIB_CB_ADAPTIVE_THRESH);
	// 如果角点数目满足要要求，那么将它加入数据
	if (imageCorners.size() == chessboardSize.area())
	{
		// 获取亚像素精度,利用迭代法提高精度
		cornerSubPix(input_gray_image,
			imageCorners,
			chessboardSize, // 搜索窗口的一半大小
			Size(-1, -1), // 死区的一半大小，(-1, -1)表示没有死区
			criteria);
	}
	namedWindow(Window_Name_0, WINDOW_AUTOSIZE);
	imshow(Window_Name_0, input_image);
}

// 进行标定，返回重投影误差
// 计算了相机内参矩阵(camera_matrix)
// 计算了畸变系数(dist_coeffs)
// 计算了旋转矩阵(rvecs)
// 计算了平移向量(tvecs)
void calibration::calibrate()
{

	if (imageCount0 == 0)
	{
		printf("未能找到Corners值，本次失败\n");
		cvWaitKey(2000);
		exit(-1);
	}
	else
	{
		// 必须重新进行去畸变
		mustInitUndistort = true;
		printf("一共使用%d张图片进行标定\n", imageCount0);
		printf("开始进行相机标定\n");
		// 开始标定
		calibrateCamera(object_points, // 3D点
			image_points,  // 图像点
			photoSize,    // 图像尺寸
			cameraMatrix, // 输出的相机矩阵
			distCoeffs,   // 畸变系数
			rvecs, tvecs); // 旋转和平移
	//	Save_Data();//畸变系数保存
		printf("相机标定结束\n");
	}
}
//计算误差
void calibration::countError()
{
	int flag = 0;
	
	for (int i = 0; i < imageCount0; i++)
	{
		if (imageCorners.size() == chessboardSize.area())
		{
			vector<Point2f> image_points2;
			//根据旋转向量,平移向量，内参矩阵和相机畸变矩阵计算实际角点在图像中的坐标
			projectPoints(object_points[i],//角点实际坐标
				rvecs[i],//旋转向量
				tvecs[i],//平移向量
				cameraMatrix,//内参矩阵
				distCoeffs,//相机畸变矩阵
				image_points2);//存储图像实际坐标
			Mat tempImagePointMat = Mat(1, image_points[i].size(), CV_32FC2);//创建用于储存角点的图像坐标的矩阵
			Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);//创建用于储存角点的图像坐标理论值的矩阵  

			for (int j = 0; j < (int)image_points[i].size(); j++)
			{
				image_points2Mat.at<Vec2f>(0, j) = Vec2d(image_points2[j].x, image_points2[j].y);
				tempImagePointMat.at<Vec2f>(0, j) = Vec2d(image_points[i][j].x, image_points[i][j].y);
			}
			errorValue[i] = norm(image_points2Mat, tempImagePointMat, NORM_L2);//理论值进行线性归一化;
			totalErrorValue += errorValue[i] /= image_points[i].size();
			cout << "第" << flag + 1 << "幅图像的平均误差:" << errorValue[i] << "像素" << endl;
			flag++;
		}
	}
	totalAveError = totalAveError / flag;
	cout << "总体平均误差:" << totalAveError << "像素" << endl;
	printf("总误差为:%f\n", totalErrorValue);
	printf("%f\n", totalAveError);
}
// 标定后去除图像中的畸变
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
	Size undis = Size(undis_int_width, undis_int_height);//校正后图像分辨率
	if (mustInitUndistort)   // 每次标定只需要初始化一次
	{
		initUndistortRectifyMap(
			cameraMatrix,  // CalibrateCamera中计算得到的相机矩阵
			distCoeffs,    // CalibrateCamera中计算得到的畸变矩阵
			Mat(),     // 可选的校正矩阵(此处为空)
			Mat(),     // 用于生成undistorted对象的相机矩阵
			undis, // image.size(), undistorted对象的尺寸
			CV_32FC1,      // 输出映射图像的类型
			map1, map2);   // x坐标和y坐标映射函数
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
	// 应用映射函数
	undistort( temp,undistorted, cameraMatrix, distCoeffs);
	/*remap( undistorted,temp, map1, map2,
		INTER_LINEAR);*/
	
    resize( undistorted, image,photoSize,INTER_LINEAR);//将图片恢复成原始大小
	//imshow(" ", undistorted);
	//显示校正后图片
	char filename[20];
	if (flag_0 == 0)
	{
		sprintf(filename, "..\\out\\%d.bmp", flag);
		imwrite(filename, image);
		printf("当前校正图片为%d.bmp\n", flag);
		//Show_Current_Time(flag);
		imshow(filename, image);
		cvWaitKey(1000);
		undistorted.release();
	//	image.release();
		destroyWindow(filename);
		return image;
	}
	//实际输出图片
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
	//	Brightness_And_Contrast_Init(undistorted,fps); 对比度和亮度调整
	}
}

//需要输入结果调用
void calibration::resultPrint()
{
	printf("相机内参矩阵：\n");
	printfMatrix(cameraMatrix);                                         //命令台显示相机内参矩阵  

	printf("相机畸变矩阵：\n");
	printfMatrix(distCoeffs);                                        //命令台显示畸变矩阵  

	printf("旋转向量组：\n");
	printfVectorMat(rvecs);                                                 //命令台显示旋转向量组  

	printf("平移向量组：\n");
	printfVectorMat(tvecs);                                                //命令台显示平移向量组 
	
}
//内参和畸变计算输出
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
//旋转和平移计算输出
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
		printf("请重启进行相机标定\n");
		printf("退出\n");
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

//结果保存在xml文件中
void calibration::saveData()
{
	FileStorage fs_0;
	fs_0.open("Camera_Calibration_Data.xml", FileStorage::WRITE);
	fs_0 << "camera_matrix" << cameraMatrix;//保存相机内矩阵
	fs_0 << "dist_coeffs" << distCoeffs;//保存畸变系数
	fs_0 << "total_ave_error" << totalAveError;//保存整体误差
	//fs_0 << "map1" << map1;
	//fs_0 << "map2" << map2;
	fs_0.release();

	FileStorage fs_1("Camera_Calibration_Data_0.xml", FileStorage::WRITE);
	//写入旋转矢量组
	fs_1 << "rvecs" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		stringstream cvter;
		string name_str = "The_RvecsValue_of_", name_num;//组成字符串
		cvter << i;
		cvter >> name_num;
		name_str.append(name_num);
		name_str.append("The_Photo");
		fs_1 << "{:" << name_str << rvecs[i] << "}";
	}
	fs_1 << "]";
	//写入平移矢量组
	fs_1 << "tvecs" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		stringstream cvter;
		string name_str = "The_TvecsValue_of_", name_num;//组成字符串
		cvter << i;
		cvter >> name_num;
		name_str.append(name_num);
		name_str.append("The_Photo");
		fs_1 << "{:" << name_str << tvecs[i] << "}";
	}
	fs_1 << "]";
	//写入每幅图误差
	fs_1 << "Error_of_Per_Image" << "[";
	for (int i = 0; i < imageCount0; i++)
	{
		stringstream cvter;
		string name_str = "The_ErrorValue_of_", name_num;//组成字符串
		cvter << i;
		cvter >> name_num;
		name_str.append(name_num);
		name_str.append("The_Photo");
		fs_1 << "{:" << name_str << errorValue[i] << "}";
	}
	fs_1 << "]";
	fs_1.release();
	//创建
	FileStorage fs_2("Camera_Calibration_Data_1.xml", FileStorage::WRITE);
	//写入角点图像坐标
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
