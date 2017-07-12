/*--------------Standard Library-------------*/
#include <iostream>
#include <fstream>
#include <math.h>
#include <iomanip>
//#include <thread>

/*--------------OpenCV-------------*/
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cv.h>

/*--------------Kinect-------------*/
#include <Kinect.h>
#include <windows.h>   

/*--------------M232-------------*/
#include "dllmain.h"

#using <System.dll>
using namespace System;
using namespace System::IO::Ports;
using namespace System::ComponentModel;
using namespace System::Threading;


using namespace std;
using namespace cv;

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

/*--------------------------------------*/
/*--------------Kinect v2---------------*/
/*--------------------------------------*/
//Initial (Sensor)
IKinectSensor* pSensor = nullptr;
ICoordinateMapper* Mapper = nullptr;
IDepthFrameReader* pFrameReaderDepth = nullptr;
IColorFrameReader* pFrameReaderColor = nullptr;

//Depth Map
int iWidthDepth = 0;
int iHeightDepth = 0;
UINT depthPointCount = 0;
UINT16* pBufferDepth = nullptr;   //Depth map origin format
UINT16 uDepthMin = 0, uDepthMax = 0;

//Color Map
int iWidthColor = 0;
int iHeightColor = 0;
UINT colorPointCount = 0;
UINT uBufferSizeColor = 0;
BYTE* pBufferColor = nullptr;    //Color map origin format (RGBA)


/*--------------------------------------*/
/*----------------OpenCV----------------*/
/*--------------------------------------*/
Mat mDepthImg;                    //Depth map UINT16
Mat mImg8bit;                     //Depth map CV_8U
Mat mColorImg;                   //Color map OpenCV format (BGRA)

//Initial
Rect2i ROI_rect;
Point ROI_p1, ROI_p2;												//start point and end point of ROI
bool ROI_S1 = false;												//flag of mouse event
bool ROI_S2 = false;
Mat ROI;
void onMouseROI(int event, int x, int y, int flags, void* param);
void FindROI(void);
//Tracking
int ROIcount = 0;
Point2i ROICenterColorS_Old, ROICenterColorS_New;				   //center of tracking object of ROI in Color
Point2i* ROIPixel = nullptr;
CameraSpacePoint* ROICameraSP = nullptr;
void MoveROI(void);


/*-------------------------------------*/
/*----------------Cam Calib------------*/
/*-------------------------------------*/
vector<Point2f> imageCorners;
vector<Point3f> objectCorners;

vector<vector<Point3f>> objectPoints;
vector<vector<Point2f>> imagePoints;

Mat cameraMatrix;
Mat distCoeffs;

//10 mm x 10 mm
const int cubeSize_Machine = 25;

Size boardSize_Machine(9, 5);
Size boardSize_ChessBoard(9, 6);
Size imageSize;


/*-------------------------------------*/
/*----------------Mt232----------------*/
/*-------------------------------------*/
bool MtHome(void);
void MtReflashWait(void);
void MtInit(void);
void MtMove(void);

bool IS_MT_MOVE = false;
bool IsMtMoving = false;
Rs232MotionData* md = new struct Rs232MotionData;

/*--------------------------------*/
/*--------------Mt232-------------*/
/*--------------------------------*/
void MtReflashWait()
{
	int i = mdr.wdt;
	int timeout = 0;

	while (i == mdr.wdt && timeout < 50)
	{
		MtReflash(md);
		timeout += 1;
		Sleep(1);
	}

}

bool MtHome()
{
	MtCmd("mt_emg 0");
	Sleep(100);
	MtCmd("mt_delay 20");
	MtCmd("mt_m_acc 20");
	MtCmd("mt_v_acc 20");
	MtCmd("mt_speed 30");
	MtCmd("mt_check_ot 0,0,0,0");
	MtCmd("mt_set_home_acc 50");
	MtCmd("mt_leave_home_speed 2,2,2,2");
	MtCmd("mt_go_home 50,50,10,50,2,2,1,255"); // 回home的速度(速度x, 速度y, 速度z, 速度u, 順序x, 順序y, 順序z, 順序u, 255 = u不回home)

	Sleep(1000);

	do{
		MtReflashWait();
		if (MtFlag(MotionStatus::emg_signal))
		{
			MtCmd("mt_abort_home");
			return false;
		}
	} while (MtFlag(MotionStatus::home_x) || MtFlag(MotionStatus::home_y) || MtFlag(MotionStatus::home_z)); //home_x 回到原點的過程 = true

	MtCmd("mt_home_finish");
	MtCmd("mt_speed1 30");
	MtCmd("mt_soft_limit 0,-2,300"); //Axe no., Left limit, Right limit
	MtCmd("mt_soft_limit 1,-2,300");
	MtCmd("mt_soft_limit 2,-2,100");
	MtCmd("mt_soft_limit 3,-450,450");
	MtCmd("mt_check_ot 1,1,1,0"); //enable limit (by axe no)
	MtCmd("mt_out 11,1"); //door switch
	MtCmd("mt_m_acc 150");  //door switch
	MtCmd("mt_v_acc 80");  //door switch


	return true;
}

void MtInit(void)
{
	array<System::String^>^ serialPorts = nullptr;
	try
	{
		// Get a list of serial port names.
		serialPorts = SerialPort::GetPortNames();
	}
	catch (Win32Exception^ ex)
	{
		Console::WriteLine(ex->Message);
	}

	Console::WriteLine("The following serial ports were found:");

	// Display each port name to the console.
	for each(System::String^ port in serialPorts)
	{
		Console::WriteLine(port);
	}

	//cout << MtTestDebby(2, 3) << endl;

	char ptr[5] = "COM3";
	//cout << "Connect I/O = " << MtConnect(ptr) << endl;

	long IS_CONECT = 0;

	//剛開機時，第一次連線都會失敗不知為何
	do
	{
		IS_CONECT = MtConnect(ptr);
		cout << "Connect I/O = " << IS_CONECT << endl;
	} while (IS_CONECT != 1);

	Sleep(50);

	bool IS_HOME = MtHome();
}

void MtMove(void)
{
	//IsMtMoving = true;

	bool IS_HOME = MtHome();

	MtReflash(md);
	cout << "md->x : " << md->x << endl;
	cout << "md->y : " << md->y << endl;
	Sleep(100);

	for (int i = 0; i < 5; i++)
	{
		MtCmd("mt_out 12,1"); //12是出膠 9是閃燈
		Sleep(100);
		MtCmd("mt_out 12,0");
		Sleep(100);
	}

	imageCorners.clear();

	for (int i = 0; i < boardSize_Machine.height; i++)
	{
		for (int j = 0; j < boardSize_Machine.width; j++)
		{
			char mybuffx[50], mybuffy[50], mybuffz[50];
			char commandx[60] = "mt_m_x ", commandy[60] = "mt_m_y ", commandz[60] = "mt_m_z ";
			
			sprintf(mybuffz, "%i", cubeSize_Machine * i);
			
			sprintf(mybuffx, "%i", cubeSize_Machine * j);

			//if (j % 2 == 0)
			//{
			//	sprintf(mybuffx, "%i", 25 * j);
			//}
			//else
			//{
			//	sprintf(mybuffx, "%i", 25 * 9 - 25 * j);
			//}

			strcat(commandx, mybuffx);
			strcat(commandz, mybuffz);

			MtCmd(commandx);
			MtCmd(commandz);

			cout << commandx << endl
				<< commandz << endl;

			//sleep(5000);

			do
			{
				MtReflash(md);

				//cout << "md->x : " << md->x << endl;
				//cout << "md->z : " << md->z << endl;

				//cout << endl << (md->x == 25 * j) << endl;

			} while (md->x != cubeSize_Machine * j || md->z != cubeSize_Machine * i);

			sleep(200);

			while (ROICenterColorS_Old.x > 1920 || ROICenterColorS_Old.y > 1080)
			{
				FindROI();
			}

			//FindROI();
			//circle(ROI,	Point2f(ROICenterColorS_New.x, ROICenterColorS_New.y), 1.0, Scalar(0, 0, 255));
			//imshow("Color Map", ROI);
			imageCorners.push_back(Point2f(ROICenterColorS_Old.x, ROICenterColorS_Old.y));
			cout << ROICenterColorS_Old.x << " " << ROICenterColorS_Old.y << endl << endl;


		}
	}

	objectCorners.clear();

	// The corners are at 3D location (X,Y,Z)= (i,j,0)
	for (int i = 0; i<boardSize_Machine.height; i++)
	{
		for (int j = 0; j<boardSize_Machine.width; j++)
		{
			//square size as 25mm x 25mm
			//objectCorners.push_back(Point3f(i*25, j*25, 0.0f));
			objectCorners.push_back(Point3f(j * cubeSize_Machine * 0.001, i * cubeSize_Machine*0.001, 0.0f));
		}
	}

	if (imageCorners.size() == boardSize_Machine.area())
	{
		imagePoints.push_back(imageCorners);
		objectPoints.push_back(objectCorners);
	}

	//cout << endl << "-------------imageCorners-------------" << endl;
	//for (int i = 0; i < imageCorners.size(); i++)
	//{
	//	cout << imageCorners.at(i) << endl;
	//}
	//cout << endl << "-------------imagePoints-------------" << endl;
	//for (int i = 0; i < imagePoints.size(); i++)
	//{
	//	cout << imagePoints.at(i) << endl;
	//}
	//cout << endl << "-------------objectCorners-------------" << endl;
	//for (int i = 0; i < objectCorners.size(); i++)
	//{
	//	cout << objectCorners.at(i) << endl;
	//}
	//cout << endl << "-------------objectPoints-------------" << endl;
	//for (int i = 0; i < objectPoints.size(); i++)
	//{
	//	cout << objectPoints.at(i) << endl;
	//}

	//vector<Mat> rvecs, tvecs;
	//// start calibration
	//calibrateCamera(objectPoints, // the 3D points
	//	imagePoints, // the image points
	//	imageSize,   // image size
	//	cameraMatrix,// output camera matrix
	//	distCoeffs,  // output distortion matrix
	//	rvecs, tvecs,// Rs, Ts
	//	0);       // set options

	//ofstream fs;
	//fs.open("test_Machine.txt");
	//fs << "intrinsic" << cameraMatrix << endl << endl;
	//fs << "distcoeff" << distCoeffs << endl << endl;


	//fs << "fx = " << cameraMatrix.at<double>(0) << endl;
	//fs << "fy = " << cameraMatrix.at<double>(4) << endl;
	//fs << "cx = " << cameraMatrix.at<double>(2) << endl;
	//fs << "cy = " << cameraMatrix.at<double>(5) << endl << endl;


	//fs << "k1 = " << cameraMatrix.at<double>(0) << endl;
	//fs << "k2 = " << cameraMatrix.at<double>(1) << endl;
	//fs << "p1 = " << cameraMatrix.at<double>(2) << endl;
	//fs << "p2 = " << cameraMatrix.at<double>(3) << endl;
	//fs << "k3 = " << cameraMatrix.at<double>(4) << endl;

	//fs.close();



}

void onMouseROI(int event, int x, int y, int flags, void* param)
{
	int thickness = 2;

	if (event == CV_EVENT_LBUTTONDOWN)
	{
		////当前鼠标位置（x，y）  
		ROI_rect.x = x;
		ROI_rect.y = y;

		ROI_S1 = true;
		ROI_S2 = false;

		//cout << "CV_EVENT_LBUTTONDOWN: " << ROI_rect.x << ", " << ROI_rect.y << endl;
	}
	else if (ROI_S1&&event == CV_EVENT_MOUSEMOVE)
	{
		ROI_S2 = true;

		ROI_p1 = Point(ROI_rect.x, ROI_rect.y);
		ROI_p2 = Point(x, y);

		ROI = mColorImg.clone();
		rectangle(ROI, ROI_p1, ROI_p2, Scalar(0, 255, 0), 2);
		cv::imshow("Color Map", ROI);

	}
	else if (ROI_S1&&event == CV_EVENT_LBUTTONUP)
	{
		ROI_S1 = false;

		ROI_rect.height = y - ROI_rect.y;
		ROI_rect.width = x - ROI_rect.x;

		ROI_p2 = Point(x, y);

		//cout << "CV_EVENT_LBUTTONUP: " << x << ", " << y << endl;
		//cout << "ROI: " << ROI_rect.width << ", " << ROI_rect.height << endl;
	}
}

void FindROI()
{
	//namedWindow("ROI");
	//namedWindow("YCrCb");

	/*--------------Find ROI-------------*/
	Mat ROI_Image = mColorImg.colRange(ROI_p1.x, ROI_p2.x + 1).rowRange(ROI_p1.y, ROI_p2.y + 1).clone();
	//imshow("ROI", ROI_Image);


	/*--------------BGR to YCrCb-------------*/
	Mat ROI_YCrCb;
	cvtColor(ROI_Image, ROI_YCrCb, /*CV_BGR2HSV*/CV_BGR2YCrCb);
	//imshow("YCrCb", ROI_YCrCb);


	/*-------------- Color Detection and Tracking-------------*/
	int rows = ROI_YCrCb.rows;
	int cols = ROI_YCrCb.cols;
	ROIcount = 0;
	//ROIDepthCount = 0;

	ROICenterColorS_New.x = 0;
	ROICenterColorS_New.y = 0;

	if (ROIPixel != nullptr)
	{
		delete[] ROIPixel;
		ROIPixel = nullptr;
	}
	ROIPixel = new Point2i[1000];

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			//cout << (float)ROI_YCrCb.at<Vec3b>(i, j)[0] << " " << (float)ROI_YCrCb.at<Vec3b>(i, j)[1] << " " << (float)ROI_YCrCb.at<Vec3b>(i, j)[2] << " " << endl;
			//cout << ROI_YCrCb.row(i).col(j) << endl << endl;

			int IsRed = (int)ROI_YCrCb.at<Vec3b>(i, j)[1];

			//threshold = 150 for fluorescent pink
			//threshold = 170 for red
			if (IsRed > 150)
			{
				ROI_Image.at<Vec4b>(i, j)[0] = 255;



				ROI_Image.at<Vec4b>(i, j)[1] = 0;
				ROI_Image.at<Vec4b>(i, j)[2] = 0;

				ROI.at<Vec4b>(i + ROI_p1.y, j + ROI_p1.x)[0] = 255;
				ROI.at<Vec4b>(i + ROI_p1.y, j + ROI_p1.x)[1] = 0;
				ROI.at<Vec4b>(i + ROI_p1.y, j + ROI_p1.x)[2] = 0;

				ROIPixel[ROIcount].x = j + ROI_p1.x;
				ROIPixel[ROIcount].y = i + ROI_p1.y;

				//cout << "ROIPixel[ROIcount]" << ROIPixel[ROIcount].x << " " << ROIPixel[ROIcount].y << endl;

				ROICenterColorS_New.x += ROIPixel[ROIcount].x;
				ROICenterColorS_New.y += ROIPixel[ROIcount].y;

				ROIcount++;
			}
		}
	}
	//cout << "ROI count" << ROIcount << endl;

	//imshow("ROI", ROI_Image);
	imshow("Color Map", ROI);




	if (ROIcount > 0)
	{
		ROICenterColorS_New.x = static_cast<int>(ROICenterColorS_New.x / ROIcount);
		ROICenterColorS_New.y = static_cast<int>(ROICenterColorS_New.y / ROIcount);

		ROICenterColorS_Old = ROICenterColorS_New;

		//cout << "Old: ( " << ROICenterColorS_Old.x << ", " << ROICenterColorS_Old.y << " )" << endl;
		//cout << "New: ( " << ROICenterColorS_New.x << ", " << ROICenterColorS_New.y << " )" << endl;

		//CameraSpaceROI();

	}
	else if (ROIcount == 0)
	{
		ROICenterColorS_Old.x = ROICenterColorS_New.x = 0;
		ROICenterColorS_Old.y = ROICenterColorS_New.y = 0;
	}

	ROI_Image.release();
	ROI_YCrCb.release();
	//ROI.release();

	//cout << "ROI dimenssion" << ROI_Image.cols << " " << ROI_Image.rows << endl;

}


int main()
{

	vector<vector<Point2i>> test;

	for (int k = 0; k < 3; k++)
	{
		vector<Point2i> line;
	
		for (int i = 0; i < 10; i++)
		{
			line.push_back(Point2i(k, i));
		}	
		test.push_back(line);

		cout << test[0].size() << endl;
	}
	
	cout << test.size() << endl;
	cout << test.at(0) << endl;
	cout << test[0].size()<< endl;



	MtInit();

#pragma region KinectStreamOpen
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description\n" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidthColor);
			pFrameDescription->get_Height(&iHeightColor);

			colorPointCount = iWidthColor * iHeightColor;
			uBufferSizeColor = colorPointCount * 4 * sizeof(BYTE);

			pBufferColor = new BYTE[4 * colorPointCount];
			//pCSPoints = new CameraSpacePoint[colorPointCount];

			imageSize.height = iHeightColor;
			imageSize.width = iWidthColor;
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pFrameReaderColor) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 3. Depth related code
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}
		if (pSensor->get_DepthFrameSource(&pFrameSource) == S_OK)
		{
			pFrameSource->get_DepthMinReliableDistance(&uDepthMin);
			pFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidthDepth);
			pFrameDescription->get_Height(&iHeightDepth);

			depthPointCount = iWidthDepth * iHeightDepth;
			pBufferDepth = new UINT16[depthPointCount];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pFrameReaderDepth) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}
		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}
#pragma endregion KinectStreamOpen

	mDepthImg = cv::Mat::zeros(iHeightDepth, iWidthDepth, CV_16UC1);
	mImg8bit = cv::Mat::zeros(iHeightDepth, iWidthDepth, CV_8UC1);
	mColorImg = cv::Mat::zeros(iHeightColor, iWidthColor, CV_8UC4);
	
	int key;
	int file_idx = 0;
	char filename[256];
	char fileidx[10];

	while (true)
	{

#pragma region KinectStreamUpdate
		/*--------------Read color data-------------*/
		IColorFrame* pFrameColor = nullptr;
		if (pFrameReaderColor->AcquireLatestFrame(&pFrameColor) == S_OK)
		{
			pFrameColor->CopyConvertedFrameDataToArray(uBufferSizeColor, pBufferColor, ColorImageFormat_Rgba);
			pFrameColor->CopyConvertedFrameDataToArray(uBufferSizeColor, mColorImg.data, ColorImageFormat_Bgra);

			pFrameColor->Release();
			pFrameColor = nullptr;

		}

		/*--------------Read depth data-------------*/
		IDepthFrame* pDFrameDepth = nullptr;
		if (pFrameReaderDepth->AcquireLatestFrame(&pDFrameDepth) == S_OK)
		{
			pDFrameDepth->CopyFrameDataToArray(depthPointCount, pBufferDepth);
			pDFrameDepth->CopyFrameDataToArray(depthPointCount, reinterpret_cast<UINT16*>(mDepthImg.data));

			pDFrameDepth->Release();
			pDFrameDepth = nullptr;
		}
#pragma endregion KinectStreamUpdate

		namedWindow("Depth Map");
		namedWindow("Color Map");

		mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);
		imshow("Depth Map", mImg8bit);
		
		cvSetMouseCallback("Color Map", onMouseROI, NULL);

#pragma region FlashScreen
		if (ROI_S1 == false && ROI_S2 == false)
		{
			// Initial State
			ROI = mColorImg.clone();
			imshow("Color Map", ROI);
		}
		else if (ROI_S2 == true)
		{
			// CV_EVENT_LBUTTONUP：ROI選取完畢
			if (ROI_S1 == false)
			{
				int thickness = 2;

				ROI = mColorImg.clone();
				rectangle(ROI, ROI_p1, ROI_p2, Scalar(0, 255, 0), thickness);

				for (int i = 0; i < imageCorners.size(); i++)
				{
					circle(ROI, imageCorners.at(i), 1.5, Scalar(0, 0, 255));
				}
				imshow("Color Map", ROI);

				FindROI();
			}
			//CV_EVENT_MOUSEMOVE：只選了左上角
			else
			{
				imshow("Color Map", ROI);
			}

		}
#pragma endregion  FlashScreen

		key = cvWaitKey(5);

		if (key == VK_ESCAPE)
		{
			break;
		}
		else if (key == 's' || key == 'S')
		{
			Mat fourChannel;
			cvtColor(mColorImg, fourChannel, CV_BGR2GRAY);
			//namedWindow("Gray");
			//imshow("Gray", fourChannel);

			sprintf(fileidx, "%03d", file_idx++);
			strcpy(filename, "SAVE_IMG");
			strcat(filename, fileidx);
			strcat(filename, ".bmp");
			
			imwrite(filename, fourChannel);
			printf("Image file %s saved.\n", filename);

		}
		else if (key == 'q' || key == 'Q')
		{

			if (ROICenterColorS_Old.x != 0 && ROICenterColorS_Old.y != 0)
			{
				//MtMove();
				//thread mThread(MtMove);
				//mThread.join();

				Thread^ thread1 = gcnew Thread(gcnew ThreadStart(MtMove));
				thread1->Name = "thread1";

				thread1->Start();
			}


		}
		else if (key == 'w' || key == 'W')
		{
			vector<Mat> rvecs, tvecs;
			// start calibration
			calibrateCamera(objectPoints, // the 3D points
				imagePoints, // the image points
				imageSize,   // image size
				cameraMatrix,// output camera matrix
				distCoeffs,  // output distortion matrix
				rvecs, tvecs,// Rs, Ts
				0);       // set options

			Mat m = Mat::ones(3, imagePoints[0].size(), CV_64FC1);
			Mat M = Mat::ones(3, objectPoints[0].size(), CV_64FC1);

			//for (int layer = 0; layer < imagePoints.size(); layer++)
			//{
			//	for (int i = 0; i < imagePoints[layer].size(); i++)
			//	{
			//		m.at<double>(0, i) = imagePoints[layer].at(i).x;
			//		m.at<double>(1, i) = imagePoints[layer].at(i).y;

			//		M.at<double>(0, i) = objectPoints[layer].at(i).x;
			//		M.at<double>(1, i) = objectPoints[layer].at(i).y;
			//	
			//	}
			//}
			for (int layer = 0; layer < imagePoints.size(); layer++)
			{
				for (int i = 0; i < imagePoints[layer].size(); i++)
				{
					m.at<double>(0, i) = imagePoints[layer].at(i).x;
					m.at<double>(1, i) = imagePoints[layer].at(i).y;

					M.at<double>(0, i) = objectPoints[layer].at(i).x;
					M.at<double>(1, i) = objectPoints[layer].at(i).y;

				}
			}

			cout << endl << m << endl;
			cout << endl << M << endl;


			ofstream fs;
			fs.open("test_Machine.txt");
			fs << "intrinsic" << cameraMatrix << endl << endl;
			fs << "distcoeff" << distCoeffs << endl << endl;


			fs << "fx = " << cameraMatrix.at<double>(0) << endl;
			fs << "fy = " << cameraMatrix.at<double>(4) << endl;
			fs << "cx = " << cameraMatrix.at<double>(2) << endl;
			fs << "cy = " << cameraMatrix.at<double>(5) << endl << endl;


			fs << "k1 = " << cameraMatrix.at<double>(0) << endl;
			fs << "k2 = " << cameraMatrix.at<double>(1) << endl;
			fs << "p1 = " << cameraMatrix.at<double>(2) << endl;
			fs << "p2 = " << cameraMatrix.at<double>(3) << endl;
			fs << "k3 = " << cameraMatrix.at<double>(4) << endl;

			fs.close();

			//Mat Rt = cameraMatrix.inv()*m*M.inv(DECOMP_SVD);

			Mat H = findHomography(objectPoints[0], imagePoints[0]);
			Mat Rt = cameraMatrix.inv()*H;

			cout << Rt;

			ofstream fs2;
			fs2.open("Coord.txt");
			for (int i = 0; i < 9; i++)
			{
				fs2 << fixed << setprecision(15) << Rt.at<double>(i) << endl;
			}
			fs2 << endl << Rt;
			fs2.close();

		}
		else if (key == 'e' || key == 'E')
		{
			Mat H = findHomography(objectPoints[0], imagePoints[0]);
			Mat Rt = cameraMatrix.inv()*H;

			cout << Rt;

			ofstream fs2;
			fs2.open("Coord.txt");
			for (int i = 0; i < 9; i++)
			{
				fs2 << fixed << setprecision(15) << Rt.at<double>(i) << endl;
			}
			fs2 << endl << Rt;
			fs2.close();

		}
		else if (key == 'r' || key == 'R')
		{
			file_idx = 0;
		}
		else if (key == 'c' || key == 'C')
		{
			destroyWindow("Depth Map");
			destroyWindow("Color Map");

			Mat img;
			Mat img2;

			//vector<std::vector<cv::Point3f>> objectPoints;
			//vector<std::vector<cv::Point2f>> imagePoints;



			Mat map1, map2;


			int loop = 1;
			int key;
			int file_idx = 0;
			int file_idx2 = 0;
			char filename[256];
			char fileidx[10];

			int i;
			//Size boardSize(9, 6);
			//Size imageSize(iWidthColor, iHeightColor);
			//vector<Point2f> imageCorners;
			//vector<Point3f> objectCorners;

			//imageSize.height = iHeightColor;
			//imageSize.width = iWidthColor;

			// The corners are at 3D location (X,Y,Z)= (i,j,0)
			for (int i = 0; i<boardSize_ChessBoard.height; i++)
			{
				for (int j = 0; j<boardSize_ChessBoard.width; j++)
				{
					//square size as 25mm x 25mm
					//objectCorners.push_back(Point3f(i*25, j*25, 0.0f));
					objectCorners.push_back(Point3f(0.025*i, 0.025*j, 0.0f));
				}
			}

			while (loop)
			{
				// find file:
				sprintf(fileidx, "%03d", file_idx++);
				strcpy(filename, "SAVE_IMG");
				strcat(filename, fileidx);
				strcat(filename, ".bmp");

				printf("IMG = %s\n", filename);

				if (ifstream(filename))
				{				
					
					img = imread(filename, 0);

					cvNamedWindow("File");
					imshow("File", img);

					cvWaitKey(400);

					cout << img.channels() << endl;
					cout << img.depth() << endl;
					cout << img.type() << endl;
					cout << (int)img.at<uchar>(100,100) << endl;

					if (img.empty())
						loop = 0;
					else
					{
						bool found = cv::findChessboardCorners(
							img, boardSize_ChessBoard, imageCorners);

						cout << imageCorners.size() << endl;
					
						if (found)
						{
							cornerSubPix(img, imageCorners,
								cv::Size(11, 11),
								cv::Size(-1, -1),
								cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
								30,      // max number of iterations
								0.1));   // min accuracy
						}
						else
						{
							cout << endl << "No corner was found" << endl;
							continue;
						}

						if (imageCorners.size() == boardSize_ChessBoard.area())
						{
							imagePoints.push_back(imageCorners);
							objectPoints.push_back(objectCorners);
						}
						img.copyTo(img2);
						drawChessboardCorners(img2, boardSize_ChessBoard, imageCorners, found);

						//cout << endl << "-------------imageCorners-------------" << endl;
						//for (int i = 0; i < imageCorners.size(); i++)
						//{
						//	cout << imageCorners.at(i) << endl;
						//}
						//cout << endl << "-------------imagePoints-------------" << endl;
						//for (int i = 0; i < imagePoints.size(); i++)
						//{
						//	cout << imagePoints.at(i) << endl;
						//}
						//cout << endl << "-------------objectCorners-------------" << endl;
						//for (int i = 0; i < objectCorners.size(); i++)
						//{
						//	cout << objectCorners.at(i) << endl;
						//}
						//cout << endl << "-------------objectPoints-------------" << endl;
						//for (int i = 0; i < objectPoints.size(); i++)
						//{
						//	cout << objectPoints.at(i) << endl;
						//}

						sprintf(fileidx, "%03d", file_idx2++);
						strcpy(filename, "Corner_IMG");
						strcat(filename, fileidx);
						strcat(filename, ".bmp");

						imwrite(filename, img2);
						printf("Image file %s saved.\n", filename);

						imshow("image", img2);

						key = cvWaitKey(400);

						////key = cvWaitKey( 5 );
						//if (key == 27)
						//{
						//	loop = 0;
						//}
					}
				}
				else
				{
					cout << endl << "File not exist" << endl;
					break;
				}


			}

			vector<Mat> rvecs, tvecs;
			// start calibration
			calibrateCamera(objectPoints, // the 3D points
				imagePoints, // the image points
				imageSize,   // image size
				cameraMatrix,// output camera matrix
				distCoeffs,  // output distortion matrix
				rvecs, tvecs,// Rs, Ts
				0);       // set options

			//cout << cameraMatrix.depth() << endl;
			//cout << cameraMatrix.type() << endl;

			//cout << distCoeffs.depth() << endl;
			//cout << distCoeffs.type() << endl;

			//FileStorage fs("test.txt", FileStorage::WRITE);
			//fs << "intrinsic" << cameraMatrix;
			//fs << "distcoeff" << distCoeffs;

			//cout << cameraMatrix.at<double>(0);


			Mat m = Mat::ones(3, imagePoints[0].size(), CV_64FC1);
			Mat M = Mat::ones(3, objectPoints[0].size(), CV_64FC1);

			//for (int layer = 0; layer < imagePoints.size(); layer++)
			//{
			//	for (int i = 0; i < imagePoints[layer].size(); i++)
			//	{
			//		m.at<double>(0, i) = imagePoints[layer].at(i).x;
			//		m.at<double>(1, i) = imagePoints[layer].at(i).y;

			//		M.at<double>(0, i) = objectPoints[layer].at(i).x;
			//		M.at<double>(1, i) = objectPoints[layer].at(i).y;

			//	}
			//}


			Mat H = findHomography(objectPoints[0], imagePoints[0]);

			for (int i = 0; i < imagePoints[0].size(); i++)
			{
				m.at<double>(0, i) = imagePoints[0].at(i).x;
				m.at<double>(1, i) = imagePoints[0].at(i).y;

				M.at<double>(0, i) = objectPoints[0].at(i).x;
				M.at<double>(1, i) = objectPoints[0].at(i).y;

			}
			cout << endl << m << endl;
			cout << endl << M << endl;

			//Mat Cam = Mat::zeros(3, 3, CV_64FC1);

			//for (int i = 0; i < cameraMatrix.cols; i++)
			//{
			//	Cam.at<double>((int)(i / 3), i % 3) = cameraMatrix.at<double>(i);
			//}
			//cout << Cam;

			//Mat Rt = cameraMatrix.inv()*m*M.inv(DECOMP_SVD);
			Mat Rt = cameraMatrix.inv()*H;
			cout << H << endl;
			cout << Rt << endl;

			//Mat R1 = Mat::zeros(3, 1, CV_64FC1);
			//Mat R2 = Mat::zeros(3, 1, CV_64FC1);
			//Mat T = Mat::zeros(3, 1, CV_64FC1);

			//R1.at<double>(0) = Rt.at<double>(0, 0);
			//R1.at<double>(1) = Rt.at<double>(1, 0);
			//R1.at<double>(2) = Rt.at<double>(2, 0);

			//R2.at<double>(0) = Rt.at<double>(0, 1);
			//R2.at<double>(1) = Rt.at<double>(1, 1);
			//R2.at<double>(2) = Rt.at<double>(2, 1);

			//T.at<double>(0) = Rt.at<double>(0, 2);
			//T.at<double>(1) = Rt.at<double>(1, 2);
			//T.at<double>(2) = Rt.at<double>(2, 2);

			//Mat R3 = R1.cross(R2);
			//R2 = R3.cross(R1);

			ofstream fs2;
			fs2.open("Coord.txt");
			for (int i = 0; i < 9; i++)
			{
				fs2 << fixed << setprecision(15) << Rt.at<double>(i) << endl;
			}
			fs2 << endl << Rt;

			//fs2 << R1.at<double>(0) << endl;
			//fs2 << R1.at<double>(1) << endl;
			//fs2 << R1.at<double>(2) << endl;

			//fs2 << R2.at<double>(0) << endl;
			//fs2 << R2.at<double>(1) << endl;
			//fs2 << R2.at<double>(2) << endl;

			//fs2 << R3.at<double>(0) << endl;
			//fs2 << R3.at<double>(1) << endl;
			//fs2 << R3.at<double>(2) << endl;

			//fs2 << T.at<double>(0) << endl;
			//fs2 << T.at<double>(1) << endl;
			//fs2 << T.at<double>(2) << endl;

			fs2.close();

			ofstream fs;
			fs.open("test.txt");
			fs << "intrinsic" << cameraMatrix << endl << endl;
			fs << "distcoeff" << distCoeffs << endl << endl;
			

			fs << "fx = " << cameraMatrix.at<double>(0) << endl;
			fs << "fy = " << cameraMatrix.at<double>(4) << endl;
			fs << "cx = " << cameraMatrix.at<double>(2) << endl;
			fs << "cy = " << cameraMatrix.at<double>(5) << endl << endl;


			fs << "k1 = " << cameraMatrix.at<double>(0) << endl;
			fs << "k2 = " << cameraMatrix.at<double>(1) << endl;
			fs << "p1 = " << cameraMatrix.at<double>(2) << endl;
			fs << "p2 = " << cameraMatrix.at<double>(3) << endl;
			fs << "k3 = " << cameraMatrix.at<double>(4) << endl;
			
			fs.close();

		}

	}

	// 3b. release frame reader
	pFrameReaderColor->Release();
	pFrameReaderColor = nullptr;

	pFrameReaderDepth->Release();
	pFrameReaderDepth = nullptr;

	// 1c. Close Sensor
	pSensor->Close();

	// 1d. Release Sensor
	pSensor->Release();
	pSensor = nullptr;







	return 0;
}