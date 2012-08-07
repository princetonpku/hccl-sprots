#include "CalibClass.h"
#include <fstream>

using namespace cv;
using namespace std;

CalibClass::CalibClass(void)
{
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	distCoeffs[0] = Mat::zeros(4,1,CV_64F);
	distCoeffs[1] = Mat::zeros(4,1,CV_64F);
	R = Mat::eye(3, 3, CV_64F);
	T = Mat(3,1,CV_64F, Scalar());
	R1 = Mat(3,3,CV_64F);
	R2 = Mat(3,3,CV_64F);
	P1 = Mat(3,4,CV_64F);
	P2 = Mat(3,4,CV_64F);
	E = Mat(3,3,CV_64F);
	F = Mat(3,3,CV_64F);

	toMain = Mat::eye(4,4,CV_64F);
	toWorld = Mat::eye(4,4,CV_64F);
}

CalibClass::~CalibClass(void)
{
}

void CalibClass::initCalib( const cv::Size bdsize, const float squaresize, const int n_images )
{
	stereo_success_count = 0;
	single_success_count[0] = 0;
	single_success_count[1] = 0;

	boardSize = bdsize;
	num_images = n_images;

	imagePoints[0].resize(num_images);
	imagePoints[1].resize(num_images);
	objectPoints.resize(num_images);

	for(int i = 0; i < num_images; i++ )
	{
		for(int j = 0; j < boardSize.height; j++ )
			for(int k = 0; k < boardSize.width; k++ )
				objectPoints[i].push_back(Point3f(j*squaresize, k*squaresize, 0));
	}
	
// 	main_pos = Point3d(0,0,0);
	toWorld.at<double>(0,0) = -1;
	toWorld.at<double>(1,1) = -1;
	toWorld.at<double>(2,2) = 1;
	toWorld.at<double>(3,3) = 1;
	toWorld.at<double>(0,3) = main_pos.x;
	toWorld.at<double>(1,3) = main_pos.y;
	toWorld.at<double>(2,3) = main_pos.z;
}

bool CalibClass::GetCorners_stereo( const cv::Mat& img1, const cv::Mat& img2, bool display, bool issinglecalibrated ) 
{
	int maxScale = 2;
	bool found;
	Mat img[2] = {img1, img2};

	imageSize = img1.size();

	int k;
	for (k = 0; k<2; ++k)
	{
		vector<Point2f>& corners = imagePoints[k][stereo_success_count];
		corners.clear();
		for( int scale = 1; scale <= maxScale; scale++ )
		{
			Mat timg;
			if( scale == 1 )
				timg = img[k];
			else
				resize(img[k], timg, Size(), scale, scale);
			found = findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH);
			if( found )
			{
				if( scale > 1 )
				{
					Mat cornersMat(corners);
					cornersMat *= 1./scale;
				}
				break;
			}
		}
		if( !found ) break;
		cornerSubPix(img[k], corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
		if(corners[0].y < corners[corners.size()-1].y)
			std::reverse(corners.begin(), corners.end());

		if( display )
		{
			Mat cimg, cimg1;
			cvtColor(img[k], cimg, CV_GRAY2BGR);
			drawChessboardCorners(cimg, boardSize, corners, found);
			double sf = 640./MAX(img[k].rows, img[k].cols);
			resize(cimg, cimg1, Size(), sf, sf);
			imshow("corners", cimg1);
			char c = (char)waitKey(500);
		}
	}
	if( k == 2 )
	{
		stereo_success_count++;
	}

	cout<<"stereo : "<<stereo_success_count<<"/"<<num_images<<endl;

	if (stereo_success_count == num_images)
	{
		StereoCalibration(issinglecalibrated);
		return true;
	}
	else return false;
}
bool CalibClass::GetCorners_single( const cv::Mat& img, int k, bool display)
{
	int maxScale = 2;
	bool found;
	imageSize = img.size();

	vector<Point2f>& corners = imagePoints[k][single_success_count[k]];
	corners.clear();
	for( int scale = 1; scale <= maxScale; scale++ )
	{
		Mat timg;
		if( scale == 1 )
			timg = img;
		else
			resize(img, timg, Size(), scale, scale);
		found = findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH);
		if( found )
		{
			single_success_count[k]++;
			if( scale > 1 )
			{
				Mat cornersMat(corners);
				cornersMat *= 1./scale;
			}
			break;
		}
	}
	if( !found ) return false;

	cornerSubPix(img, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
	if(corners[0].y > corners[corners.size()-1].y)
		std::reverse(corners.begin(), corners.end());

	if( display )
	{
		Mat cimg, cimg1;
		cvtColor(img, cimg, CV_GRAY2BGR);
		drawChessboardCorners(cimg, boardSize, corners, found);
		double sf = 640./MAX(img.rows, img.cols);
		resize(cimg, cimg1, Size(), sf, sf);
		imshow("corners", cimg1);
		char c = (char)waitKey(500);
	}	

	cout<< k << " : " << single_success_count[k]<<"/"<<num_images<<endl;

	if (single_success_count[k] == num_images)
	{
		SingleCalibration(k);
		return true;
	}
	else return false;
}

void CalibClass::StereoCalibration(bool issinglecalibrated)
{
	cout << "Running stereo calibration ...\n";

	if (issinglecalibrated)
	{
		rms_err = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
			CV_CALIB_FIX_INTRINSIC +
//			CV_CALIB_USE_INTRINSIC_GUESS +
//			CV_CALIB_FIX_ASPECT_RATIO +
//			CV_CALIB_ZERO_TANGENT_DIST +
//			CV_CALIB_SAME_FOCAL_LENGTH +
//			CV_CALIB_RATIONAL_MODEL +
//			CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2 + CV_CALIB_FIX_K3 +
			0);

		stereoRectify(cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

		//Precompute maps for cv::remap()
		initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
	} 
	else
	{
		cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
		cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
		distCoeffs[0] = Mat::zeros(5,1,CV_64F);
		distCoeffs[1] = Mat::zeros(5,1,CV_64F);

		rms_err = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
			cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, E, F,
			TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
//			CV_CALIB_FIX_ASPECT_RATIO +
//			CV_CALIB_FIX_PRINCIPAL_POINT +
//			CV_CALIB_ZERO_TANGENT_DIST +
//			CV_CALIB_SAME_FOCAL_LENGTH +
//			CV_CALIB_RATIONAL_MODEL +
			/*CV_CALIB_FIX_K1 + CV_CALIB_FIX_K2  +*/ CV_CALIB_FIX_K3 +
			0);

		stereoRectify(cameraMatrix[0], distCoeffs[0],
			cameraMatrix[1], distCoeffs[1],
			imageSize, R, T, R1, R2, P1, P2, Q,
			CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

		//Precompute maps for cv::remap()
		initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
	}	
	cout << "done with RMS error=" << rms_err << endl;

	toMain = Mat::eye(4,4,CV_64F);
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			toMain.at<double>(i,j) = R.at<double>(i,j);
		toMain.at<double>(i,3) = T.at<double>(i,0);
	}


	// OpenCV can handle left-right
	// or up-down camera arrangements
//	isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));	
	isVerticalStereo = false;

	CheckQuality();

	Print();
}
void CalibClass::SingleCalibration( int k )
{
	cout << "Running single calibration ...\n";

	cameraMatrix[k] = Mat::eye(3, 3, CV_64F);
 	distCoeffs[k] = Mat::zeros(5,1,CV_64F);

	cameraMatrix[k] = initCameraMatrix2D(objectPoints, imagePoints[k], imageSize);
	
	vector<Mat> r_vec(num_images), t_vec(num_images);

	rms_err_single[k] = calibrateCamera(objectPoints, imagePoints[k], imageSize, cameraMatrix[k], distCoeffs[k], r_vec, t_vec,
//		CV_CALIB_USE_INTRINSIC_GUESS +
//		CV_CALIB_FIX_PRINCIPAL_POINT +
//		CV_CALIB_FIX_ASPECT_RATIO +
//		CV_CALIB_SAME_FOCAL_LENGTH +
//		CV_CALIB_ZERO_TANGENT_DIST +
//		CV_CALIB_RATIONAL_MODEL +
		CV_CALIB_FIX_K3 /*+ CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5*/ +
		0);
		
	cout << k <<"th single RMS error =" << rms_err_single[k] << endl;

	Print(k);
}

void CalibClass::CheckQuality()
{
	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for( int i = 0; i < num_images; i++ )
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for( int k = 0; k < 2; k++ )
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
		}
		for( int j = 0; j < npt; j++ )
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +	imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] + imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}

	avg_reprojt_err = err/npoints;
	cout << "Average Reprojection err = " << avg_reprojt_err << endl;
}

void CalibClass::ShowRectified(const cv::Mat& img1, const cv::Mat& img2)
{	
	Mat canvas, img[2] = {img1, img2};
	double scalefactor;
	int w, h;
	if( !isVerticalStereo )
	{
		scalefactor = 600./MAX(imageSize.width, imageSize.height);
		scalefactor = 1;
		w = cvRound(imageSize.width*scalefactor);
		h = cvRound(imageSize.height*scalefactor);
		canvas.create(h, w*2, CV_8UC3);
	}
	else
	{
		scalefactor = 300./MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*scalefactor);
		h = cvRound(imageSize.height*scalefactor);
		canvas.create(h*2, w, CV_8UC3);
	}

	for(int k = 0; k < 2; k++ )
	{
		Mat rimg, cimg;
		remap(img[k], rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
		cvtColor(rimg, cimg, CV_GRAY2BGR);
		
		Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
		resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);

		Rect vroi(cvRound(validRoi[k].x*scalefactor), cvRound(validRoi[k].y*scalefactor), cvRound(validRoi[k].width*scalefactor), cvRound(validRoi[k].height*scalefactor)); 
		rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
	}

		if( !isVerticalStereo )
			for(int j = 0; j < canvas.rows; j += 16 )
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for(int j = 0; j < canvas.cols; j += 16 )
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);
	char c = (char)waitKey(0);

	imwrite("canvas.png",canvas);
}
void CalibClass::ShowUndist(const cv::Mat& img1, const cv::Mat& img2)
{	
	Mat canvas1, canvas2, img[2] = {img1, img2};
	double scalefactor;
	int w, h;
	if( !isVerticalStereo )
	{
		scalefactor = 600./MAX(imageSize.width, imageSize.height);
		scalefactor = 1;
		w = cvRound(imageSize.width*scalefactor);
		h = cvRound(imageSize.height*scalefactor);
		canvas1.create(h, w*2, CV_8UC3);
		canvas2.create(h, w*2, CV_8UC3);
	}
	else
	{
		scalefactor = 300./MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*scalefactor);
		h = cvRound(imageSize.height*scalefactor);
		canvas1.create(h*2, w, CV_8UC3);
		canvas2.create(h*2, w, CV_8UC3);
	}

	for(int k = 0; k < 2; k++ )
	{
		Mat rimg, cimg1, cimg2;
		undistort(img[k], rimg, cameraMatrix[k], distCoeffs[k], cameraMatrix[k]);
		cvtColor(img[k], cimg1, CV_GRAY2BGR);
		cvtColor(rimg, cimg2, CV_GRAY2BGR);

		Mat canvasPart1 = !isVerticalStereo ? canvas1(Rect(w*k, 0, w, h)) : canvas1(Rect(0, h*k, w, h));
		resize(cimg1, canvasPart1, canvasPart1.size(), 0, 0, CV_INTER_AREA);

		Mat canvasPart2 = !isVerticalStereo ? canvas2(Rect(w*k, 0, w, h)) : canvas2(Rect(0, h*k, w, h));
		resize(cimg2, canvasPart2, canvasPart2.size(), 0, 0, CV_INTER_AREA);
	}
	imshow("distort", canvas1);
	imshow("Undistort", canvas2);
	char c = (char)waitKey(0);
}

void CalibClass::savePara( const char* str )
{
	// save parameters
	FileStorage fs(str, CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];

		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs << "E" << E << "F" << F;

		fs.release();
	}
	else
		cout << "Error: can not save the parameters\n";
	loadPara(str);
}
void CalibClass::loadPara( const char* str )
{
	// load parameters
	FileStorage fs(str, CV_STORAGE_READ);
	if( fs.isOpened() )
	{
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];

		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs["E"] >> E;
		fs["F"] >> F;

		fs.release();
	}
	else
		cout << "Error: can not load the parameters\n";
		

	toMain = Mat::eye(4,4,CV_64F);
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			toMain.at<double>(i,j) = R.at<double>(i,j);
		toMain.at<double>(i,3) = T.at<double>(i,0);
	}
}
void CalibClass::Print()
{
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			cout << cameraMatrix[0].at<double>(i,j) << " ";
		cout << endl;
	}
	for(int i = 0; i < distCoeffs[0].rows; ++i)
	{
		cout << distCoeffs[0].at<double>(i, 0) << " ";
	}
	cout << endl;

	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			cout << cameraMatrix[1].at<double>(i,j) << " ";
		cout << endl;
	}
	for(int i = 0; i < distCoeffs[1].rows; ++i)
	{
		cout << distCoeffs[1].at<double>(i, 0) << " ";	
	}
	cout << endl;

	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			cout << R.at<double>(i,j) << " ";
		cout << endl;
	}
	cout << endl;

	for(int j = 0; j < 3; ++j)
		cout << T.at<double>(j,0) << " ";
	cout << endl;
}
void CalibClass::Print( int k )
{
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			cout << cameraMatrix[k].at<double>(i,j) << " ";
		cout << endl;
	}

	for(int i = 0; i < distCoeffs[k].rows; ++i)
	{
			cout << distCoeffs[k].at<double>(i, 0) << " ";	
	}

	cout << endl;
}
cv::Point3d CalibClass::cvtmain2world(cv::Point3d pt )
{
	Point3d result;
	result.x = pt.x*toWorld.at<double>(0,0)+pt.y*toWorld.at<double>(0,1)+pt.z*toWorld.at<double>(0,2)+toWorld.at<double>(0,3);
	result.y = pt.x*toWorld.at<double>(1,0)+pt.y*toWorld.at<double>(1,1)+pt.z*toWorld.at<double>(1,2)+toWorld.at<double>(1,3);
	result.z = pt.x*toWorld.at<double>(2,0)+pt.y*toWorld.at<double>(2,1)+pt.z*toWorld.at<double>(2,2)+toWorld.at<double>(2,3);
	return result;
}
cv::Point3d CalibClass::cvtsub2world(cv::Point3d pt )
{
	Mat t_temp(4,4,CV_64F);
	t_temp = toWorld*toMain;
// 	for(int i = 0; i < 4; ++i)
// 	for(int j = 0; j < 4; ++j)
// 	{
// 		t_temp.at<double>(i,j) = 0;
// 		for(int k = 0; k < 4; ++k)
// 			t_temp.at<double>(i,j) += t_World.at<double>(i,k)*t_Sub.at<double>(k,j);
// 	}

	Point3d result;
	result.x = pt.x*t_temp.at<double>(0,0)+pt.y*t_temp.at<double>(0,1)+pt.z*t_temp.at<double>(0,2)+t_temp.at<double>(0,3);
	result.y = pt.x*t_temp.at<double>(1,0)+pt.y*t_temp.at<double>(1,1)+pt.z*t_temp.at<double>(1,2)+t_temp.at<double>(1,3);
	result.z = pt.x*t_temp.at<double>(2,0)+pt.y*t_temp.at<double>(2,1)+pt.z*t_temp.at<double>(2,2)+t_temp.at<double>(2,3);
	return result;
}
Point2d CalibClass::cvtsub2main_epiLine(Point2d pt)
{
	Point2d result;

	Point3d Pt_l_origin = Point3d(0,0,0);
	Point3d Pt_l_h;
	Pt_l_h.x = (pt.x - cameraMatrix[0].at<double>(0, 2))/cameraMatrix[0].at<double>(0,0);
	Pt_l_h.y = (pt.y - cameraMatrix[0].at<double>(1, 2))/cameraMatrix[0].at<double>(1,1);
	Pt_l_h.z = 1;

	Point3d Pt_r_origin = Point3d(T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0));
	Point3d Pt_r_h;
	Pt_r_h.x = Pt_l_h.x*R.at<double>(0,0)+Pt_l_h.y*R.at<double>(0,1)+Pt_l_h.z*R.at<double>(0,2)+T.at<double>(0,0);
	Pt_r_h.y = Pt_l_h.x*R.at<double>(1,0)+Pt_l_h.y*R.at<double>(1,1)+Pt_l_h.z*R.at<double>(1,2)+T.at<double>(1,0);
	Pt_r_h.z = Pt_l_h.x*R.at<double>(2,0)+Pt_l_h.y*R.at<double>(2,1)+Pt_l_h.z*R.at<double>(2,2)+T.at<double>(2,0);

	Pt_r_origin.x = Pt_r_origin.x/Pt_r_origin.z*cameraMatrix[1].at<double>(0,0) + cameraMatrix[1].at<double>(0,2);
	Pt_r_origin.y = Pt_r_origin.y/Pt_r_origin.z*cameraMatrix[1].at<double>(1,1) + cameraMatrix[1].at<double>(1,2);
	Pt_r_origin.z = 1;

	Pt_r_h.x = Pt_r_h.x/Pt_r_h.z*cameraMatrix[1].at<double>(0,0) + cameraMatrix[1].at<double>(0,2);
	Pt_r_h.y = Pt_r_h.y/Pt_r_h.z*cameraMatrix[1].at<double>(1,1) + cameraMatrix[1].at<double>(1,2);
	Pt_r_h.z = 1;

	result.x = (Pt_r_h.y - Pt_r_origin.y)/(Pt_r_h.x - Pt_r_origin.x);
	result.y = Pt_r_origin.y - result.x * Pt_r_origin.x;

	return result;  // y = ax+b; --> return (a,b)
}
cv::Point3d CalibClass::calc_ref_MainCoord( cv::Point2d pt1, cv::Point2d pt2, double& _e1, double& _e2, double& _e3 )
{
	// pt1 : sub camera, pt2 : main camera
	Point3d result;

	Point3d Pt_l_origin = Point3d(T.at<double>(0,0), T.at<double>(1,0), T.at<double>(2,0));
	Point3d Pt_l_h;
	Pt_l_h.x = (pt1.x - cameraMatrix[0].at<double>(0, 2))/cameraMatrix[0].at<double>(0,0);
	Pt_l_h.y = (pt1.y - cameraMatrix[0].at<double>(1, 2))/cameraMatrix[0].at<double>(1,1);
	Pt_l_h.z = 1;

	Point3d Pt_r_origin;
	Point3d Pt_r_h;
	Pt_r_h.x = Pt_l_h.x*R.at<double>(0,0)+Pt_l_h.y*R.at<double>(0,1)+Pt_l_h.z*R.at<double>(0,2);
	Pt_r_h.y = Pt_l_h.x*R.at<double>(1,0)+Pt_l_h.y*R.at<double>(1,1)+Pt_l_h.z*R.at<double>(1,2);
	Pt_r_h.z = Pt_l_h.x*R.at<double>(2,0)+Pt_l_h.y*R.at<double>(2,1)+Pt_l_h.z*R.at<double>(2,2);

	Pt_l_h = Pt_r_h;

	Pt_r_origin = Point3d(0,0,0);
	Pt_r_h.x = (pt2.x - cameraMatrix[1].at<double>(0, 2))/cameraMatrix[1].at<double>(0,0);
	Pt_r_h.y = (pt2.y - cameraMatrix[1].at<double>(1, 2))/cameraMatrix[1].at<double>(1,1);
	Pt_r_h.z = 1;

	Point3d a,b,c;
	a.x = Pt_l_origin.x - Pt_r_origin.x;
	b.x = Pt_l_origin.y - Pt_r_origin.y;
	c.x = Pt_l_origin.z - Pt_r_origin.z;
	a.y = Pt_l_h.x;
	b.y = Pt_l_h.y;
	c.y = Pt_l_h.z;
	a.z = -Pt_r_h.x;
	b.z = -Pt_r_h.y;
	c.z = -Pt_r_h.z;

	double A,B,C,D,E;
	A = a.y*a.y + b.y*b.y +c.y*c.y;
	B = a.z*a.z + b.z*b.z +c.z*c.z;
	C = a.y*a.z + b.y*b.z +c.y*c.z;
	D = a.y*a.x + b.y*b.x +c.y*c.x;
	E = a.x*a.z + b.x*b.z +c.x*c.z;
	
	double t1 = (C*E-B*D)/(A*B-C*C);
	double t2 = (A*E-C*D)/(C*C-A*B);

	Point3d pt3_1 = Point3d(Pt_l_origin.x + t1*Pt_l_h.x, Pt_l_origin.y + t1*Pt_l_h.y, Pt_l_origin.z + t1*Pt_l_h.z);
	Point3d pt3_2 = Point3d(Pt_r_origin.x + t2*Pt_r_h.x, Pt_r_origin.y + t2*Pt_r_h.y, Pt_r_origin.z + t2*Pt_r_h.z);

	_e1 = Pt_l_origin.x + t1*Pt_l_h.x - (Pt_r_origin.x + t2*Pt_r_h.x);
	_e2 = Pt_l_origin.y + t1*Pt_l_h.y - (Pt_r_origin.y + t2*Pt_r_h.y);
	_e3 = Pt_l_origin.z + t1*Pt_l_h.z - (Pt_r_origin.z + t2*Pt_r_h.z);

	result = 0.5*(pt3_1+pt3_2);

	return result;
}
cv::Point3d CalibClass::getDir( const cv::Point2d& pt, int i )
{
	Point3d dir;	
	dir.x = (pt.x - cameraMatrix[i].at<double>(0,2))/cameraMatrix[i].at<double>(0,0);
	dir.y = (pt.y - cameraMatrix[i].at<double>(1,2))/cameraMatrix[i].at<double>(1,1);
	dir.z = 1;

	if (i==0)
	{
		return cvtmain2world(dir);
	}
	else if (i==1)
	{
		return cvtsub2world(dir);
	}
}

cv::Mat CalibClass::undistImage( const cv::Mat& img, const int k )
{
	Mat tem;
	undistort(img, tem, cameraMatrix[k], distCoeffs[k], cameraMatrix[k]);
	return tem;
}
void CalibClass::saveIntrinsicPara( const char* str )
{
	// save parameters
	FileStorage fs(str, CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];

		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";
}
void CalibClass::saveExtrinsicPara( const char* str )
{
	// save parameters
	FileStorage fs(str, CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs << "E" << E << "F" << F;

		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";
}

void CalibClass::loadIntrinsicPara( const char* str )
{
	// load parameters
	FileStorage fs(str, CV_STORAGE_READ);
	if( fs.isOpened() )
	{
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];

		fs.release();
	}
	else
		cout << "Error: can not load the intrinsic parameters\n";
}
void CalibClass::loadExtrinsicPara( const char* str )
{
	// load parameters
	FileStorage fs(str, CV_STORAGE_READ);
	if( fs.isOpened() )
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs["E"] >> E;
		fs["F"] >> F;
		fs.release();
	}
	else
		cout << "Error: can not load the extrinsic parameters\n";


	toMain = Mat::eye(4,4,CV_64F);
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < 3; ++j)
			toMain.at<double>(i,j) = R.at<double>(i,j);
		toMain.at<double>(i,3) = T.at<double>(i,0);
	}
}
