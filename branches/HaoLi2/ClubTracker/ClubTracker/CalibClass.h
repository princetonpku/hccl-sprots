#pragma once
#include <opencv2/opencv.hpp>

class CalibClass
{
public:
	CalibClass();
	~CalibClass(void);

	int stereo_success_count;
	int single_success_count[2];
	int num_images;

	cv::Rect validRoi[2];
	cv::Mat rmap[2][2];
	
	cv::Point3d main_pos;

	void initCalib(const cv::Size bssize = cv::Size(8, 7), const float sqaresize = 65, const int n_images = 15);

	bool GetCorners_stereo(const cv::Mat& img1, const cv::Mat& img2, bool display = true, bool issinglcalibrated = true);
	bool GetCorners_single(const cv::Mat& img, int k = 0, bool display = true);

	void StereoCalibration(bool issinglecalibrated);
	void SingleCalibration(int k);
	
	void CheckQuality();
	
	void savePara(const char* str);
	void saveIntrinsicPara(const char* str);
	void saveExtrinsicPara(const char* str);
	void loadPara(const char* str);
	void loadIntrinsicPara(const char* str);
	void loadExtrinsicPara(const char* str);
	

	void ShowRectified(const cv::Mat& img1, const cv::Mat& img2);
	void ShowUndist(const cv::Mat& img1, const cv::Mat& img2);
	
	void Print();
	void Print(int k);


	cv::Point3d cvtmain2world(cv::Point pt);
	cv::Point3d cvtsub2world(cv::Point pt);

	cv::Point3d cvtmain2world(cv::Point3d pt);
	cv::Point3d cvtsub2world(cv::Point3d pt);

	cv::Point2d cvtsub2main_epiLine(cv::Point2d pt);
	cv::Point3d calc_ref_MainCoord(cv::Point2d pt1, cv::Point2d pt2, double& _e1, double& _e2, double& _e3);

	cv::Point3d getDir(const cv::Point2d& pt, int i);

	cv::Mat undistImage(const cv::Mat& img, const int k);

	cv::Size boardSize;
	cv::Size imageSize;

	std::vector<std::vector<cv::Point2f>> imagePoints[2];
	std::vector<std::vector<cv::Point3f>> objectPoints;

	cv::Mat cameraMatrix[2], distCoeffs[2];
	cv::Mat R, T, E, F;

	cv::Mat R1, R2, P1, P2, Q;

	cv::Mat toMain, toWorld;

	double rms_err;
	double rms_err_single[2];
	double avg_reprojt_err;

	bool isVerticalStereo;
};

