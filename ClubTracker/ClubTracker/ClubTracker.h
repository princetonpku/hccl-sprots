#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <fstream>

#define USING_SINTEF
#ifdef USING_SINTEF
#include <GoTools/creators/ApproxCurve.h>
#include <GoTools/geometry/GoTools.h>
#include <GoTools/geometry/GoIntersections.h>
#include <GoTools/creators/SmoothCurve.h>
#endif


class CalibClass;

class ClubTracker
{
public:
	ClubTracker();
	~ClubTracker(void);


	//////////////////////////////////////////////////////////////////////////
	//   variables
	//
	//
	//////////////////////////////////////////////////////////////////////////

public:
	int num_frame;
	cv::Size imgsize;
	bool isTrackingFinished2d, isTrackingFinished3d;

	int club_langth;

	int coef_threshold,		// used when get the motion mask
		coef_num_errod,
		coef_num_dilt,

		gauss_siz,			// used when get the blurred motion mask
		gauss_sig,

		contour_threshold,	// used when get the contour

		hough_threshold;		// used when get the line segments
	double
		hough_minlangth,
		hough_maxgap,

		anglethr,			// used when get the line candidates
		distnthr1,			// two line segments are same group if angle difference and distance satisfy threshold values.
		distnthr2,			// 

		angle_d2_thr,		// for checking continuous

		canny1,
		canny2;

	cv::Point2f img_center;

	int up_start, up_end;
	int down_start, down_end;

	int num_upswing;
	int num_downswing;

	cv::Vec4f plane_upswing;
	cv::Vec4f plane_downswing;

	cv::Point3f centerOf_Up_swingplane;
	cv::Point3f centerOf_Down_swingplane;


	// index
	std::vector<int> main_indx;
	std::vector<int> sub_indx;

	std::vector<int> isEmpty_indx;
	std::vector<int> nonEmpty_indx;
	std::vector<int> upSwing_indx;
	std::vector<int> downSwing_indx;

	std::vector<int> outlier_up_indx;
	std::vector<int> outlier_down_indx;
	std::vector<int> outlier_up_indx2;
	std::vector<int> outlier_down_indx2;


	// 2d points of each camera
	std::vector<std::vector<cv::Vec4f>> main_lineVec;
	std::vector<std::vector<cv::Vec4f>> sub_lineVec;

	std::vector<cv::Point2f> main_pt1_2d;
	std::vector<cv::Point2f> main_pt2_2d;

	std::vector<cv::Point2f> sub_pt1_2d;
	std::vector<cv::Point2f> sub_pt2_2d;
	
	std::vector<cv::Vec4f> main_lines;	// full frame result
	std::vector<cv::Vec4f> sub_lines;

	std::vector<cv::Vec4f> main_lines1;	// origin tracking result
	std::vector<cv::Vec4f> sub_lines1;

	std::vector<cv::Vec4f> main_upswing[4];
	std::vector<cv::Vec4f> main_downswing[4];

	std::vector<cv::Vec4f> sub_upswing[4];
	std::vector<cv::Vec4f> sub_downswing[4];
	
	std::vector<float> up_para;
	std::vector<float> down_para;

	// 3d points of each camera
	cv::Point3f main_pt0;
	std::vector<cv::Point3f> main_pt1;
	std::vector<cv::Point3f> main_pt2;
	std::vector<cv::Point3f> main_n;

	cv::Point3f sub_pt0;
	std::vector<cv::Point3f> sub_pt1;
	std::vector<cv::Point3f> sub_pt2;
	std::vector<cv::Point3f> sub_n;

	// tracking result points
	std::vector<cv::Point3f> pt1;
	std::vector<cv::Point3f> pt2;
	std::vector<cv::Point3f> dir;

	std::vector<cv::Point3f> pt1_smoothed;
	std::vector<cv::Point3f> pt2_smoothed;
	std::vector<cv::Point3f> dir_smoothed;

	std::vector<cv::Vec6f> cross_line;

	//////////////////////////////////////////////////////////////////////////
	//   functions
	//
	//
	//////////////////////////////////////////////////////////////////////////
	
public:

	// initialize ant setting functions
	void Initialize( const int n_frame, const cv::Size imgsz);
	
	// tracking functions
	void Tracking2D(const std::vector<cv::Mat>& main_imgs, const std::vector<cv::Mat>& sub_imgs, CalibClass& calib);
	void dataconverting(const int n);
	void Tracking3D(CalibClass& calib);	
	void smoothing(const int iter, const double eps, const double alpha, CalibClass& calib);
	void saveTrackingResult();

	// drawing functions
	void drawClubTrajectory2d(cv::Mat& m_img, cv::Mat& s_img, int frame, CalibClass& calib);
	void drawClubTrajectory2dup(cv::Mat& m_img, cv::Mat& s_img, int frame, CalibClass& calib);
	void drawClubTrajectory2ddown(cv::Mat& m_img, cv::Mat& s_img, int frame, CalibClass& calib);

	cv::Mat VariableOptimization(const std::vector<cv::Mat>& m_imgs, const std::vector<cv::Mat>& s_imgs, const int frame, bool debug = false);
	cv::Mat VariableOptimization2( const std::vector<cv::Mat>& m_imgs, const std::vector<cv::Mat>& s_imgs, const int frame, bool debug = false);

	void dadtaUpdate();

private:

	// 2D tracking functions
	void Tracking2Dimg(const std::vector<cv::Mat>& img_buffer, std::vector<std::vector<cv::Vec4f>>& line_buffer, const int frame);
	void GpuTracking2Dimg(const std::vector<cv::Mat>& img_buffer, std::vector<std::vector<cv::Vec4f>>& line_buffer, const int frame);	
	std::vector<std::vector<cv::Point>> GetContour(const cv::Mat& img_buffer, const int thr, const int max_siz = 7);
	std::vector<std::vector<cv::Point>> GpuGetContour(const cv::gpu::GpuMat& img_buffer, const int thr, const int max_siz = 7);	
	cv::Mat GetMotionMask(const cv::Mat& img, const cv::Mat& img_before, const cv::Mat& img_after, int t, int n1, int n2);
	cv::gpu::GpuMat GpuGetMotionMask(const cv::gpu::GpuMat& img, const cv::gpu::GpuMat& img_before, const cv::gpu::GpuMat& img_after, int t, int n1, int n2);
	cv::Mat distanceInColorsplace(const cv::Mat& m1, const cv::Mat& m2);
	cv::gpu::GpuMat GpudistanceInColorsplace(const cv::gpu::GpuMat& m1, const cv::gpu::GpuMat& m2);
	std::vector<cv::Vec4f> lineGroupping( const std::vector<cv::Vec4i>& lines, const float anglethr, const float distthr1, const float distthr2, bool debug = false );
	std::vector<cv::Vec4f> lineGroupping2( const std::vector<cv::Vec4i>& lines, const float anglethr, const float distthr1, const float distthr2, bool debug = false );
	std::vector<cv::Vec4f> lineGroupping3( const std::vector<cv::Vec4i>& lines, const float anglethr, const float distthr1, const float distthr2, bool debug = false );
	cv::Vec4f getMiddleLine( const cv::Vec4f* lines );
	void modelAdjust(cv::Mat& model, const cv::Vec4f& l1, const cv::Vec4f& l2);
	void getIntersection(const cv::Mat& model, const std::vector<cv::Vec4f>& lines, const std::vector<float>& para, cv::Mat& data);
	void getIntersection2(const cv::Mat& model, const std::vector<cv::Vec4f>& lines, cv::Mat& data);
	void clublength_interp2d(std::vector<cv::Vec4f>& lines);
	cv::Mat pt2_interpolation(std::vector<cv::Vec4f>& lines, const int n);
#ifdef USING_SINTEF
	void pt2_interpolation(std::vector<cv::Vec4f>& lines, const std::vector<float>& para, const int n);
	void pt2_interpolation2(std::vector<cv::Vec4f>& lines, const std::vector<float>& para, const double thr);
	void interpolation(std::vector<cv::Point3f>& data, const std::vector<float>& para, const double thr);
#endif
	void pt1_interpolation(std::vector<cv::Vec4f>& lines, const int n);

	// 2D data post processing functions
	void sorting_LinePoints();
	void determineUpDown();
	void checkContinuous();
	void checkContinuous2();
	void getAngle(const cv::Mat& data, std::vector<float>& angle);
	void getDist(const std::vector<cv::Vec4f>& data, std::vector<float>& dist);
	void stereo2Dmatching(CalibClass& calib);
	void stereo2Dmatching(std::vector<cv::Vec4f>& m_data, std::vector<cv::Vec4f>& s_data, CalibClass& calib);
	void datacopy(const int from, const int to)
	{
		main_upswing[to]= main_upswing[from];
		main_downswing[to] = main_downswing[from];
		sub_upswing[to] = sub_upswing[from];
		sub_downswing[to] = sub_downswing[from];
	}
	
	// 3D reconstruction function
	void Tracking3Dpts(CalibClass& calib, const int frame);

	// 3D data post processing functions
	void PointInterpolation(std::vector<cv::Point3f>& pt3d, const std::vector<float>& wrongindx);
	void motionSmoothing3d(cv::Mat& data, cv::Mat& dir);
	void directionSmoothing(cv::Mat& dir, const std::vector<float>& param, const int n);
	void directionSmoothing(cv::Mat& dir, const int n);
	void quaternionSmoothing(cv::Mat& dir);

	cv::Vec4f getApproxPlane(int start_indx, int end_indx, int ref_indx);
	cv::Vec4f getApproxPlane(const cv::Mat pts);

	std::vector<cv::Mat> VariableOptimization(const std::vector<cv::Mat>& img_buffer, const int frame, bool debug = false);
	std::vector<cv::Mat> VariableOptimization2( const std::vector<cv::Mat>& img_buffer, const int frame, bool debug = false );

	std::vector<cv::Mat> GpuVariableOptimization(const std::vector<cv::Mat>& img_buffer, const int frame);


	// Smoothing function
	double myFunctionup(const std::vector<double>& upt, const std::vector<int>& upindx, CalibClass& calib);
	double myFunctiondown(const std::vector<double>& downt, const std::vector<int>& downindx, CalibClass& calib);
	void calDirup(std::vector<cv::Point3f>& dirup, CalibClass& calib);
	void calDirdown(std::vector<cv::Point3f>& dirdown, CalibClass& calib);
};

// data I/O functions
template<typename T> void saveData(const std::vector<T>& data, const char* filename)
{
	ofstream fout(filename);
	for (int i = 0; i<data.size(); ++i)
	{
		fout << data[i] << endl;
	}
	fout.close();
}
template<typename T> void saveData(const std::vector<cv::Point_<T>>& pts, const char* filename)
{
	ofstream fout(filename);
	for (int i = 0; i<pts.size(); ++i)
	{
		fout << pts[i].x << "\t" << pts[i].y << endl;
	}
	fout.close();
}
template<typename T> void saveData(const std::vector<cv::Point3_<T>>& pts, const char* filename)
{
	ofstream fout(filename);
	for (int i = 0; i<pts.size(); ++i)
	{
		fout << pts[i].x << "\t" << pts[i].y << "\t" << pts[i].z << endl;
	}
	fout.close();
}

template<typename T> void inputData(std::vector<T>& data, const char* filename)
{
	data.clear();
	ifstream fin(filename);
	T tem;
	while(!fin.eof())
	{
		fin >> tem;
		data.push_back(tem);
	}
	fin.close();
	data.pop_back();
}
template<typename T> void inputData(std::vector<cv::Point_<T>>& pts, const char* filename)
{
	pts.clear();
	ifstream fin(filename);
	Point_<T> tem;
	while(!fin.eof())
	{
		fin >> tem.x >> tem.y;
		pts.push_back(tem);
	}
	fin.close();
	pts.pop_back();
}
template<typename T> void inputData(std::vector<cv::Point3_<T>>& pts, const char* filename)
{
	pts.clear();
	ifstream fin(filename);
	Point3_<T> tem;
	while(!fin.eof())
	{
		fin >> tem.x >> tem.y >> tem.z;
		pts.push_back(tem);
	}
	fin.close();
	pts.pop_back();
}


