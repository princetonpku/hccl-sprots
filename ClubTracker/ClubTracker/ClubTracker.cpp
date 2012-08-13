#include "ClubTracker.h"
#include "CalibClass.h"
#include "Imgprocessing.h"
#include "ShortestPath.h"

#include "LSWMS.h"

using namespace cv;
using namespace std;

float rad2degree = 180/CV_PI;
float degree2rad = CV_PI/180;


ClubTracker::ClubTracker()
	: club_langth(600) ,isTrackingFinished2d(false), isTrackingFinished3d(false)
	,coef_threshold(40), contour_threshold(100), coef_num_errod(1), coef_num_dilt(1)
	,hough_threshold(30), hough_minlangth(30), hough_maxgap(15)
	,anglethr(5), distnthr1(40), distnthr2(2), gauss_siz(3), gauss_sig(10)
	,angle_d2_thr(20)
	,canny1(200)
	,canny2(100)
{
	rad2degree = 180/CV_PI;
	degree2rad = CV_PI/180;
}

ClubTracker::~ClubTracker(void)
{
}

// initialize and setting
void ClubTracker::Initialize( const int n_frame, const cv::Size imgsz )
{
	num_frame = n_frame;
	isTrackingFinished2d = false;
	isTrackingFinished3d = false;

	main_lineVec.clear();
	main_lineVec.resize(num_frame);
	sub_lineVec.clear();
	sub_lineVec.resize(num_frame);

	main_indx.clear();
	sub_indx.clear();

	isEmpty_indx.clear();
	nonEmpty_indx.clear();
	
	main_lines1.clear();
	sub_lines1.clear();
	
	main_lines.clear();	
	sub_lines.clear();
	main_pt1_2d.clear();
	main_pt2_2d.clear();
	sub_pt1_2d.clear();
	sub_pt2_2d.clear();
	
	main_pt1.clear();
	main_pt2.clear();
	main_n.clear();
	main_pt1.resize(num_frame);
	main_pt2.resize(num_frame);
	main_n.resize(num_frame);

	sub_pt1.clear();
	sub_pt2.clear();
	sub_n.clear();
	sub_pt1.resize(num_frame);
	sub_pt2.resize(num_frame);
	sub_n.resize(num_frame);

	pt1.clear();
	pt2.clear();
	dir.clear();
	pt1.resize(num_frame);
	pt2.resize(num_frame);
	dir.resize(num_frame);

	cross_line.clear();
	cross_line.resize(num_frame);

	upSwing_indx.clear();
	downSwing_indx.clear();

	imgsize = imgsz;
	img_center = Point2f(imgsize.width*0.5, imgsize.height*0.5);	
}

// tracking functions
void ClubTracker::Tracking2D( const std::vector<cv::Mat>& main_imgs, const std::vector<cv::Mat>& sub_imgs, CalibClass& calib )
{
	std::vector<cv::Mat> t = VariableOptimization(main_imgs, 99);
	int i = 0;
	// 2d tracking
#pragma omp parallel for num_threads(4)
	for (i = 0; i<num_frame; ++i)
	{
		Tracking2Dimg(main_imgs, main_lineVec, i);
		Tracking2Dimg(sub_imgs, sub_lineVec, i);
	}

	main_indx = ShortestPath(main_lineVec).findPath();
	sub_indx = ShortestPath(sub_lineVec).findPath();


	sorting_LinePoints();
 	checkContinuous2();
	
// 	int start = 0;
// 	int end = 72;
// 	{
// 		Mat img(main_imgs[0].size(), CV_8UC3, Scalar());
// 		for (int i = start; i<=end; ++i)
// 		{
// 			if(isEmpty_indx[i])
// 				drawLineSegment(img, main_lineVec[i][main_indx[i]], Scalar(0,0,255));
// 		}
// 		imshow("a", img);
// 		waitKey(0);
// 	}
// 	{
// 		Mat img(main_imgs[0].size(), CV_8UC3, Scalar());
// 		for (int i = start; i<=end; ++i)
// 		{
// 			if(isEmpty_indx[i])
// 				drawLineSegment(img, sub_lineVec[i][sub_indx[i]], Scalar(0,0,255));
// 		}
// 		imshow("a", img);
// 		waitKey(0);
// 	}

	determineUpDown();

	//////////////////////////////////////////////////////////////////////////
	datacopy(0, 1);
	stereo2Dmatching(main_upswing[1], sub_upswing[1], calib);
	stereo2Dmatching(main_downswing[1], sub_downswing[1], calib);

	//////////////////////////////////////////////////////////////////////////
	datacopy(1, 2);
	int dataindx = 2;
	double thr = 10.0;
	int upswing_degree = 5;
	int downswing_degree = 10;
#ifdef USING_SINTEF
// 	pt2_interpolation2(main_upswing[dataindx], up_para, thr);
// 	pt2_interpolation2(sub_upswing[dataindx], up_para, thr);
// 	pt2_interpolation2(main_downswing[dataindx], down_para, thr);
// 	pt2_interpolation2(sub_downswing[dataindx], down_para, thr);
	pt2_interpolation(main_upswing[dataindx], up_para, upswing_degree);
	pt2_interpolation(sub_upswing[dataindx], up_para, upswing_degree);
	pt2_interpolation(main_downswing[dataindx], down_para, downswing_degree);
	pt2_interpolation(sub_downswing[dataindx], down_para, downswing_degree);

#endif
#ifndef USING_SINTEF
	pt2_interpolation(main_upswing[dataindx], upswing_degree);
	pt2_interpolation(sub_upswing[dataindx], upswing_degree);
	pt2_interpolation(main_downswing[dataindx], downswing_degree);
	pt2_interpolation(sub_downswing[dataindx], downswing_degree);
#endif

	//////////////////////////////////////////////////////////////////////////
	datacopy(2, 3);	
	stereo2Dmatching(calib);
	dataindx = 3;
#ifdef USING_SINTEF
	pt2_interpolation(main_upswing[dataindx], up_para, upswing_degree);
	pt2_interpolation(sub_upswing[dataindx], up_para, upswing_degree);
	pt2_interpolation(main_downswing[dataindx], down_para, downswing_degree);
	pt2_interpolation(sub_downswing[dataindx], down_para, downswing_degree);
#endif

	dataconverting(3);
}
void ClubTracker::dataconverting( const int n )
{
	main_lines.clear();
	main_lines.resize(num_frame);
	sub_lines.clear();
	sub_lines.resize(num_frame);

	for (int i = 0; i<upSwing_indx.size(); ++i)
	{
		main_lines[upSwing_indx[i]] = main_upswing[n][i];
		sub_lines[upSwing_indx[i]] = sub_upswing[n][i];
	}

	for (int i = 0; i<downSwing_indx.size(); ++i)
	{
		main_lines[downSwing_indx[i]] = main_downswing[n][i];
		sub_lines[downSwing_indx[i]] = sub_downswing[n][i];
	}

	main_pt1_2d = Mat(main_lines).reshape(1).colRange(0,2).reshape(2).clone();
	main_pt2_2d = Mat(main_lines).reshape(1).colRange(2,4).reshape(2).clone();
	sub_pt1_2d = Mat(sub_lines).reshape(1).colRange(0,2).reshape(2).clone();
	sub_pt2_2d = Mat(sub_lines).reshape(1).colRange(2,4).reshape(2).clone();

	isTrackingFinished2d = true;
}
void ClubTracker::Tracking3D(CalibClass& calib)
{
	main_pt0 = calib.cvtmain2world(Point3d(0,0,0));
	sub_pt0 = calib.cvtsub2world(Point3d(0,0,0));

	// 3d tracking
	for (int i = 0; i<num_frame; ++i)
	{
		Tracking3Dpts(calib, i);
	}
	isTrackingFinished3d = true;

	smoothing(200, 0.015, 1.2, calib);
}


// 2d tracking functions
void ClubTracker::Tracking2Dimg( const std::vector<cv::Mat>& img_buffer, std::vector<std::vector<cv::Vec4f>>& line_buffer, const int frame )
{
	Size img_sz = img_buffer[0].size();
	Mat imgs[3], yuv[3], msk, msked;

	// get the motion mask and masked img=======================================
	imgs[0] = img_buffer[frame].clone();
	if (frame == 0)
	{
		imgs[1] = img_buffer[frame+1].clone();
		imgs[2] = img_buffer[frame].clone();
	}
	else if (frame == num_frame-1)
	{
		imgs[1] = img_buffer[frame-1].clone();
		imgs[2] = img_buffer[frame].clone();
	}
	else
	{
		imgs[1] = img_buffer[frame-1].clone();
		imgs[2] = img_buffer[frame+1].clone();
	}

	for (int i = 0; i<3; ++i)
		cvtColor(imgs[i], yuv[i], CV_BGR2YUV);	

	msk = GetMotionMask(yuv[0], yuv[1], yuv[2], coef_threshold, coef_num_errod, coef_num_dilt);
	// 	msk = GetMotionMask(imgs[0], imgs[1], imgs[2], coef_threshold, coef_num_errod, coef_num_dilt);

	int gsiz = gauss_siz*2+1;
	GaussianBlur(msk, msk, Size(gsiz,gsiz), gauss_sig);	

// 	if (0)
// 	{
// 		Mat edge;
// 		Canny(msk, edge, canny1, canny2);
// 		vector<cv::Vec4i> lin;
// 		HoughLinesP(edge, lin, 1, CV_PI/180, hough_threshold, hough_minlangth, hough_maxgap);		
// 		if (lin.empty()) return;
// 		vector<Vec4f> new_lines2 = temfunc(lin, anglethr, distnthr1, distnthr2);
// 		if (new_lines2.empty()) return;
// 		line_buffer[frame].clear();
// 		for (int i = 0; i<new_lines2.size(); ++i)
// 		{		
// 			Point2f p1(new_lines2[i][0], new_lines2[i][1]);
// 			Point2f p2(new_lines2[i][3], new_lines2[i][2]);
// 	
// 			if (norm(p1-img_center)<norm(p2-img_center))
// 			{
// 				line_buffer[frame].push_back(new_lines2[i]);
// 			}
// 			else
// 			{
// 				line_buffer[frame].push_back(Vec4i(new_lines2[i][2], new_lines2[i][3], new_lines2[i][0], new_lines2[i][1]));
// 			}		
// 		}
// 		return;
// 	}

	vector<vector<Point>> contours = GetContour(msk, contour_threshold);

	vector<vector<cv::Vec4i>> new_lines;
	new_lines.reserve(contours.size());

	for (int i = 0; i<contours.size(); ++i)
	{
		// drawing approximated contour
		Mat tem(img_sz.height, img_sz.width, CV_8UC1, Scalar(0));
		drawVector(tem, contours[i], Scalar(255), 1, false);

		// get bounding rect of contour
		Rect roi_Rect = boundingRect(Mat(contours[i]));
		Mat roi(tem, roi_Rect);
		Point shift_pt(roi_Rect.x, roi_Rect.y);

		// perform hough transform
		vector<cv::Vec4i> lin;
		HoughLinesP(roi, lin, 1, CV_PI/180, hough_threshold, hough_minlangth, hough_maxgap);

		if (lin.size()>1)
		{
			vector<Vec4i> tem(lin.size(), Vec4i(shift_pt.x, shift_pt.y, shift_pt.x, shift_pt.y));
			new_lines.push_back(Mat(Mat(lin)+Mat(tem)));
		}
	}

	vector<vector<Vec4f>> new_lines2;
	new_lines2.reserve(new_lines.size());
	for (int i = 0; i<new_lines.size(); ++i)
	{
		vector<Vec4f> tem = lineGroupping(new_lines[i], anglethr, distnthr1, distnthr2);
		if (tem.size()>0) new_lines2.push_back(tem);
	}

	line_buffer[frame].clear();
	for (int i = 0; i<new_lines2.size(); ++i)
	{
		for (int j = 0; j<new_lines2[i].size(); ++j)
		{
			Point2f p1(new_lines2[i][j][0], new_lines2[i][j][1]);
			Point2f p2(new_lines2[i][j][3], new_lines2[i][j][2]);

			if (norm(p1-img_center)<norm(p2-img_center))
			{
				line_buffer[frame].push_back(new_lines2[i][j]);
			}
			else
			{
				line_buffer[frame].push_back(Vec4i(new_lines2[i][j][2], new_lines2[i][j][3], new_lines2[i][j][0], new_lines2[i][j][1]));
			}
		}		
	}
}

std::vector<std::vector<cv::Point>> ClubTracker::GetContour( const cv::Mat& img_buffer, const int thr, const int max_siz /*= 7*/ )
{
	vector<vector<Point>> contour;
	// get segmented contours=============================================================
	int n_contour = 0,
		n_line = 0;

	findContours(img_buffer.clone(), contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	n_contour = contour.size();

	if (n_contour>0)
	{
		sort(contour.begin(), contour.end(),[](vector<Point>& a, vector<Point>& b){return a.size()>b.size();});

		if(n_contour>max_siz)
		{
			n_contour = max_siz;
			contour.resize(n_contour);
		}

		for (int i = 0; i<n_contour; ++i)
		{
			if (contour[n_contour-1-i].size() < thr ) contour.pop_back();
		}
		n_contour = contour.size();
	}

	return contour;
}
cv::Mat ClubTracker::GetMotionMask(const cv::Mat& img, const cv::Mat& img_before, const cv::Mat& img_after, int t, int n1, int n2)
{
	cv::Mat tem1, tem2, msk;
	tem1 = distanceInColorsplace(img, img_before);
	tem2 = distanceInColorsplace(img, img_after);

	threshold(tem1, tem1, t, 255, CV_THRESH_BINARY);
	tem1.convertTo(tem1, CV_8UC1);
	erode(tem1, tem1, Mat(), Point(-1,-1), n1);
	dilate(tem1, tem1, Mat(), Point(-1,-1), n1+1);
	erode(tem1, tem1, Mat(), Point(-1,-1), 1);

	threshold(tem2, tem2, t, 255, CV_THRESH_BINARY);
	tem2.convertTo(tem2, CV_8UC1);
	erode(tem2, tem2, Mat(), Point(-1,-1), n1);
	dilate(tem2, tem2, Mat(), Point(-1,-1), n1+1);
	erode(tem2, tem2, Mat(), Point(-1,-1), 1);

	bitwise_and(tem1, tem2, msk);

	dilate(msk, msk, cv::Mat(), cv::Point(-1,-1), 2);
	erode(msk, msk, cv::Mat(), cv::Point(-1,-1), 2);

	return msk;
}
cv::Mat ClubTracker::distanceInColorsplace(const cv::Mat& m1, const cv::Mat& m2)
{
	cv::Mat tem, splt[3], rslt;

	cv::Mat(abs(m1 - m2)).convertTo(tem, CV_32FC3);

	tem = tem.mul(tem);
	split(tem, splt);

	float w1 = 0.2,
		w2 = 0.4,
		w3 = 0.4;

	rslt = w1*splt[0] + w2*splt[1] + w3*splt[2];

	return rslt;
}


// 여러 선분중 나란하고 가까운 것들을 합친다.
std::vector<cv::Vec4f> ClubTracker::lineGroupping( const std::vector<cv::Vec4i>& lines, const float anglethr, const float distthr1, const float distthr2, bool debug /*= false */ )
{
	Mat img;
	if (debug)
	{
		img = Mat(700,700,CV_8UC3, Scalar());//--> for debug
		drawLines(img, lines, colors[0], 2);//--> for debug
		imshow("a", img);//--> for debug
	 	waitKey(0);//--> for debug
	}

	// 먼저 각도를 모두 구한다.
	int n = lines.size();	
	vector<float> angles;
	angles.reserve(n);
	for_each(lines.begin(), lines.end(),[&angles](const Vec4i& line)
	{
		Point2f p1(line[0], line[1]);
		Point2f p2(line[2], line[3]);

		Point2f dir = p2-p1;
		float angl = atan2(dir.y,dir.x)*180/CV_PI;

		if (angl < 0) angles.push_back(angl+180);
		else angles.push_back(angl);
	});


	// 각도와 거리에 따라서 그룹을 나눈다.
	vector<vector<Vec4i>> group(1);
	vector<float> angle2(1);
	vector<float> dist(1);
	group[0].push_back(lines[0]);
	angle2[0] = angles[0];
	dist[0] = pt2linDist(Point2f(), pt2line(lines[0]));


	for (int i = 1; i<n; ++i)
	{
		// 모든 그룹과 비교한다.
		for (int j = 0; j<group.size(); ++j)
		{
			float angle_diff = abs(angles[i]-angle2[j]);
			if (angle_diff > 90) angle_diff = 180-angle_diff;
			double temd = pt2linDist(Point2f(), pt2line(lines[i]));
			float dist_diff = abs(temd - dist[j]);

			// 각도 차이가 anglethr 미만이고 거리 차가 distthr1 미만이면 그룹에 포함시키고 다음 선분을 비교한다.
			// 그룹에 해당하는 각도를 업데이트 한다.
			if (angle_diff < anglethr && dist_diff<distthr1)
			{					
				group[j].push_back(lines[i]);
				angle2[j] = (angle2[j]*(group[j].size()-1)+angles[i])/float(group[j].size());
				dist[j] = (dist[j]*(group[j].size()-1)+temd)/float(group[j].size());
				break;
			}
			// 맞는 그룹이 없으면 그룹을 새로 생성한다.
			else if(j==group.size()-1)
			{
				vector<Vec4i> tem(1);
				tem[0] = lines[i];
				group.push_back(tem);
				angle2.push_back(angles[i]);
				dist.push_back(temd);
				break;
			}				
		}		
	}


	if (debug)
	{
		img = Scalar();//--> for debug
		for (int i = 0; i<group.size();++i)
		{
			drawLines(img, group[i], colors[i], 2);//--> for debug
		}
		imshow("b", img);//--> for debug
	 	waitKey(0);//--> for debug
	}

	// 각 그룹을 평행한 두 개의 선분으로 나타낸다.
	vector<vector<Vec4f>> new_group;
	new_group.reserve(group.size());
	for (int i = 0; i<group.size(); ++i)
	{
		n = group[i].size();
		if (n>1)
		{
			// 평행한 두 쌍으로 나눌 준비를 한다.
			vector<Vec4f> group1;
			vector<Vec4f> group2;
			group1.reserve(n);
			group2.reserve(n);

			group1.push_back(group[i][0]);

			for_each(group[i].begin()+1, group[i].end(), [&group1, &group2, distthr2](Vec4i& line)
			{
				Point2f p1(line[0], line[1]);
				Point2f p2(line[2], line[3]);
				Point2f p3(group1[0][0], group1[0][1]);
				Point2f p4(group1[0][2], group1[0][3]);

				Point2f center = (p1+p2)*0.5;
				Point2f dir = p4-p3;

				float distnc = abs(dir.y*(center.x-p3.x)-dir.x*(center.y-p3.y))/norm(dir);

				if (distnc<distthr2) group1.push_back(line);
				else group2.push_back(line);
			});
			if (group1.size()>0 && group2.size()>0)
			{ 
				vector<Vec4f> tem(2);
				tem[0] = mergeLine(group1);
				tem[1] = mergeLine(group2);
				new_group.push_back(tem);
			}
		}
	}

	if (debug)
	{
		img = Scalar();//--> for debug
		for (int i = 0; i<new_group.size();++i)//--> for debug
		{
			drawLines(img, new_group[i], colors[i], 2);//--> for debug
		}
		imshow("c", img);//--> for debug
	 	waitKey(0);//--> for debug
	}

	// 두 개의 선분을 합쳐서 하나의 중심 선분을 만들어 낸다.
 	n = new_group.size();
	vector<Vec4f> newline;
	newline.reserve(n);
	for (int i = 0; i<n; ++i)
	{
		newline.push_back(getMiddleLine(new_group[i].data()));
	}

	if (debug)
	{
		img = Scalar();
		drawLines(img, newline, colors[3], 2);//--> for debug	
		imshow("d", img);
		waitKey(0);
		destroyAllWindows();
	}

	return newline;	
}

std::vector<cv::Vec4f> ClubTracker::lineGroupping2( const std::vector<cv::Vec4i>& lines, const float anglethr, const float distthr1, const float distthr2, bool debug /*= false */ )
{
	Mat img;
	if (debug)
	{
		img = Mat(700,700,CV_8UC3, Scalar());//--> for debug
		drawLines(img, lines, colors[0], 2);//--> for debug
		imshow("a", img);//--> for debug
		waitKey(0);//--> for debug
	}

	// 먼저 각도와 길이를 모두 구한다. (degree)
	int n = lines.size();	
	vector<float> angles;
	vector<float> lengths;
	angles.reserve(n);
	lengths.reserve(n);
	for_each(lines.begin(), lines.end(),[&angles, &lengths](const Vec4i& line)
	{
		Point2f p1(line[0], line[1]);
		Point2f p2(line[2], line[3]);

		Point2f dir = p2-p1;
		float angl = atan2(dir.y,dir.x)*rad2degree;

		if (angl < 0) angles.push_back(angl+180);
		else angles.push_back(angl);

		lengths.push_back(norm(dir));
	});
	
	// 각도에 따라서 그룹을 나눈다.	
	vector<vector<Vec4i>> group1(1);
	group1[0].push_back(lines[0]);

	vector<vector<pair<float, float>>> inform1(1);	//(angle, length)
	inform1[0].push_back(pair<float, float>(angles[0], lengths[0]));

	vector<float> up1(1, angles[0]*lengths[0]);
	vector<float> down1(1, lengths[0]);
	vector<float> angle1(1, angles[0]);

	for (int i = 1; i<n; ++i)
	{
		// 모든 그룹과 비교한다.
		for (int j = 0; j<group1.size(); ++j)
		{
			float angle_diff = abs(angles[i]-angle1[j]);
			if (angle_diff > 90)
			{
				angle_diff = 180-angle_diff;
				angle1[j] > angles[i] ? angles[i] += 180 : angles[i] -= 180;
			}

			// 각도 차이가 anglethr 미만이면 그룹에 포함시키고 다음 선분을 비교한다.
			// 그룹에 해당하는 각도를 업데이트 한다.
			if (angle_diff < anglethr)
			{					
				group1[j].push_back(lines[i]);
 				inform1[j].push_back(pair<float, float>(angles[i], lengths[i]));
				up1[j] += lengths[i]*angles[i];
				down1[j] += lengths[i];				
				angle1[j] = up1[j]/down1[j];
				break;
			}
			// 맞는 그룹이 없으면 그룹을 새로 생성한다.
			else if(j==group1.size()-1)
			{				
				group1.push_back(vector<Vec4i>(1, lines[i]));
				inform1.push_back(vector<pair<float, float>>(1, pair<float, float>(angles[i], lengths[i])));
				angle1.push_back(angles[i]);
				up1.push_back(lengths[i]*angles[i]);
				down1.push_back(lengths[i]);
				break;
			}				
		}		
	}

	if (debug)
	{
		img = Scalar();//--> for debug
		for (int i = 0; i<group1.size(); ++i)
		{			
			drawLines(img, group1[i], colors[i], 2);			
		}
		imshow("b", img);//--> for debug
		waitKey(0);//--> for debug
	}

	// 각 그룹의 기울기를 정렬한다.
	vector<vector<Vec4f>> group2;
	vector<vector<pair<float, float>>> inform2;
	group2.reserve(group1.size());
	inform2.reserve(inform1.size());
	vector<vector<float>> dist2;
	dist2.reserve(group1.size());

	for (int i = 0; i<group1.size(); ++i)
	{		
		if(group1[i].size()>1)
		{
			vector<float> temdist(group1[i].size());
			vector<Vec4f> tem(group1[i].size());
			vector<pair<float, float>> teminform(inform1[i].size());
			for (int j = 0; j<group1[i].size(); ++j)
			{				
				float dangle = (angle1[i] - inform1[i][j].first);
				tem[j] = rotLineSegment(group1[i][j], -dangle, Point2f((group1[i][j][0]+group1[i][j][2])*0.5, (group1[i][j][1]+group1[i][j][3])*0.5));
				teminform[j].first = angle1[i];
				teminform[j].second = inform1[i][j].second;
				temdist[j] = pt2linDist(Point2f(), pt2line(tem[j]));
			}
			group2.push_back(tem);
			inform2.push_back(teminform);
			dist2.push_back(temdist);
		}		
	}
	if (debug)
	{
		img = Scalar();//--> for debug
		for (int i = 0; i<group2.size(); ++i)
		{			
			// check angle
			vector<float> temangle;
			for_each(group2[i].begin(), group2[i].end(),[&temangle](const Vec4i& line)
			{
				Point2f p1(line[0], line[1]);
				Point2f p2(line[2], line[3]);
				Point2f dir = p2-p1;

				float angl = atan2(dir.y,dir.x)*rad2degree;

				if (angl < 0) temangle.push_back(angl+180);
				else temangle.push_back(angl);				
			});

			drawLines(img, group2[i], colors[i], 2);
		}
		imshow("c", img);//--> for debug
		waitKey(0);//--> for debug
	}

	// check angle
	for (int i = 0; i<group2.size(); ++i)
	{			
		vector<float> temangle;
		for_each(group2[i].begin(), group2[i].end(),[&temangle](const Vec4i& line)
		{
			Point2f p1(line[0], line[1]);
			Point2f p2(line[2], line[3]);
			Point2f dir = p2-p1;

			float angl = atan2(dir.y,dir.x)*rad2degree;

			if (angl < 0) temangle.push_back(angl+180);
			else temangle.push_back(angl);				
		});
	}


	if (group2.size()>0)
	{
		return group2[0];
	}
	//////////////////////////////////////////////////////////////////////////
	vector<Vec4f> newline2(1);
	return newline2;
	//////////////////////////////////////////////////////////////////////////



	for (int i = 0; i<group2.size(); ++i)
	{
		// 거리 차이에 따라 다시 그룹을 나눈다.
		vector<vector<Vec4f>> group3(1);
		group3[0].push_back(group2[i][0]);
		vector<vector<pair<float, float>>> inform3(1);
		inform3[0].push_back(inform2[i][0]);

		vector<float> up3(1, dist2[i][0]*inform2[i][0].second);
		vector<float> down3(1, inform2[i][0].second);
		vector<double> dist3(1, dist2[i][0]);		

		for (int j = 1; j<group2[i].size(); ++j)
		{
			// 모든 temgroup와 비교한다.
			for (int k = 0; k<group3.size(); ++k)
			{
				float dist_diff = abs(dist2[i][j] - dist3[k]);

				// threshold값 이하이면 해당 그룹에 포함시키고 업데이트.
				if (dist_diff < distthr2)
				{					
					group3[k].push_back(group2[i][j]);
					inform3[k].push_back(inform3[i][j]);
					up3[k] += dist2[i][j]*inform2[i][j].second;
					down3[k] += inform2[i][j].second;
					dist3[k] = up3[k]/down3[k];
					break;
				}
				// 맞는 그룹이 없으면 그룹을 새로 생성한다.
				else if(k==group1.size()-1)
				{					
					group3.push_back(vector<Vec4f>(1, group2[i][j]));
					inform3.push_back(vector<pair<float, float>>(1, inform3[i][j]));
					up3.push_back(dist2[i][j]*inform2[i][j].second);
					down3.push_back(inform2[i][j].second);
					dist3.push_back(dist2[i][j]);
					break;
				}
			}
		}

		// 거리차이에 따라 나눈 그룹을 하나의 직선으로 나타낸다.
		float tem_threshold = 30;
		vector<Vec4f> temgroup(group3.size());

		for (int j = 0; j<group3.size(); ++j)
		{
			if (group3[j].size()==1)
			{
				temgroup[j] = group3[j][0];
			}
			else
			{
				// 직선을 하나로 합친다.
				vector<Vec4f> temline;
				temline.reserve(group3[j].size());
				for (int k = 0; k<group3[j].size(); ++k)
				{
					int count = 0;
					for (int l = 0; l<group3[j].size(); ++l)
					{
						if (k!=l)
						{
							if (calLineSegDist(group3[j][k], group3[j][l]) < tem_threshold) count++;
						}
					}
					if (count>0)
					{
						temline.push_back(group3[j][k]);
					}
				}
				temgroup[j] = mergeLine(temline);
			}
		}

		// 두개의 평행한 직선으로 나눈다.
	} 

// 
// 	// 각 그룹을 평행한 두 개의 선분으로 나타낸다.
// 	vector<vector<Vec4f>> new_group;
// 	new_group.reserve(group.size());
// 	for (int i = 0; i<group.size(); ++i)
// 	{
// 		n = group[i].size();
// 		if (n>1)
// 		{
// 			// 평행한 두 쌍으로 나눌 준비를 한다.
// 			vector<Vec4f> group1;
// 			vector<Vec4f> group2;
// 			group1.reserve(n);
// 			group2.reserve(n);
// 
// 			group1.push_back(group[i][0]);
// 
// 			for_each(group[i].begin()+1, group[i].end(), [&group1, &group2, distthr2](Vec4i& line)
// 			{
// 				Point2f p1(line[0], line[1]);
// 				Point2f p2(line[2], line[3]);
// 				Point2f p3(group1[0][0], group1[0][1]);
// 				Point2f p4(group1[0][2], group1[0][3]);
// 
// 				Point2f center = (p1+p2)*0.5;
// 				Point2f dir = p4-p3;
// 
// 
// 				float distnc = abs(dir.y*(center.x-p3.x)-dir.x*(center.y-p3.y))/norm(dir);
// 
// 				if (distnc<distthr2) group1.push_back(line);
// 				else group2.push_back(line);
// 			});
// 			if (group1.size()>0 && group2.size()>0)
// 			{ 
// 				vector<Vec4f> tem(2);
// 				tem[0] = mergeLine(group1);
// 				tem[1] = mergeLine(group2);
// 				new_group.push_back(tem);
// 			}
// 		}
// 	}
// 
// 	if (debug)
// 	{
// 		img = Scalar();//--> for debug
// 		for (int i = 0; i<new_group.size();++i)//--> for debug
// 		{
// 			drawLines(img, new_group[i], colors[i], 2);//--> for debug
// 		}
// 		imshow("c", img);//--> for debug
// 		waitKey(0);//--> for debug
// 	}
// 
// 	// 두 개의 선분을 합쳐서 하나의 중심 선분을 만들어 낸다.
// 	n = new_group.size();
	vector<Vec4f> newline(1);
// 	newline.reserve(n);
// 	for (int i = 0; i<n; ++i)
// 	{
// 		newline.push_back(getMiddleLine(new_group[i].data()));
// 	}
// 
// 	if (debug)
// 	{
// 		img = Scalar();
// 		drawLines(img, newline, colors[3], 2);//--> for debug	
// 		imshow("d", img);
// 		waitKey(0);
// 		destroyAllWindows();
// 	}

	return newline;
}

std::vector<LineSegment2Df> ClubTracker::MeargingSegment(const std::vector<LineSegment2Df>& input)
{
	double thr = 10;
	int n = input.size();
	std::vector<LineSegment2Df> output;
	output.reserve(n);
	std::vector<bool> check(n, false);

	for (int i = 0; i<n; ++i)
	{
		if (!check[i])
		for (int j = i+1; j<n; ++j)
		{
			if (!check[j])
			{
				Point2f d1 = input[j].P1()-input[i].P1();
				Point2f d2 = input[j].P2()-input[i].P1();
				double n1 = norm(d1);
				double n2 = norm(d2);
				vector<pair<double, Point2f>> v(4);
				v[0] = pair<double, Point2f>(0, input[i].P1());
				v[1] = pair<double, Point2f>(input[i].length, input[i].P2());
				v[2] = pair<double, Point2f>((input[i].dir.dot(d1)>0) ? n1 : -n1, input[j].P1());
				v[3] = pair<double, Point2f>((input[i].dir.dot(d2)>0) ? n2 : -n2, input[j].P2());

				if ((0<v[2].first && v[2].first<v[1].first) || (0<v[3].first && v[3].first<v[1].first))
				{
					if ((0<v[2].first && v[2].first<v[1].first) && (0<v[3].first && v[3].first<v[1].first))
					{
						output.push_back(input[i]); check[i] = check[j] = true;	break;
					}
					else if (0<v[2].first && v[2].first<v[1].first)
					{
						v[3].first < 0 ? output.push_back(LineSegment2Df(v[3].second, v[1].second)) : output.push_back(LineSegment2Df(v[0].second, v[3].second));
						check[i] = check[j] = true;	break;
					}
					else
					{
						v[2].first < 0 ? output.push_back(LineSegment2Df(v[2].second, v[1].second)) : output.push_back(LineSegment2Df(v[0].second, v[2].second));
						check[i] = check[j] = true;	break;
					}
				}
				else if (v[2].first<0 && v[3].first<0)
				{
					if (v[2].first < v[3].first)
					{if (v[3].first > -thr) output.push_back(LineSegment2Df(v[2].second, v[1].second)); check[i] = check[j] = true; break;}
					else
					{if (v[2].first > -thr) output.push_back(LineSegment2Df(v[3].second, v[1].second)); check[i] = check[j] = true; break;}
				}
				else if (v[2].first>v[1].first && v[3].first>v[1].first)
				{
					if (v[2].first < v[3].first)
					{if (v[2].first-v[1].first < thr) output.push_back(LineSegment2Df(v[0].second, v[3].second)); check[i] = check[j] = true; break;}
					else
					{if (v[3].first-v[1].first < thr) output.push_back(LineSegment2Df(v[0].second, v[2].second)); check[i] = check[j] = true; break;}
				}
				else/* if ((v[2].first<0 && v[3].first>v[1].first) || (v[2].first>v[1].first && v[3].first<0) )*/
				{
					output.push_back(input[j]); check[i] = check[j] = true;	break;
				}
				
			}
			if (j==n-1)
			{
				output.push_back(input[i]);
				break;
			}
		}
		if (i==n-1)
		{
			output.push_back(input[i]);
			break;
		}
	}

	if (output.size()==input.size())
	{
		return output;
	}
	else return MeargingSegment(output);
}

LineSegment2Df ClubTracker::PairingSegment( const std::vector<LineSegment2Df>& input )
{






	return LineSegment2Df();
}

std::vector<cv::Vec4f> ClubTracker::lineGroupping3( const std::vector<cv::Vec4i>& lines, const float anglethr, const float distthr1, const float distthr2, bool debug /*= false */ )
{
	vector<Vec4f> return_line_segments;
	return_line_segments.reserve(5);
	if (lines.empty())
	{
		return return_line_segments;
	}
	Mat img;
	if (debug)
	{
		img = Mat(700,700,CV_8UC3, Scalar());//--> for debug
		drawLines(img, lines, colors[0], 2);//--> for debug
		imshow("a", img);//--> for debug
		waitKey(0);//--> for debug
	}
	
	int n = lines.size();	
	vector<LineSegment2Df> lsegments(lines.size());
	for (int i = 0; i<n; ++i)
	{
		lsegments[i] = LineSegment2Df(lines[i]);
	}
	sort(lsegments.begin(), lsegments.end(), [](const LineSegment2Df& a, const LineSegment2Df& b){return a.length>b.length;});

	// 각도에 따라서 그룹을 나눈다.	
	vector<vector<LineSegment2Df>> group1(1);
	group1[0].push_back(lsegments[0]);	
	vector<double> up1(1, lsegments[0].theta*lsegments[0].length);
	vector<double> down1(1, lsegments[0].length);
	vector<double> angle1(1, lsegments[0].theta);

	for (int i = 1; i<n; ++i)
	{
		// 모든 그룹과 비교한다.
		for (int j = 0; j<group1.size(); ++j)
		{
			float angle_diff = abs(lsegments[i].theta-angle1[j]);
			if (angle_diff > 90)
			{
				lsegments[i].Reverse();
 				angle_diff = 180-angle_diff;
			}

			// 각도 차이가 anglethr 미만이면 그룹에 포함시키고 다음 선분을 비교한다.
			// 그룹에 해당하는 각도를 업데이트 한다.
 			if (angle_diff < anglethr)
			{					
				group1[j].push_back(lsegments[i]);				
				up1[j] += lsegments[i].length*lsegments[i].theta;
				down1[j] += lsegments[i].length;
				angle1[j] = up1[j]/down1[j];
				break;
			}
			// 맞는 그룹이 없으면 그룹을 새로 생성한다.
			else if(j==group1.size()-1)
			{				
				group1.push_back(vector<LineSegment2Df>(1, lsegments[i]));
				up1.push_back(lsegments[i].length*lsegments[i].theta);
				down1.push_back(lsegments[i].length);
				angle1.push_back(lsegments[i].theta);
				break;
			}				
		}		
	}

	if (debug)
	{
		img = Scalar();//--> for debug
		for (int i = 0; i<group1.size(); ++i)
		{
			for (int j = 0; j<group1[i].size(); ++j)
			{
				drawLineSegment(img, group1[i][j].line, colors[i], 2);
			}			
		}
		imshow("b", img);//--> for debug
		waitKey(0);//--> for debug
	}

	vector<vector<LineSegment2Df>>::iterator it_lvv = group1.begin();
	vector<LineSegment2Df>::iterator it_lv;
	vector<double>::iterator it_dv = angle1.begin();

	// 각 그룹의 기울기를 정렬한다.
	for (; it_lvv!=group1.end(); ++it_lvv, ++it_dv)
	{
 		for_each((*it_lvv).begin(), (*it_lvv).end(), [it_dv](LineSegment2Df& line)
 		{
			line.Rotate(*it_dv-line.theta); 	
 		});
	}
	if (debug)
	{
		img = Scalar();//--> for debug
		for (int i = 0; i<group1.size(); ++i)
		{
			for (int j = 0; j<group1[i].size(); ++j)
			{
				drawLineSegment(img, group1[i][j].line, colors[i], 2);
			}			
		}
		imshow("c", img);//--> for debug
		waitKey(0);//--> for debug
		destroyAllWindows();
	}


// #pragma omp parallel for
	for (int i = 0; i<group1.size(); ++i)
	{
		// 거리 차이에 따라 다시 그룹을 나눈다.
		vector<vector<LineSegment2Df>> group2(1);
		group2[0].push_back(group1[i][0]);

		vector<double> up2(1, group1[i][0].rho*group1[i][0].length);
		vector<double> down2(1, group1[i][0].length);
		vector<double> rho2(1, group1[i][0].rho);		
		
		for (int j = 1; j<group1[i].size(); ++j)
		{
			for (int k = 0; k<group2.size(); ++k)
			{
				float dist_diff = abs(group1[i][j].rho - rho2[k]);
 
 				// threshold값 이하이면 해당 그룹에 포함시키고 업데이트.
 				if (dist_diff < distthr2)
 				{					
 					group2[k].push_back(group1[i][j]);
 					up2[k] += group1[i][j].rho*group1[i][j].length;
 					down2[k] += group1[i][j].length;
 					rho2[k] = up2[k]/down2[k];
 					break;
				}
				// 맞는 그룹이 없으면 그룹을 새로 생성한다.
				else if(k==group2.size()-1)
 				{					
					group2.push_back(vector<LineSegment2Df>(1, group1[i][j]));					
					up2.push_back(group1[i][j].rho*group1[i][j].length);
					down2.push_back(group1[i][j].length);
					rho2.push_back(group1[i][j].rho);
					break;
				}
			}
		}

		if (group2.size()<2)
		{
			break;
		}

		// 거리에 따라서 정렬
		for (it_lvv = group2.begin(), it_dv = rho2.begin(); it_lvv!=group2.end(); ++it_lvv, ++it_dv)
		{
			for_each((*it_lvv).begin(), (*it_lvv).end(), [&it_dv, &return_line_segments](LineSegment2Df& l)
			{
				Point2f Tvec(l.implicit_param[0], l.implicit_param[1]);
				float Tmag = l.implicit_param[2]<0 ? (*it_dv)-l.rho : -(*it_dv)+l.rho;
				l.Translate(Tmag*Tvec);
// 				return_line_segments.push_back(l.line);
			});
		}
		
		vector<LineSegment2Df> group3;
		group3.reserve(group2.size());

		// 각 그룹을 한 직선으로 합친다.
		for_each(group2.begin(), group2.end(), [&](vector<LineSegment2Df>& lv)
		{
			lv = MeargingSegment(lv);
			if (lv.size()==1)
			{
				group3.push_back(lv[0]);
			}
			else
			{
				LineSegment2Df tem = *(max_element(lv.begin(), lv.end(), [](LineSegment2Df a, LineSegment2Df b){return a.length < b.length;}));
				group3.push_back(tem);
			}
		});

		return_line_segments.push_back(PairingSegment(group3).line);
	}
	
	
	return return_line_segments;
	
// 	//////////////////////////////////////////////////////////////////////////
// 	vector<Vec4f> newline2(1);
// 	return newline2;
// 	//////////////////////////////////////////////////////////////////////////

}

cv::Vec4f ClubTracker::getMiddleLine( const cv::Vec4f* lines )
{
	Point2f p1(lines[0][0], lines[0][1]);
	Point2f p2(lines[0][2], lines[0][3]);
	Point2f p3(lines[1][0], lines[1][1]);
	Point2f p4(lines[1][2], lines[1][3]);

	Vec3f ln;
	Point2f pp1, pp2;
	if (norm(p2-p1)>norm(p4-p3))
	{
		ln = pt2line(lines[1]);
		pp1 = p1;
		pp2 = p2;
	}
	else
	{
		ln = pt2line(lines[0]);
		pp1 = p3;
		pp2 = p4;
	}

	Point2f dir(ln[0], ln[1]);
	float d1 = 0.5*(dir.ddot(pp1)+ln[2]);
	float d2 = 0.5*(dir.ddot(pp2)+ln[2]);

	Point2f np1 = pp1-d1*dir;
	Point2f np2 = pp2-d2*dir;

	return Vec4f(np1.x, np1.y, np2.x, np2.y);
}

void ClubTracker::modelAdjust( cv::Mat& model, const cv::Vec4f& l1, const cv::Vec4f& l2 )
{
	vector<Point2f> p = model.clone();
	int n = p.size();
	Point2f pt, dir;

	p[0] = Point2f(l1[2], l1[3]);
	p[n-1] = Point2f(l2[2], l2[3]);

// 	pt = GetCrossPoint(pt2line(l1), pt2line(p[0], p[1]));
// 	dir = p[0] - p[1];
// 	Normalize(dir);
// 	p[0] = pt + 5*dir;
// 
// 	pt = GetCrossPoint(pt2line(l2), pt2line(p[n-1], p[n-2]));
// 	dir = p[n-1] - p[n-2];
// 	Normalize(dir);
// 	p[n-1] = pt + 5*dir;

	Mat(p).copyTo(model);
}
void ClubTracker::getIntersection( const cv::Mat& model, const std::vector<cv::Vec4f>& lines, const std::vector<float>& para, cv::Mat& data )
{
	vector<double> cross_para;

	for (int i = 0; i<lines.size(); ++i)
	{
		vector<Point2f> rslt_pts;	
		Vec4f tem = lines[i];

		Point2f grip(tem[0], tem[1]);
		Point2f head(tem[2], tem[3]);
		Point2f dir = head-grip;
		Normalize(dir);
		head  = head + 1000*(dir);
		tem[2] = head.x;
		tem[3] = head.y;

		Bezier_intersection2d(model, tem, rslt_pts);


// 		if (!r_para.empty())
// 		{
// 			Go::Point rpt;
// 
// 			if (r_para.size()==1 || cross_para.empty())
// 			{
// 				cross_para.push_back(r_para[0].first);
// 				lin.point(rpt, r_para[0].second);
// 				lines[i][2] = rpt[0];
// 				lines[i][3] = rpt[1];
// 			}
// 			else
// 			{
// 				for (unsigned j = 0; j<r_para.size(); ++j)
// 				{
// 					if (cross_para[i-1] < r_para[j].first)
// 					{
// 						cross_para.push_back(r_para[j].first);
// 						lin.point(rpt, r_para[j].second);
// 						lines[i][2] = rpt[0];
// 						lines[i][3] = rpt[1];
// 						break;
// 					}
// 				}
// 			}
// 		}
// 		else
// 		{
// 			cross_para.push_back(paraval[i]);
// 		}

	}

}

void ClubTracker::getIntersection2( const cv::Mat& model, const std::vector<cv::Vec4f>& lines, cv::Mat& data )
{
	for (int i = 0; i<lines.size(); ++i)
	{
		vector<Point2f> rslt_pts;	
		Vec4f tem = lines[i];
		Point2f pa(tem[0], tem[1]);
		Point2f pb(tem[2], tem[3]);
		pa  = pa + 0.5*(pa-pb);
		tem[0] = pa.x;
		tem[1] = pa.y;

		Bezier_intersection2d(model, tem, rslt_pts);

		if (!rslt_pts.empty())
		{
			Point2f tem;
			if(i>0) tem = Point2f(lines[i-1][0], lines[i-1][1]);
			else tem = Point2f(lines[i][0], lines[i][1]);

			float val = 1000;
			int indx = 0;
			for (int j = 0; j<rslt_pts.size(); ++j)
			{
				float d = norm(tem-rslt_pts[j]);
				if (d<val)
				{
					val = d;
					indx = j;
				}
			}
			Mat(rslt_pts).row(indx).copyTo(data.row(i));
		}
	}
}
void ClubTracker::clublength_interp2d( std::vector<cv::Vec4f>& lines )
{
	vector<float> dist(lines.size());
	for (int i = 0; i<dist.size(); ++i)
	{
		Point2f pa(lines[i][0], lines[i][1]);
		Point2f pb(lines[i][2], lines[i][3]);
		dist[i] = norm(pa-pb);
	}

	Mat data = Mat(dist);	
	bezier_val(bezier_model(data, chord_legth(data), 2), chord_legth(data)).copyTo(data);	

	for (int i = 0; i<dist.size(); ++i)
	{
		Point2f pa(lines[i][0], lines[i][1]);
		Point2f pb(lines[i][2], lines[i][3]);
		Point2f dir = pa-pb;
		Normalize(dir);
		pa = pb+dist[i]*dir;
		lines[i][0] = pa.x;
		lines[i][1] = pa.y;
	}
}

// 2d data post processing
void ClubTracker::sorting_LinePoints()
{
	isEmpty_indx.clear();
	isEmpty_indx.resize(num_frame);
	nonEmpty_indx.clear();
	nonEmpty_indx.reserve(num_frame);
	main_lines1.clear();
	main_lines1.reserve(num_frame);
	sub_lines1.clear();
	sub_lines1.reserve(num_frame);

	Vec4f mlin_cur, mlin_pre, slin_cur, slin_pre;
	Point2f dir_pre, dir;
	int count = 0;
	for (int i = 0; i<num_frame; ++i)
	{
		if (!main_lineVec[i].empty() && !sub_lineVec[i].empty())
		{
			mlin_cur = main_lineVec[i][main_indx[i]];
			slin_cur = sub_lineVec[i][sub_indx[i]];

			if (count<3)
			{
				if (mlin_cur[1]>mlin_cur[3]) mlin_cur = Vec4f(mlin_cur[2], mlin_cur[3], mlin_cur[0], mlin_cur[1]);
				if (slin_cur[1]>slin_cur[3]) slin_cur = Vec4f(slin_cur[2], slin_cur[3], slin_cur[0], slin_cur[1]);
			}
			else
			{
				dir_pre = Point2f(mlin_pre[2]-mlin_pre[0], mlin_pre[3]-mlin_pre[1]);
				dir = Point2f(mlin_cur[2]-mlin_cur[0], mlin_cur[3]-mlin_cur[1]);
				if (dir.ddot(dir_pre)<0) mlin_cur = Vec4f(mlin_cur[2], mlin_cur[3], mlin_cur[0], mlin_cur[1]);

				dir_pre = Point2f(slin_pre[2]-slin_pre[0], slin_pre[3]-slin_pre[1]);
				dir = Point2f(slin_cur[2]-slin_cur[0], slin_cur[3]-slin_cur[1]);
				if (dir.ddot(dir_pre)<0) slin_cur = Vec4f(slin_cur[2], slin_cur[3], slin_cur[0], slin_cur[1]);
			}

			mlin_pre = mlin_cur;
			slin_pre = slin_cur;

			++count;
			nonEmpty_indx.push_back(i);
			isEmpty_indx[i] = 1;

			main_lines1.push_back(mlin_cur);
			sub_lines1.push_back(slin_cur);
		}
	}
}
void ClubTracker::determineUpDown()
{
	Mat dir = Mat(main_lines1).reshape(1).colRange(2,4) - Mat(main_lines1).reshape(1).colRange(0,2);
	Mat dir2 = Mat(sub_lines1).reshape(1).colRange(2,4) - Mat(sub_lines1).reshape(1).colRange(0,2);

	Mat tem;
	phase(dir.col(0), dir.col(1), tem, true);
	vector<float> angle = tem;
	phase(dir2.col(0), dir2.col(1), tem, true);
	vector<float> angle2 = tem;
	
	for (int i = 1; i<angle.size(); ++i)
	{
		float d1 = angle[i] - angle[i-1];
		if (d1<-270) d1 += 360;
		else if (d1>270) d1 -= 360;

		angle[i] = angle[i-1] + d1;

		float d2 = angle2[i] - angle2[i-1];
		if (d2<-270) d2 += 360;
		else if (d2>270) d2 -= 360;

		angle2[i] = angle2[i-1] + d2;
	}
	saveData(angle, "angle1.txt");
	saveData(angle2, "angle2.txt");

	up_end = max_element(angle.begin(), angle.end())-angle.begin();

	int interval = 3;
		
 	down_start = up_end+interval;
 	up_end -= interval;

	double pick_angle = 330;
	double start_angle = 90;
	double end_angle = -270;

// 	for (int i = up_end; i>-1; --i)	///////////////////////////////////////////////////
// 	{		
// 		if (angle[i] < pick_angle)
// 		{			
// 			up_end= i+1;
// 			break;
// 		}		
// 	}

	up_start = 0;
	for (int i = up_end-10; i>=0; --i)
	{		
		if (angle[i] < start_angle)
		{
			up_start = i+3;
			break;
		}
		else if (angle[i]-angle[i+1]>0 || angle2[i]-angle2[i+1]>0)
		{
			up_start = i+3;
			break;
		}
		else
		{
			up_start = i+2;
		}
	}	

// 	for (int i = down_start; i<angle.size(); ++i)	/////////////////////////////////
// 	{		
// 		if (angle[i]<pick_angle)
// 		{
// 			down_start = i;
// 			break;
// 		}
// 	}


	for (int i = down_start+10; i<angle.size(); ++i)
	{
		down_end = i-3;
		if (angle[i]<end_angle)
		{
			break;
		}
		else if (angle[i]-angle[i-1]>0 || angle2[i]-angle2[i-1]>0)
		{
			break;
		}
	}
	up_para.clear();
	down_para.clear();
	up_para.reserve(up_end-up_start+1);
	down_para.reserve(down_end-down_start+1);
	for (int i = up_start; i<=up_end; ++i)
	{
		up_para.push_back(abs((angle[up_start]-angle[i])/(angle[up_end]-angle[up_start])));
	}
	for (int i = down_start; i<=down_end; ++i)
	{
		down_para.push_back(abs((angle[down_start]-angle[i])/(angle[down_end]-angle[down_start])));
	}

	vector<int> tem_indx(num_frame);
	upSwing_indx.clear();
	downSwing_indx.clear();
	upSwing_indx.reserve(up_end-up_start+1);
	downSwing_indx.reserve(down_end-down_start+1);
	for (int i = up_start; i<=up_end; ++i)
	{
		tem_indx[nonEmpty_indx[i]] = 1;
		upSwing_indx.push_back(nonEmpty_indx[i]);
	}
	for (int i = down_start; i<=down_end; ++i)
	{
		tem_indx[nonEmpty_indx[i]] = 2;
		downSwing_indx.push_back(nonEmpty_indx[i]);
	}
	isEmpty_indx = tem_indx;
	num_upswing = upSwing_indx.size();
	num_downswing = downSwing_indx.size();

	main_upswing[0] = Mat(main_lines1).rowRange(up_start, up_end+1).clone();
	main_downswing[0] = Mat(main_lines1).rowRange(down_start, down_end+1).clone();
	sub_upswing[0] = Mat(sub_lines1).rowRange(up_start, up_end+1).clone();
	sub_downswing[0] = Mat(sub_lines1).rowRange(down_start, down_end+1).clone();
}
cv::Mat ClubTracker::pt2_interpolation( std::vector<cv::Vec4f>& lines, const int n )
{
	Mat data;
	Mat model;
	vector<float> para = chord_legth(data);
	data = Mat(lines).reshape(1).colRange(2,4).reshape(2);
	model = bezier_model(data, para, n);
	modelAdjust(model, lines[0], lines[lines.size()-1]);
	getIntersection(model, lines, para, data);

	return model;
}

#ifdef USING_SINTEF
void ClubTracker::pt2_interpolation( std::vector<cv::Vec4f>& lines, const std::vector<float>& para, const int n )
{
	if (lines.size() < n+1) return;
	Mat data;
	Mat model;

	data = Mat(lines).reshape(1).colRange(2,4).reshape(2);
	model = bezier_model(data, para, n);

// 	Mat tem(700,700, CV_8UC3, Scalar());
// 	drawLines(tem, lines, Scalar(0,255,0));
// 
// 	for (int i = 1; i<model.rows; ++i)
// 	{
// 		line(tem, Point(model.at<float>(i-1,0), model.at<float>(i-1,1)), Point(model.at<float>(i,0), model.at<float>(i,1)), Scalar(255,0,0));
// 	}
// 
	modelAdjust(model, lines[0], lines[lines.size()-1]);
// 
// 	for (int i = 1; i<model.rows; ++i)
// 	{
// 		line(tem, Point(model.at<float>(i-1,0), model.at<float>(i-1,1)), Point(model.at<float>(i,0), model.at<float>(i,1)), Scalar(0,0,255));
// 	}
// 
// 	imshow("tem", tem);
// 	waitKey(0);
// 	destroyAllWindows();

	vector<double> coef(model.rows*3, 0);
	vector<double> knot(model.rows*2, 1);
	for (int i = 0; i<model.rows; ++i)
	{
		for (int j = 0; j<2; ++j)
		{
			coef[i*3+j] = model.at<float>(i,j);
		}
		knot[i] = 0;
	}
	Go::SplineCurve curve_model(model.rows, n+1, knot.begin(), coef.begin(), 3);


	// get cross point
	vector<double> cross_para;
	for (unsigned i = 0; i<lines.size(); ++i)
	{
		Go::Point p1(lines[i][0], lines[i][1], 0);
		Go::Point p2(lines[i][2], lines[i][3], 0);
		Go::Point dir = (p2 - p1);
		dir.normalize();
		p2 += dir*1000;

		Go::SplineCurve lin(p1, p2);
		vector<pair<double, double>> r_para;		
		Go::intersectcurves(&curve_model, &lin, 0.01, r_para);
		std::sort(r_para.begin(), r_para.end(), [](const pair<double, double>& p1, const pair<double, double>& p2)
		{
			return p1.first <  p2.first;
		});
		if (!r_para.empty())
		{
			Go::Point rpt;

			if (cross_para.empty())
			{
				cross_para.push_back(r_para[0].first);
				lin.point(rpt, r_para[0].second);
				lines[i][2] = rpt[0];
				lines[i][3] = rpt[1];
			}
			else
			{
				for (unsigned j = 0; j<r_para.size(); ++j)
				{
					if (cross_para[i-1] < r_para[j].first)
					{
						cross_para.push_back(r_para[j].first);
						lin.point(rpt, r_para[j].second);
						lines[i][2] = rpt[0];
						lines[i][3] = rpt[1];
						break;
					}
				}
			}
		}
		else
		{
			cross_para.push_back(para[i]);
		}
	}


}
void ClubTracker::pt2_interpolation2( std::vector<cv::Vec4f>& lines, const std::vector<float>& para, const double thr )
{
	int num_data = lines.size();
	vector<double> pts(num_data*3);
	vector<double> paraval(num_data);
	for (unsigned i = 0; i<num_data; ++i)
	{
		pts[i*3+0] = lines[i][2];
		pts[i*3+1] = lines[i][3];
		pts[i*3+2] = 0;
		paraval[i] = para[i];
	}
	double mxdist;
	double avdist;

	// get model
	Go::SplineCurve curve_model;
	curve_model = *Go::ApproxCurve(pts, paraval, 3, thr).getApproxCurve(mxdist, avdist);

// 	vector<double> coef(curve_model.coefs_begin(), curve_model.coefs_end());
// 	for (int i = 0; i<2; ++i)
// 	{
// 		coef[i] = lines[0][i+2];
// 		coef[3*(curve_model.numCoefs()-1)+i] = lines[num_data-1][i+2];
// 	}
// 	Go::SplineCurve curve_model2(curve_model.numCoefs(), curve_model.order(), curve_model.knotsBegin(), coef.begin(),3);

	// get cross point
	vector<double> cross_para;
	for (unsigned i = 0; i<num_data; ++i)
	{
		Go::Point p1(lines[i][0], lines[i][1], 0);
		Go::Point p2(lines[i][2], lines[i][3], 0);
		Go::Point dir = (p2 - p1);
		dir.normalize();
		p2 += dir*1000;

		Go::SplineCurve lin(p1, p2);
		vector<pair<double, double>> r_para;		
		Go::intersectcurves(&curve_model, &lin, 0.01, r_para);
		std::sort(r_para.begin(), r_para.end(), [](const pair<double, double>& p1, const pair<double, double>& p2)
		{
			return p1.first <  p2.first;
		});
		if (!r_para.empty())
		{
			Go::Point rpt;

			if (cross_para.empty())
			{
				cross_para.push_back(r_para[0].first);
				lin.point(rpt, r_para[0].second);
				lines[i][2] = rpt[0];
				lines[i][3] = rpt[1];
			}
			else
			{
				for (unsigned j = 0; j<r_para.size(); ++j)
				{
					if (cross_para[i-1] < r_para[j].first)
					{
						cross_para.push_back(r_para[j].first);
						lin.point(rpt, r_para[j].second);
						lines[i][2] = rpt[0];
						lines[i][3] = rpt[1];
						break;
					}
				}
			}
		}
		else
		{
			cross_para.push_back(paraval[i]);
		}
	}
}
void ClubTracker::interpolation( std::vector<cv::Point3f>& data, const std::vector<float>& para, const double thr )
{
	int num_data = data.size();
	vector<double> pts(num_data*3);
	vector<double> paraval(num_data);
	for (unsigned i = 0; i<num_data; ++i)
	{
		pts[i*3+0] = data[i].x;
		pts[i*3+1] = data[i].y;
		pts[i*3+2] = data[i].z;
		paraval[i] = para[i];
	}
	double mxdist;
	double avdist;
	// get model
	Go::SplineCurve curve_model;
	curve_model = *Go::ApproxCurve(pts, paraval, 3, thr).getApproxCurve(mxdist, avdist);

	for (unsigned i = 0; i<num_data; ++i)
	{
		Go::Point pt;
		curve_model.point(pt, paraval[i]);

		data[i].x = pt[0];
		data[i].y = pt[1];
		data[i].z = pt[2];		
	}
}
#endif
void ClubTracker::pt1_interpolation( std::vector<cv::Vec4f>& lines, const int n )
{
	Mat data;
	Mat model;

	data = Mat(lines).reshape(1).colRange(0,2).reshape(2);
	model = bezier_model(data, chord_legth(data), n);
	modelAdjust(model, lines[0], lines[lines.size()-1]);
	getIntersection2(model, lines, data);

}
void ClubTracker::stereo2Dmatching(CalibClass& calib)
{
	stereo2Dmatching(main_upswing[3], sub_upswing[3], calib);
	stereo2Dmatching(main_downswing[3], sub_downswing[3], calib);
}
void ClubTracker::stereo2Dmatching( std::vector<cv::Vec4f>& m_data, std::vector<cv::Vec4f>& s_data, CalibClass& calib )
{
	int n = m_data.size();
	Point2f pt;
	vector<Vec3f> epGrip_m2s, epHead_m2s, epGrip_s2m, epHead_s2m;
	
	computeCorrespondEpilines(Mat(m_data).reshape(1).colRange(0,2).reshape(2).clone(), 2, calib.F, epGrip_m2s);
	computeCorrespondEpilines(Mat(m_data).reshape(1).colRange(2,4).reshape(2).clone(), 2, calib.F, epHead_m2s);
	computeCorrespondEpilines(Mat(s_data).reshape(1).colRange(0,2).reshape(2).clone(), 1, calib.F, epGrip_s2m);
	computeCorrespondEpilines(Mat(s_data).reshape(1).colRange(2,4).reshape(2).clone(), 1, calib.F, epHead_s2m);

	for (int i = 0; i<n; ++i)
	{		
		Point2f head(s_data[i][2], s_data[i][3]);
		Point2f grip(s_data[i][0], s_data[i][1]);		
		
		// head part
		pt = GetCrossPoint(epHead_m2s[i], pt2line(s_data[i]));

		Point2f dir = head - grip;
		Point2f dir2(-epHead_m2s[i][1], epHead_m2s[i][0]);
		double val = min(abs(dir.ddot(dir2)/norm(dir)/norm(dir2)), 1.0);
		double angle = acos(val);
		
		if (angle > CV_PI*0.03)
		{
			if (norm(head-grip) < norm(pt-grip))
			{
				s_data[i][2] = pt.x;
				s_data[i][3] = pt.y;
			}
			else
			{
				pt = GetCrossPoint(epHead_s2m[i], pt2line(m_data[i]));
				m_data[i][2] = pt.x;
				m_data[i][3] = pt.y;
			}
		}
	}

}

// 3d tracking function
void ClubTracker::Tracking3Dpts( CalibClass& calib, const int frame )
{
	if (isEmpty_indx[frame]==0) return;
	
	main_pt1[frame] = calib.getDir(main_pt1_2d[frame], 0);
	main_pt2[frame] = calib.getDir(main_pt2_2d[frame], 0);

	sub_pt1[frame] = calib.getDir(sub_pt1_2d[frame], 1);
	sub_pt2[frame] = calib.getDir(sub_pt2_2d[frame], 1);
	
	main_n[frame] = GetPlaneNormal(main_pt0, main_pt1[frame], main_pt2[frame]);
	sub_n[frame] = GetPlaneNormal(sub_pt0, sub_pt1[frame], sub_pt2[frame]);

	dir[frame] = sub_n[frame].cross(main_n[frame]);

	Point3f dir_ref = main_pt2[frame]-main_pt1[frame];
	if (dir[frame].dot(dir_ref) < 0)
	{
		dir[frame] = -dir[frame];
	}
	Normalize(dir[frame]);

	Point3f temp;
	double e = 0;
	temp = calib.calc_ref_MainCoord(sub_pt2_2d[frame],main_pt2_2d[frame],e,e,e);
	pt2[frame] = calib.cvtmain2world(temp);

// 	temp = calib.calc_ref_MainCoord(sub_pt1_2d[frame],main_pt1_2d[frame],e,e,e);
// 	pt1[frame] = calib.cvtmain2world(temp);
// 	dir[frame] = pt2[frame] - pt1[frame];
// 	Normalize(dir[frame]);

	pt1[frame] = pt2[frame] - club_langth*dir[frame];
}

// 3d data post processing
void ClubTracker::PointInterpolation( std::vector<cv::Point3f>& pt3d, const std::vector<float>& wrongindx )
{
	int n = pt3d.size();
	std::vector<float> indx(n);
	std::vector<float> x(n);
	std::vector<float> y(n);
	std::vector<float> z(n);

	for (int i = 0; i<n; ++i)
	{
		indx[i] = i;
		x[i] = pt3d[i].x;
		y[i] = pt3d[i].y;
		z[i] = pt3d[i].z;
	}

	std::vector<float> nindx = indx;
	for (int i = 0; i<wrongindx.size(); ++i)
	{
		int v = wrongindx[i]-i;
		nindx.erase(nindx.begin()+v);
		x.erase(x.begin()+v);
		y.erase(y.begin()+v);
		z.erase(z.begin()+v);
	}

	std::vector<float> temx = cubic_spline(nindx, x, wrongindx);
	std::vector<float> temy = cubic_spline(nindx, y, wrongindx);
	std::vector<float> temz = cubic_spline(nindx, z, wrongindx);

	for (int i = 0; i<wrongindx.size(); ++i)
	{
		pt3d[wrongindx[i]] = cv::Point3f(temx[i], temy[i], temz[i]);
	}
}
void ClubTracker::motionSmoothing3d( cv::Mat& data, cv::Mat& dir )
{

}
void ClubTracker::directionSmoothing( cv::Mat& dir, const std::vector<float>& param, const int n )
{	
	int row = dir.rows;
	Mat r1 = bezier_val(bezier_model(dir, param, n), param);

	int mx_indx = 0;
	float mx_val = 0;
	for (int i = 0; i<row; ++i)
	{
		float val = norm(r1.row(i) - dir.row(i));
		if (val>mx_val)
		{
			mx_indx = i;
			mx_val = val;
		}
	}

	int tem = 3;
	if (tem+1<mx_indx && mx_indx<row-tem-2)
	{
		Mat new_dir = dir.rowRange(0, mx_indx-tem).clone();
		new_dir.push_back(dir.rowRange(mx_indx+tem+1, row));

		Mat new_param = Mat(param).rowRange(0, mx_indx-tem).clone();
		new_param.push_back(Mat(param).rowRange(mx_indx+tem+1, row));

		Mat model = bezier_model(new_dir, new_param, n);

		bezier_val(model, param).copyTo(dir);
	}
	else r1.copyTo(dir);
}
void ClubTracker::directionSmoothing( cv::Mat& dir, const int n )
{
	int row = dir.rows;
	vector<float> param = chord_legth(dir);
	Mat r1 = bezier_val(bezier_model(dir, param, n), param);

	int mx_indx = 0;
	float mx_val = 0;
	for (int i = 0; i<row; ++i)
	{
		float val = norm(r1.row(i) - dir.row(i));
		if (val>mx_val)
		{
			mx_indx = i;
			mx_val = val;
		}
	}

	int tem = 5;

	if (tem+1<mx_indx && mx_indx<row-tem-2)
	{
		Mat new_dir = dir.rowRange(0, mx_indx-tem).clone();
		new_dir.push_back(dir.rowRange(mx_indx+tem+1, row));
		param = chord_legth(new_dir);

		Mat model = bezier_model(new_dir, param, n);


		vector<float> new_param(row);
		for (int i = 0; i<mx_indx-tem; ++i) new_param[i] = param[i];
		for (int i = mx_indx+tem+1; i<row; ++i) new_param[i] = param[i-tem*2-1];

		double a = param[mx_indx-tem-1];
		double b = param[mx_indx-tem];
		double d = b-a;
		for (int i = mx_indx-tem; i<mx_indx+tem+1; ++i)
		{
			new_param[i] = a+d/(tem*2+2)*(i-mx_indx+tem+1);
		}
		bezier_val(model, new_param).copyTo(dir);
	}
	else r1.copyTo(dir);
}

void ClubTracker::quaternionSmoothing( cv::Mat& dir )
{
}

cv::Vec4f ClubTracker::getApproxPlane( int start_indx, int end_indx, int ref_indx )
{
	int n = end_indx - start_indx + 1;

	Mat A(n, 2, CV_32FC1);
	Mat Z(n, 1, CV_32FC1);

	for (int i = 0; i<n; ++i)
	{
		A.at<float>(i, 0) = pt2[start_indx + i].x - pt2[ref_indx].x;
		A.at<float>(i, 1) = pt2[start_indx + i].y - pt2[ref_indx].y;
		
		Z.at<float>(i, 0) = pt2[start_indx + i].z - pt2[ref_indx].z;
	}

	Mat r;
	solve(A, Z, r, DECOMP_NORMAL);

	Point3f tem(r.at<float>(0,0), r.at<float>(1,0), -1);	
	float d = -tem.x*pt2[ref_indx].x - tem.y*pt2[ref_indx].y + pt2[ref_indx].z;
	float v = norm(tem);

	return Vec4f(tem.x/v, tem.y/v,tem.z/v, d/v);
}
cv::Vec4f ClubTracker::getApproxPlane( const cv::Mat pts )
{
	int n = pts.rows;

	Mat A(n, 3, CV_32FC1, Scalar(1));
	Mat Z(n, 1, CV_32FC1);

	pts.reshape(1).col(0).copyTo(A.col(0));
	pts.reshape(1).col(1).copyTo(A.col(1));
	pts.reshape(1).col(2).copyTo(Z);

	Mat r;
	solve(A, Z, r, DECOMP_NORMAL);

	Point3f tem(r.at<float>(0,0), r.at<float>(1,0), -1);
	float d = r.at<float>(2,0);

	float v = norm(tem);

	return Vec4f(tem.x/v, tem.y/v,tem.z/v, d/v);
}

void ClubTracker::drawClubTrajectory2d( cv::Mat& m_img, cv::Mat& s_img, int frame, CalibClass& calib )
{
	Point p1_cur, p1_pre, p2_cur, p2_pre;
	int indx = 0, indx_pre = 0;
	Scalar clr, clr2(0,255,0);

	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i]==0)
		{
			indx_pre = 0;
		}
		else if (isEmpty_indx[i])
		{
			if (isEmpty_indx[i]==1) clr = Scalar(200,255,0);
			else if (isEmpty_indx[i]==2) 			clr = Scalar(0,200,255);
			if (indx_pre)
			{
				p1_cur = Point(main_lines[i][0], main_lines[i][1]);
				p2_cur = Point(main_lines[i][2], main_lines[i][3]);
				p2_pre = Point(main_lines[indx_pre][2], main_lines[indx_pre][3]);
				line(m_img, p2_cur, p2_pre, clr, 2);
				line(m_img, p2_cur, p1_cur, clr2, 2);

				p1_cur = Point(sub_lines[i][0], sub_lines[i][1]);
				p2_cur = Point(sub_lines[i][2], sub_lines[i][3]);
				p2_pre = Point(sub_lines[indx_pre][2], sub_lines[indx_pre][3]);
				line(s_img, p2_cur, p2_pre, clr, 2);
				line(s_img, p2_cur, p1_cur, clr2, 2);
			}
			indx_pre = i;
		}
	}


	vector<Vec3f> lines;
	vector<Point2f> pts(1);
	Point l1, l2;
 	if (isEmpty_indx[frame])
 	{
	 	drawLineSegment(m_img, main_lines[frame], Scalar(255,200,100), 2);
		drawLineSegment(s_img, sub_lines[frame], Scalar(255,200,100), 2);
 	}
}
void ClubTracker::drawClubTrajectory2dup( cv::Mat& m_img, cv::Mat& s_img, int frame, CalibClass& calib )
{
	Point p1_cur, p1_pre, p2_cur, p2_pre;
	int indx = 0, indx_pre = 0;
	Scalar clr, clr2(0,255,0);

	clr = Scalar(200,255,0);

	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i]!=1)
		{
			indx_pre = 0;
		}
		else if (isEmpty_indx[i]==1)
		{
			if (indx_pre)
			{
				p1_cur = Point(main_lines[i][0], main_lines[i][1]);
				p2_cur = Point(main_lines[i][2], main_lines[i][3]);
				p2_pre = Point(main_lines[indx_pre][2], main_lines[indx_pre][3]);
				line(m_img, p2_cur, p2_pre, clr, 2);
				line(m_img, p2_cur, p1_cur, clr2, 2);

				p1_cur = Point(sub_lines[i][0], sub_lines[i][1]);
				p2_cur = Point(sub_lines[i][2], sub_lines[i][3]);
				p2_pre = Point(sub_lines[indx_pre][2], sub_lines[indx_pre][3]);
				line(s_img, p2_cur, p2_pre, clr, 2);
				line(s_img, p2_cur, p1_cur, clr2, 2);
			}
			else
			{
				p1_cur = Point(main_lines[i][0], main_lines[i][1]);
				p2_cur = Point(main_lines[i][2], main_lines[i][3]);
				line(m_img, p2_cur, p1_cur, clr2, 2);

				p1_cur = Point(sub_lines[i][0], sub_lines[i][1]);
				p2_cur = Point(sub_lines[i][2], sub_lines[i][3]);
				line(s_img, p2_cur, p1_cur, clr2, 2);
			}
			indx_pre = i;
		}
	}

	if (isEmpty_indx[frame]==1)
	{
		drawLineSegment(m_img, main_lines[frame], Scalar(255,200,100), 2);
		drawLineSegment(s_img, sub_lines[frame], Scalar(255,200,100), 2);
	}
}
void ClubTracker::drawClubTrajectory2ddown( cv::Mat& m_img, cv::Mat& s_img, int frame, CalibClass& calib )
{
	Point p1_cur, p1_pre, p2_cur, p2_pre;
	int indx = 0, indx_pre = 0;
	Scalar clr, clr2(0,255,0);
	clr = Scalar(0,200,255);

	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i]!=2)
		{
			indx_pre = 0;
		}
		else if (isEmpty_indx[i]==2)
		{
			if (indx_pre)
			{
				p1_cur = Point(main_lines[i][0], main_lines[i][1]);
				p2_cur = Point(main_lines[i][2], main_lines[i][3]);
				p2_pre = Point(main_lines[indx_pre][2], main_lines[indx_pre][3]);
				line(m_img, p2_cur, p2_pre, clr, 2);
				line(m_img, p2_cur, p1_cur, clr2, 2);

				p1_cur = Point(sub_lines[i][0], sub_lines[i][1]);
				p2_cur = Point(sub_lines[i][2], sub_lines[i][3]);
				p2_pre = Point(sub_lines[indx_pre][2], sub_lines[indx_pre][3]);
				line(s_img, p2_cur, p2_pre, clr, 2);
				line(s_img, p2_cur, p1_cur, clr2, 2);
			}
			else
			{ 
				p1_cur = Point(main_lines[i][0], main_lines[i][1]);
				p2_cur = Point(main_lines[i][2], main_lines[i][3]);
				line(m_img, p2_cur, p1_cur, clr2, 2);

				p1_cur = Point(sub_lines[i][0], sub_lines[i][1]);
				p2_cur = Point(sub_lines[i][2], sub_lines[i][3]);
				line(s_img, p2_cur, p1_cur, clr2, 2);
			}

			indx_pre = i;
		}
	}

	if (isEmpty_indx[frame]==2)
	{
		drawLineSegment(m_img, main_lines[frame], Scalar(255,200,100), 2);
		drawLineSegment(s_img, sub_lines[frame], Scalar(255,200,100), 2);
	}
}

cv::Mat ClubTracker::VariableOptimization2( const std::vector<cv::Mat>& m_imgs, const std::vector<cv::Mat>& s_imgs, const int frame, bool debug)
{
	num_frame = m_imgs.size();
	imgsize = m_imgs[0].size();

	int rows = 2, cols = 2;
	std::vector<Mat> subs;
	Mat rslt = Mat(imgsize.height*rows, imgsize.width*cols, CV_8UC3);
	MakeSubMat(rslt, subs, rows, cols);

	vector<Mat> tem;
	tem = VariableOptimization2(m_imgs, frame, debug);
	tem[0].copyTo(subs[0]);
	tem[1].copyTo(subs[1]);


	tem = VariableOptimization2(s_imgs, frame, debug);
	tem[0].copyTo(subs[2]);
	tem[1].copyTo(subs[3]);

	float sf = 0.5;
	Mat show;
	resize(rslt, show, Size(), sf, sf);

	return show;
}
std::vector<cv::Mat> ClubTracker::VariableOptimization2( const std::vector<cv::Mat>& img_buffer, const int frame, bool debug /*= false*/ )
{
	vector<Mat> subs(2);

	// 	cout<< frame << " : ";
	Size img_sz = img_buffer[0].size();

	Mat imgs[3], yuv[3], msk, msked;


	// get the motion mask and masked img=======================================
	imgs[0] = img_buffer[frame].clone();
	if (frame == 0)
	{
		imgs[1] = img_buffer[frame+1].clone();
		imgs[2] = img_buffer[frame].clone();
	}
	else if (frame == num_frame-1)
	{
		imgs[1] = img_buffer[frame-1].clone();
		imgs[2] = img_buffer[frame].clone();
	}
	else
	{
		imgs[1] = img_buffer[frame-1].clone();
		imgs[2] = img_buffer[frame+1].clone();
	}

	for (int i = 0; i<3; ++i)
		cvtColor(imgs[i], yuv[i], CV_BGR2YUV);
	msk = GetMotionMask(yuv[0], yuv[1], yuv[2], coef_threshold, coef_num_errod, coef_num_dilt);
	// 	msk = GetMotionMask(imgs[0], imgs[1], imgs[2], coef_threshold, coef_num_errod, coef_num_dilt);


// 	int gsiz = gauss_siz*2+1;
// 	GaussianBlur(msk, msk, Size(gsiz,gsiz), gauss_sig);

	cvtColor(msk, subs[0], CV_GRAY2BGR); // temporally save
	// 	bitwise_not(subs[0], subs[0]);
	subs[1] = subs[0].clone();
// 	imwrite("tem.bmp", msk);
	subs[0] = Scalar();

	vector<vector<Point>> contours = GetContour(msk, contour_threshold);
	vector<vector<cv::Vec4i>> new_lines;
	new_lines.reserve(contours.size());

	for (int i = 0; i<contours.size(); ++i)
	{
		// drawing approximated contour
		Mat tem(img_sz.height, img_sz.width, CV_8UC1, Scalar(0));
		drawVector(tem, contours[i], Scalar(255), 1, false);
		drawVector(subs[0], contours[i], Scalar(255,255,255), 2, false);

		// get bounding rect of contour
		Rect roi_Rect = boundingRect(Mat(contours[i]));
		Mat roi(tem, roi_Rect);
		Point shift_pt(roi_Rect.x, roi_Rect.y);

		// perform hough transform
		vector<cv::Vec4i> lin;
 		HoughLinesP(roi, lin, 1, CV_PI/180, hough_threshold, hough_minlangth, hough_maxgap);

		if (lin.size()>1)
		{
			vector<Vec4i> tem(lin.size(), Vec4i(shift_pt.x, shift_pt.y, shift_pt.x, shift_pt.y));
			new_lines.push_back(Mat(Mat(lin)+Mat(tem)));
		}
	}

// 	new_lines.resize(1);
// 	// tem fun
// 	int R = 3;
// 	LSWMS lswms(msk.size(), R, 10);	
// 	vector<LSEG> lSegs;
// 	vector<double> errors;
// 	lswms.run(msk, lSegs, errors);
// 	new_lines[0].resize(lSegs.size());
// 	for (int i = 0; i<lSegs.size(); ++i)
// 	{
// 		new_lines[0][i] = Vec4i(lSegs[i][0].x, lSegs[i][0].y, lSegs[i][1].x, lSegs[i][1].y);
// 	}


	for (int i = 0; i<new_lines.size(); ++i)
		drawLines(subs[0], new_lines[i], colors[i], 2);


	vector<vector<Vec4f>> new_lines2;
	new_lines2.reserve(new_lines.size());
	for (int i = 0; i<new_lines.size(); ++i)
	{
		vector<Vec4f> tem = lineGroupping3(new_lines[i], anglethr, distnthr1, distnthr2, debug);
		if (tem.size()>0) new_lines2.push_back(tem);
	}

	for (int i = 0; i<new_lines2.size(); ++i)
		drawLines(subs[1], new_lines2[i], colors[i], 2);

	return subs;
}

cv::Mat ClubTracker::VariableOptimization( const std::vector<cv::Mat>& m_imgs, const std::vector<cv::Mat>& s_imgs, const int frame, bool debug)
{
	num_frame = m_imgs.size();
	imgsize = m_imgs[0].size();

	int rows = 2, cols = 2;
	std::vector<Mat> subs;
	Mat rslt = Mat(imgsize.height*rows, imgsize.width*cols, CV_8UC3);
	MakeSubMat(rslt, subs, rows, cols);

	vector<Mat> tem;
	tem = VariableOptimization(m_imgs, frame, debug);
	tem[0].copyTo(subs[0]);
	tem[1].copyTo(subs[1]);


	tem = VariableOptimization(s_imgs, frame, debug);
	tem[0].copyTo(subs[2]);
	tem[1].copyTo(subs[3]);

	float sf = 0.5;
	Mat show;
	resize(rslt, show, Size(), sf, sf);

	return show;
}
std::vector<cv::Mat> ClubTracker::VariableOptimization( const std::vector<cv::Mat>& img_buffer, const int frame, bool debug /*= false*/ )
{
	vector<Mat> subs(2);

	// 	cout<< frame << " : ";
	Size img_sz = img_buffer[0].size();

	Mat imgs[3], yuv[3], msk, msked;


	// get the motion mask and masked img=======================================
	imgs[0] = img_buffer[frame].clone();
	if (frame == 0)
	{
		imgs[1] = img_buffer[frame+1].clone();
		imgs[2] = img_buffer[frame].clone();
	}
	else if (frame == num_frame-1)
	{
		imgs[1] = img_buffer[frame-1].clone();
		imgs[2] = img_buffer[frame].clone();
	}
	else
	{
		imgs[1] = img_buffer[frame-1].clone();
		imgs[2] = img_buffer[frame+1].clone();
	}

	for (int i = 0; i<3; ++i)
		cvtColor(imgs[i], yuv[i], CV_BGR2YUV);
	msk = GetMotionMask(yuv[0], yuv[1], yuv[2], coef_threshold, coef_num_errod, coef_num_dilt);
	// 	msk = GetMotionMask(imgs[0], imgs[1], imgs[2], coef_threshold, coef_num_errod, coef_num_dilt);
	
	//////////////////////////////////////////////////////////////////////////
	// tem function --->
	//////////////////////////////////////////////////////////////////////////
	if (0)
	{
		Mat tem, edge;
		imgs[0].copyTo(tem, msk);
		
		subs[0] = tem.clone();
	
		Canny(imgs[0], edge, canny1, canny2);
		edge.copyTo(tem, msk);
		cvtColor(tem, subs[1], CV_GRAY2BGR);
	
		vector<cv::Vec4i> lin;
		HoughLinesP(tem, lin, 0.5, CV_PI/180, hough_threshold, hough_minlangth, hough_maxgap);
		drawLines(subs[0], lin, colors[1], 2);
	
		char text[10];
		itoa(lin.size(), text, sizeof(text));
		cv::putText(subs[0], text, Point(10,30), CV_FONT_ITALIC, 1, Scalar(0,255,255), 2);
	
		if (!lin.empty())
		{
			vector<cv::Vec4f> newlin = lineGroupping(lin, anglethr, distnthr1, distnthr2, debug);
			drawLines(subs[1], newlin, colors[1], 2);
		}
	
		return subs;
	}

	//////////////////////////////////////////////////////////////////////////
	// <---
	//////////////////////////////////////////////////////////////////////////

	int gsiz = gauss_siz*2+1;
	GaussianBlur(msk, msk, Size(gsiz,gsiz), gauss_sig);


	cvtColor(msk, subs[0], CV_GRAY2BGR); // temporally save
// 	bitwise_not(subs[0], subs[0]);
	subs[1] = subs[0].clone();

	subs[0] = Scalar();
	
	if (0)
	{
		Mat edge;
		Canny(msk, edge, canny1, canny2);
		vector<cv::Vec4i> lin;
		HoughLinesP(edge, lin, 1, CV_PI/180, hough_threshold, hough_minlangth, hough_maxgap);
		if (lin.empty()) return subs;
		drawLines(subs[0], lin, colors[0], 2);

		vector<Vec4f> new_lines2 = lineGroupping(lin, anglethr, distnthr1, distnthr2, debug);
		if (new_lines2.empty()) return subs;
		drawLines(subs[1], new_lines2, colors[0], 2);

		return subs;
	}

	vector<vector<Point>> contours = GetContour(msk, contour_threshold);

	vector<vector<cv::Vec4i>> new_lines;
	new_lines.reserve(contours.size());

	for (int i = 0; i<contours.size(); ++i)
	{
		// drawing approximated contour
		Mat tem(img_sz.height, img_sz.width, CV_8UC1, Scalar(0));
		drawVector(tem, contours[i], Scalar(255), 1, false);
		drawVector(subs[0], contours[i], Scalar(255,255,255), 2, false);

		// get bounding rect of contour
		Rect roi_Rect = boundingRect(Mat(contours[i]));
		Mat roi(tem, roi_Rect);
		Point shift_pt(roi_Rect.x, roi_Rect.y);

		// perform hough transform
		vector<cv::Vec4i> lin;
		HoughLinesP(roi, lin, 1, CV_PI/180, hough_threshold, hough_minlangth, hough_maxgap);

		if (lin.size()>1)
		{
			vector<Vec4i> tem(lin.size(), Vec4i(shift_pt.x, shift_pt.y, shift_pt.x, shift_pt.y));
			new_lines.push_back(Mat(Mat(lin)+Mat(tem)));
		}
	}

	for (int i = 0; i<new_lines.size(); ++i)
		drawLines(subs[0], new_lines[i], colors[i], 2);


	vector<vector<Vec4f>> new_lines2;
	new_lines2.reserve(new_lines.size());
	for (int i = 0; i<new_lines.size(); ++i)
	{
		vector<Vec4f> tem = lineGroupping(new_lines[i], anglethr, distnthr1, distnthr2, debug);
		if (tem.size()>0) new_lines2.push_back(tem);
	}

	for (int i = 0; i<new_lines2.size(); ++i)
		drawLines(subs[1], new_lines2[i], colors[i], 2);

	return subs;
}


void ClubTracker::saveTrackingResult()
{
	saveData(pt2_smoothed, "3d_points_head.txt");
	saveData(dir_smoothed, "3d_points_dir.txt");
	saveData(pt1_smoothed, "3d_points_grip.txt");

	ofstream fout("swing _plane.txt");
	fout<<plane_upswing[0]<<"\t"<<plane_upswing[1]<<"\t"<<plane_upswing[2]<<"\t"<<plane_upswing[3]<<endl;
	fout<<plane_downswing[0]<<"\t"<<plane_downswing[1]<<"\t"<<plane_downswing[2]<<"\t"<<plane_downswing[3]<<endl;
	fout.close();
}

void ClubTracker::checkContinuous()
{
	vector<float> angle;
	vector<float> dist;
	float dd_angle = 0;
	int indx_wrong1 = 0;
	int indx_wrong2 = 0;
	int indx_wrong3 = 0;
	int indx_wrong4 = 0;


	getAngle(Mat(main_upswing[0]), angle);
 	for (int i = 2; i<angle.size(); ++i)
 	{
		dd_angle = angle[i] - 2*angle[i-1] + angle[i-2];

		if (abs(dd_angle) > angle_d2_thr) // 임의의 threshold 값
		{
			indx_wrong1 = i;
			break;
		}
 	}
	getAngle(Mat(sub_upswing[0]), angle);
	for (int i = 2; i<angle.size(); ++i)
	{
		dd_angle = angle[i] - 2*angle[i-1] + angle[i-2];

		if (abs(dd_angle) > angle_d2_thr) // 임의의 threshold 값
		{
			indx_wrong2 = i;
			break;
		}
	}
	getAngle(Mat(main_downswing[0]), angle);
	for (int i = 2; i<angle.size(); ++i)
	{
		dd_angle = angle[i] - 2*angle[i-1] + angle[i-2];

		if (abs(dd_angle) > angle_d2_thr) // 임의의 threshold 값
		{
			indx_wrong3 = i;
			break;
		}
	}
	getAngle(Mat(sub_downswing[0]), angle);
	for (int i = 2; i<angle.size(); ++i)
	{
		dd_angle = angle[i] - 2*angle[i-1] + angle[i-2];

		if (abs(dd_angle) > angle_d2_thr) // 임의의 threshold 값
		{
			indx_wrong4 = i;
			break;
		}
	}

	if (indx_wrong1 || indx_wrong3)
	{
		if (indx_wrong1>0)
		{
			int indx = upSwing_indx[indx_wrong1];
	 		main_lineVec[indx].erase(main_lineVec[indx].begin() + main_indx[indx]);
		}
		if (indx_wrong3>0)
		{
			int indx = downSwing_indx[indx_wrong3];
			main_lineVec[indx].erase(main_lineVec[indx].begin() + main_indx[indx]);
		}
		main_indx.clear();
// 		main_indx = CShortestPath().BuildGraph(main_lineVec);
		main_indx = ShortestPath(main_lineVec).findPath();
	}
	if (indx_wrong2||indx_wrong4)
	{
		if (indx_wrong2>0)
		{
			int indx = upSwing_indx[indx_wrong2];
			sub_lineVec[indx].erase(sub_lineVec[indx].begin() + sub_indx[indx]);
		}
		if (indx_wrong4>0)
		{
			int indx = downSwing_indx[indx_wrong4];
			sub_lineVec[indx].erase(sub_lineVec[indx].begin() + sub_indx[indx]);
		}
		sub_indx.clear();
// 		sub_indx = CShortestPath().BuildGraph(sub_lineVec);
		sub_indx = ShortestPath(sub_lineVec).findPath();
	}
	if (!indx_wrong1&&!indx_wrong2&&!indx_wrong3&&!indx_wrong4)
	{
		return;
	}

	sorting_LinePoints();
	determineUpDown();
	checkContinuous();

// 	getAngle(Mat(main_downswing1), angle);
// 	saveData(angle, "mdownswingangle.txt");
// 	getAngle(Mat(sub_upswing1), angle);
// 	saveData(angle, "supswingangle.txt");
// 	getAngle(Mat(sub_downswing1), angle);
// 	saveData(angle, "sdownswingangle.txt");
}
void ClubTracker::checkContinuous2()
{
	vector<float> angle;
	vector<float> dist;
	float dd_angle = 0;
	int indx_wrong1 = 0;
	int indx_wrong2 = 0;

	getAngle(Mat(main_lines1), angle);
	for (int i = 2; i<angle.size(); ++i)
	{
		dd_angle = angle[i] - 2*angle[i-1] + angle[i-2];

		if (abs(dd_angle) > angle_d2_thr) // 임의의 threshold 값
		{
			indx_wrong1 = i;
			break;
		}
	}
	getAngle(Mat(sub_lines1), angle);
	for (int i = 2; i<angle.size(); ++i)
	{
		dd_angle = angle[i] - 2*angle[i-1] + angle[i-2];

		if (abs(dd_angle) > angle_d2_thr) // 임의의 threshold 값
		{
			indx_wrong2 = i;
			break;
		}
	}

	if (indx_wrong1>0)
	{
		int indx = nonEmpty_indx[indx_wrong1];
		main_lineVec[indx].erase(main_lineVec[indx].begin() + main_indx[indx]);
		main_indx.clear();
		main_indx = ShortestPath(main_lineVec).findPath();
	}

	if (indx_wrong2>0)
	{
		int indx = nonEmpty_indx[indx_wrong2];
		sub_lineVec[indx].erase(sub_lineVec[indx].begin() + sub_indx[indx]);
		sub_indx.clear();
		sub_indx = ShortestPath(sub_lineVec).findPath();
	}	

	if (!indx_wrong1&&!indx_wrong2)
	{
		return;
	}

	sorting_LinePoints();
	checkContinuous2();
}

void ClubTracker::getAngle( const cv::Mat& data, vector<float>& angle )
{
	Mat dir = data.reshape(1).colRange(2,4) - data.reshape(1).colRange(0,2);

	Mat tem;
	phase(dir.col(0), dir.col(1), tem, true);
	angle = tem;

	for (int i = 1; i<angle.size(); ++i)
	{
		float d1 = angle[i] - angle[i-1];
		if (d1<-270) d1 += 360;
		else if (d1>270) d1 -= 360;

		angle[i] = angle[i-1] + d1;
	}
}

void ClubTracker::getDist( const std::vector<cv::Vec4f>& data, std::vector<float>& dist )
{
	dist.clear();
	dist.resize(data.size());
	for (int i = 0; i<data.size(); ++i)
	{
		dist[i] = pt2linDist(Point2f(), pt2line(data[i]));
	}
}

void ClubTracker::dadtaUpdate()
{
	main_pt1_2d = Mat(main_lines).reshape(1).colRange(0,2).reshape(2).clone();
	main_pt2_2d = Mat(main_lines).reshape(1).colRange(2,4).reshape(2).clone();
	sub_pt1_2d = Mat(sub_lines).reshape(1).colRange(0,2).reshape(2).clone();
	sub_pt2_2d = Mat(sub_lines).reshape(1).colRange(2,4).reshape(2).clone();
}

double ClubTracker::myFunctionup( const std::vector<double>& upt, const std::vector<int>& upindx, CalibClass& calib )
{
	// t를 이용해서 main image의 line segmet를 약간 비튼다.
	for (unsigned i = 0; i<upindx.size(); ++i)
	{		
		main_pt1_2d[upindx[i]].y += upt[i];
		main_pt2_2d[upindx[i]].y -= upt[i];
	}

	// 비틀어진 결과를 이용해서 3차원 direction vector를 다시 구한다.
	vector<Point3f> temdirup;
	calDirup(temdirup,calib);

	// 2차원 데이터를 원상태로 복구시킨다.
	main_pt1_2d = Mat(main_lines).reshape(1).colRange(0,2).reshape(2).clone();
	main_pt2_2d = Mat(main_lines).reshape(1).colRange(2,4).reshape(2).clone();

	// 3차원 방향벡터를 polynomial로 근사한 후 차이를 구한다.
	vector<Point3f> ntemdirup = temdirup;
	//poly_fitting(Mat(ntemdirup), 4);
	Mat data = Mat(ntemdirup);
	bezier_val(bezier_model(data, up_para, 4), up_para).copyTo(data);


	// 차이를 계산해서 return한다.
	double rval = 0;
	for (unsigned i = 0; i<upSwing_indx.size(); ++i)
	{
		rval += norm(temdirup[i] - ntemdirup[i]);
	}
	rval /= num_upswing;

	return rval;
}
double ClubTracker::myFunctiondown( const std::vector<double>& downt, const std::vector<int>& downindx, CalibClass& calib )
{
	// t를 이용해서 main image의 line segmet를 약간 비튼다.
	for (unsigned i = 0; i<downindx.size(); ++i)
	{		
		main_pt1_2d[downindx[i]].y += downt[i];
		main_pt2_2d[downindx[i]].y -= downt[i];
	}

	// 비틀어진 결과를 이용해서 3차원 direction vector를 다시 구한다.
	vector<Point3f> temdirdown;
	calDirdown(temdirdown,calib);

	// 2차원 데이터를 원상태로 복구시킨다.
	main_pt1_2d = Mat(main_lines).reshape(1).colRange(0,2).reshape(2).clone();
	main_pt2_2d = Mat(main_lines).reshape(1).colRange(2,4).reshape(2).clone();

	// 3차원 방향벡터를 polynomial로 근사한 후 차이를 구한다.
	vector<Point3f> ntemdirdown = temdirdown;
	Mat data = Mat(ntemdirdown);
	bezier_val(bezier_model(data, down_para, 6), down_para).copyTo(data);


	// 차이를 계산해서 return한다.
	double rval = 0;
	for (unsigned i = 0; i<downSwing_indx.size(); ++i)
	{
		rval += norm(temdirdown[i] - ntemdirdown[i]);
	}
	rval /= num_downswing;

	return rval;
}
void ClubTracker::calDirup( std::vector<cv::Point3f>& dirup, CalibClass& calib )
{
	dirup.clear();
	dirup.reserve(num_upswing);

	for (int i = 0; i<num_frame; ++i)
	{
		if (isEmpty_indx[i]==1)
		{
			Point3f tem = GetPlaneNormal(sub_pt0, calib.getDir(sub_pt1_2d[i], 1), calib.getDir(sub_pt2_2d[i], 1)).cross(GetPlaneNormal(main_pt0, calib.getDir(main_pt1_2d[i], 0), calib.getDir(main_pt2_2d[i], 0)));
			Point3f dir_ref = main_pt2[i]-main_pt1[i];
			if (tem.dot(dir_ref) < 0) tem = -tem;
			Normalize(tem);
			dirup.push_back(tem);
		}
	}
}
void ClubTracker::calDirdown( std::vector<cv::Point3f>& dirdown, CalibClass& calib )
{
	dirdown.clear();
	dirdown.reserve(num_downswing);

	for (int i = 0; i<num_frame; ++i)
	{
		if (isEmpty_indx[i]==2)
		{
			Point3f tem = GetPlaneNormal(sub_pt0, calib.getDir(sub_pt1_2d[i], 1), calib.getDir(sub_pt2_2d[i], 1)).cross(GetPlaneNormal(main_pt0, calib.getDir(main_pt1_2d[i], 0), calib.getDir(main_pt2_2d[i], 0)));
			Point3f dir_ref = main_pt2[i]-main_pt1[i];
			if (tem.dot(dir_ref) < 0) tem = -tem;
			Normalize(tem);
			dirdown.push_back(tem);
		}
	}

}
void ClubTracker::smoothing(const int iter_, const double eps, const double alpha, CalibClass& calib)
{
	vector<Point3f> temp1up, temp1down, temp2up, temp2down, temdirup, temdirdown;
	for(int i = 0; i < num_frame; ++i)
	{
		if (isEmpty_indx[i]==1)
		{
			temp1up.push_back(pt1[i]);
			temp2up.push_back(pt2[i]);
			temdirup.push_back(dir[i]);
		}
		if (isEmpty_indx[i]==2)
		{
			temp1down.push_back(pt1[i]);
			temp2down.push_back(pt2[i]);
			temdirdown.push_back(dir[i]);
		}
	}

	
	// head smoothing
#ifdef USING_SINTEF
	interpolation(temp2up, up_para, 100.0);
	interpolation(temp2down, down_para, 100.0);
#endif
	Mat data;
#ifndef USING_SINTEF
	data = Mat(temp2up);
	bezier_val(bezier_model(data, up_para, 3), up_para).copyTo(data);
	data = Mat(temp2down);
	bezier_val(bezier_model(data, down_para, 6), down_para).copyTo(data);
#endif

	outlier_up_indx.clear();
	outlier_up_indx2.clear();
	double pi = acos(-1.0);
	double anglethr = pi*0.1;
	for (int i = 0; i<num_upswing; ++i)
	{
		if (getUnsignedAngle(main_n[upSwing_indx[i]], sub_n[upSwing_indx[i]]) < anglethr )
		{
			outlier_up_indx.push_back(upSwing_indx[i]);
			outlier_up_indx2.push_back(i);
		}
	}

	vector<double> tup(outlier_up_indx.size());	
	for(int iter = 0; iter < iter_; ++iter)
	{
		vector<double> dir_t(tup.size(), 0);
		double c0 = myFunctionup(tup, outlier_up_indx, calib);
		vector<double> t_temp = tup;
		for(int i = 0; i < dir_t.size(); ++i)
		{
			t_temp = tup;
			t_temp[i] += eps;
			double c_t = myFunctionup(t_temp, outlier_up_indx, calib);
			dir_t[i] = (c_t-c0)/eps;
		}
		for(int i = 0; i < dir_t.size(); ++i)
		{
			double a = alpha;
			if(iter > iter_*0.8) a = alpha*0.5;			
			// 			if(dir_t[i] > 0) a = -a;
			// 			tup[i] += a*dir_t[i];
			tup[i] -= a*dir_t[i];
			// 			tup[i] += a*eps;
		}
	}
	for (unsigned i = 0; i<tup.size(); ++i)
	{		
		main_pt1_2d[outlier_up_indx[i]].y += tup[i];
		main_pt2_2d[outlier_up_indx[i]].y -= tup[i];
	}

	vector<Point3f> t;
	calDirup(t,calib);

	main_pt1_2d = Mat(main_lines).reshape(1).colRange(0,2).reshape(2).clone();
	main_pt2_2d = Mat(main_lines).reshape(1).colRange(2,4).reshape(2).clone();

	vector<Point3f> ntemdirup = t;
	data = Mat(ntemdirup);
	bezier_val(bezier_model(data, up_para, 4), up_para).copyTo(data);

	
	//==================================================================

	outlier_down_indx.clear();
	outlier_down_indx2.clear();
	for (int i = 0; i<num_downswing; ++i)
	{
		if (getUnsignedAngle(main_n[downSwing_indx[i]], sub_n[downSwing_indx[i]]) < anglethr )
		{
			outlier_down_indx.push_back(downSwing_indx[i]);
			outlier_down_indx2.push_back(i);
		}
	}


	vector<double> tdown(outlier_down_indx.size());	
	for(int iter = 0; iter < iter_; ++iter)
	{
		vector<double> dir_t(tdown.size(), 0);
		double c0 = myFunctiondown(tdown, outlier_down_indx, calib);
		vector<double> t_temp = tdown;
		for(int i = 0; i < dir_t.size(); ++i)
		{
			t_temp = tdown;
			t_temp[i] += eps;
			double c_t = myFunctiondown(t_temp, outlier_down_indx, calib);
			dir_t[i] = (c_t-c0)/eps;
		}
		for(int i = 0; i < dir_t.size(); ++i)
		{
			double a = alpha;
			if(iter > iter_*0.8) a = alpha*0.5;			
			// 			if(dir_t[i] > 0) a = -a;
			// 			tdown[i] += a*dir_t[i];
			tdown[i] -= a*dir_t[i];
			// 			tdown[i] += a*eps;

		}
	}
	for (unsigned i = 0; i<tdown.size(); ++i)
	{		
		main_pt1_2d[outlier_down_indx[i]].y += tdown[i];
		main_pt2_2d[outlier_down_indx[i]].y -= tdown[i];
	}

	vector<Point3f> td;
	calDirdown(td,calib);

	main_pt1_2d = Mat(main_lines).reshape(1).colRange(0,2).reshape(2).clone();
	main_pt2_2d = Mat(main_lines).reshape(1).colRange(2,4).reshape(2).clone();

	vector<Point3f> ntemdirdown = td;
	data = Mat(ntemdirdown);
	bezier_val(bezier_model(data, down_para, 6), down_para).copyTo(data);


	pt1_smoothed.clear();
	pt1_smoothed.resize(num_frame);
	pt2_smoothed.clear();
	pt2_smoothed.resize(num_frame);
	dir_smoothed.clear();
	dir_smoothed.resize(num_frame);

	for (int i = 0; i<num_upswing; ++i)
	{
		pt2_smoothed[upSwing_indx[i]] = temp2up[i];
		dir_smoothed[upSwing_indx[i]] = ntemdirup[i];
		pt1_smoothed[upSwing_indx[i]] = temp2up[i] - club_langth/norm(ntemdirup[i])*ntemdirup[i];
	}
	for (int i = 0; i<num_downswing; ++i)
	{
		pt2_smoothed[downSwing_indx[i]] = temp2down[i];
		dir_smoothed[downSwing_indx[i]] = ntemdirdown[i];
		pt1_smoothed[downSwing_indx[i]] = temp2down[i] - club_langth/norm(ntemdirdown[i])*ntemdirdown[i];
	}


	plane_upswing = getApproxPlane(Mat(temp2up));
	plane_downswing = getApproxPlane(Mat(temp2down));

	return;
}


