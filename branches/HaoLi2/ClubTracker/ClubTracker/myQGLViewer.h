#ifndef MYQGLVIEWER_H
#define MYQGLVIEWER_H

#include <QGLViewer/qglviewer.h>
#include <opencv2/opencv.hpp>

class MyClass;

class myQGLViewer : public QGLViewer
{
    Q_OBJECT

public:
    myQGLViewer(QWidget *parent = 0);

	MyClass* parent;
	
	int frame;


	bool isDrawCamer;
	bool isDrawClub;
	bool isDrawEnd_trjtory;
	bool isDrawShaft_trjtory;
	bool isDrawUpswingplane;
	bool isDrawDownswingplane;
	bool isDrawSmooth;
	
	cv::Point3f m_rect[2][4];

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
	std::vector<cv::Point3f> pt1_smoothed;
	std::vector<cv::Point3f> pt2_smoothed;

	int up_start, up_end;
	int down_start, down_end;

	std::vector<int> isEmpty_indx;
	std::vector<int> nonEmpty_indx;

	// drawing functions
	void setCamerCorner();
	void drawCamera(int frame);
	void drawClub(int frame);
	void drawClubTrajectory(int frame);
	void drawShaftTrajectory(int frame);
	void drawSmooth(int frame);
	void drawPlane(int frame);
	void drawUpswingPlane();
	void drawDownswingPlane();
	void drawPlane(cv::Point3f p0, cv::Point3f p1, cv::Point3f p2, int len);
	void drawPlane2(cv::Vec4f pln, int ist, int ied, int frame);

protected :
    virtual void draw();
    virtual void init();		
};

#endif // MYQGLVIEWER_H