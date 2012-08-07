#ifndef MYCLASS_H
#define MYCLASS_H

#define HAVE_KINECT
//#define HAVE_CAMERA

#include <QtGui/QDialog>
#include "ui_MyClass.h"
#include "ClubTracker.h"
#include "CalibClass.h"
#include <xncppwrapper.h>

#ifdef HAVE_CAMERA
#include "GenieCamera.h"
#endif

class myQGLViewer;
class QTimer;
class QLabel;

typedef struct _myColor{
	unsigned char r, g, b;
}myColor;

class MyClass : public QDialog
{
	Q_OBJECT

public:
	MyClass(QWidget *parent = 0, Qt::WFlags flags = 0);
	~MyClass();

	myQGLViewer*	m_QGLViewer;
	QTimer*			m_QTimer;	
	QLabel*			m_label_optimize;
		
	ClubTracker		tracker;
	
	// calibration class
	CalibClass		calib;
	CalibClass		calib_kinect;
	CalibClass		calib_cam_knt;

#ifdef HAVE_CAMERA
	// camera class
	GenieCamera		camera;
#endif
	cv::Mat main_display, sub_display;
	std::vector<cv::Mat> cam_imgs[2];
	std::vector<double>	highcamTime;
	
	// kinect class
	xn::Context context;
	xn::ImageGenerator g_image;
	xn::DepthGenerator g_depth;
	xn::Recorder	kinect_recorder;
	xn::Player		kinect_player;
	std::vector<int>		kinect_indx;
	std::vector<XnPoint3D> pts;
	std::vector<XnPoint3D> npts;
	std::vector<myColor> clr;
	std::vector<cv::Mat> kinect_color;
	std::vector<cv::Mat> kinect_depth;
	std::vector<double>	kinectTime;
	cv::Mat kinect_display1, kinect_display2, depth_buffer;
	int kinectFrameMax;
	void initKinectSystem();

	char filename[100];
	QSize imgsize;
	int num_frame;

	void updataCalibinform();
	void setCalib(cv::Size calibsiz, float squaresiz, int numimgs);

	cv::Mat trnsfrm;
	std::vector<cv::Point3d> transform(const cv::Mat& affine, std::vector<cv::Point3d>& pts);
	std::vector<cv::Point3f> transform(const cv::Mat& affine, std::vector<cv::Point3f>& pts);

	// mouse action
	int max_pt;
	std::vector<cv::Point2d> clickedPt_main;
	std::vector<cv::Point2d> clickedPt_sub;	
	std::vector<cv::Point2d> clickedPt_depth;
	std::vector<cv::Point3d> clickedPt_camera;
	std::vector<cv::Point3d> clickedPt_kinect;
	double displayScale_camera;
	double displayScale_kinect;
	QPoint clickedPoint1;
	QPoint clickedPoint2;
	QPoint clickedPoint3;
	bool isclicked1;
	bool isclicked2;
	bool isclicked3;
	int clickedindx;
	cv::Point3f main_origin, sub_origin, main_dir, sub_dir, real_3d;
	bool isInsideWidget(const QPoint& pt, const QPoint& pt_widget, const QSize& siz_widget);
	void calc3D();

	cv::Mat getAffine(const std::vector<cv::Point3d>& p, const std::vector<cv::Point3d>& q);

	// reprojected point
	cv::Point2d reprojectedpt_main;
	cv::Point2d reprojectedpt_sub;
	cv::Point2d reproject(const cv::Point3d& pt3d, const CalibClass& calib);

	// graph
	int graph_w, graph_h;
	cv::Mat head_graph[3];
	cv::Mat dir_graph[3];

	std::vector<cv::Point3f> headup, headdown, dirup, dirdown;
	std::vector<cv::Point3f> sheadup, sheaddown, sdirup, sdirdown;
	void setGraphData();
	void setGraph(int f);
	void drawGraph( cv::Mat& canvas, const cv::Mat& data, const cv::Scalar clr);
	void drawGraph( cv::Mat& canvas, const cv::Mat& data, const cv::Mat& sdata);


private:
	Ui::MyClassClass ui;

public slots:
	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
	void changeTab(int indx);

#ifdef HAVE_CAMERA
	void cameraPlay();
	void cameraRecord();
	void calibSet();
	void calibCamera();
	void calibKinect();
	void cameraCapture();
	void setCamParam(int);
	void setCamFPS();
	void kinectMaxFrame(int v){kinectFrameMax = v;};
	void delayTime(int v){camera.record_delay = v;};
#endif
	void toggleTimer();
	void timerCallback();
	void sliderCallback(int);
	void sliderCallback(bool);
	void vsliderCallback(int);

	void loadImg();
	void loadKinect();
	void loadPara();
	void optimize();
	void tracking2d();
	void tracking3d();
	void registration();
	void datasetting();
	void capture();

	void toggleCamera(bool b);
	void toggleClub(bool b);
	void toggleEndT(bool b);
	void toggleShaftT(bool b);
	void toggleUpP(bool b);
	void toggleDownP(bool b);
	void toggleSmooth(bool b);

	void setThreshold(int v){tracker.coef_threshold = v;sliderCallback(ui.horizontalSlider->value());};
	void saveclickedPt();

	void houghT(int v){tracker.hough_threshold = v;sliderCallback(ui.horizontalSlider->value());};
	void houghMinLangth(double v){tracker.hough_minlangth = v;sliderCallback(ui.horizontalSlider->value());};
	void houghMaxGap(double v){tracker.hough_maxgap = v;sliderCallback(ui.horizontalSlider->value());};
	void angleThr(double v){tracker.anglethr = v;sliderCallback(ui.horizontalSlider->value());};
	void distThr(double v){tracker.distnthr1 = v;sliderCallback(ui.horizontalSlider->value());};
	void canny1(double v){tracker.gauss_siz = v;sliderCallback(ui.horizontalSlider->value());};
	void canny2(double v){tracker.gauss_sig = v;sliderCallback(ui.horizontalSlider->value());};

	void hide_show_kinect();
	void hide_show_3D();

	void loaddata();
	void smoothingset();
	void hide_show_graph();
	
};

#endif // MYCLASS_H
