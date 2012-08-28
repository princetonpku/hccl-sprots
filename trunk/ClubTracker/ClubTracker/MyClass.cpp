#include "MyClass.h"
#include "myQGLViewer.h"
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>
#include <QLabel>
#include <QMouseEvent>
#include <fstream>
#include <Windows.h>
#include <ctime>

using namespace cv;
using namespace std;

static const int timerate = 10;

#define DATA_PATH "d:\\Users\\hounhoun\\Desktop\\datas\\0725"
#define PARA_PATH "d:\\Users\\hounhoun\\Desktop\\datas\\0725"
#define CALIB_PATH "calib\\%s.yml"
#define SAVE_CLICKED_PATH "calib\\%s.txt"

#define SAVE_PATH "data\\%d.bmp"
#define SAVE_KINECT_PATH "data\\kinect.oni"

char massge[100];

MyClass::MyClass(QWidget *parent, Qt::WFlags flags)
	: QDialog(parent, flags)
	, imgsize(700, 700), num_frame(0), kinectFrameMax(100), displayScale_camera(0.5), displayScale_kinect(0.7), clickedindx(0)
	, m_QGLViewer(NULL), m_QTimer(NULL), m_label_optimize(NULL)
	, max_pt(4), trnsfrm(Mat::eye(4,4,CV_64FC1))
	, graph_h(200), graph_w(300)

{
	// variable allocate
	m_label_optimize = new QLabel();
	m_QGLViewer = new myQGLViewer();
	m_QGLViewer->parent = this;
	m_QTimer = new QTimer(this);

	ui.setupUi(this);
	connect(ui.tabWidget, SIGNAL(currentChanged(int)), this, SLOT(changeTab(int)));
	connect(m_QTimer, SIGNAL(timeout()), this, SLOT(timerCallback()));
	connect(ui.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(sliderCallback(int)));	
	connect(ui.pushButton_play, SIGNAL(clicked()), this, SLOT(toggleTimer()));

	connect(ui.pushButton_loadimg, SIGNAL(clicked()), this, SLOT(loadImg()));
	connect(ui.pushButton_loadpara, SIGNAL(clicked()), this, SLOT(loadPara()));
	connect(ui.pushButton_kinect, SIGNAL(clicked()), this, SLOT(loadKinect()));
	connect(ui.pushButton_kinect_hides_how, SIGNAL(clicked()), this, SLOT(hide_show_kinect()));
	connect(ui.pushButton_optimize, SIGNAL(clicked()), this, SLOT(optimize()));
	connect(ui.pushButton_tracking2d, SIGNAL(clicked()), this, SLOT(tracking2d()));
	connect(ui.pushButton_datasetting, SIGNAL(clicked()), this, SLOT(datasetting()));
	connect(ui.pushButton_tracking3d, SIGNAL(clicked()), this, SLOT(tracking3d()));
	connect(ui.pushButton_3d_hide_show, SIGNAL(clicked()), this, SLOT(hide_show_3D()));
	connect(ui.pushButton_capture, SIGNAL(clicked()), this, SLOT(capture()));
	connect(ui.pushButton_registration, SIGNAL(clicked()), this, SLOT(registration()));
	connect(ui.pushButton_graph_show_hide, SIGNAL(clicked()), this, SLOT(hide_show_graph()));

	connect(ui.checkBox_camera, SIGNAL(clicked(bool)), this, SLOT(toggleCamera(bool)));
	connect(ui.checkBox_club, SIGNAL(clicked(bool)), this, SLOT(toggleClub(bool)));
	connect(ui.checkBox_endtr, SIGNAL(clicked(bool)), this, SLOT(toggleEndT(bool)));
	connect(ui.checkBox_shafttr, SIGNAL(clicked(bool)), this, SLOT(toggleShaftT(bool)));
	connect(ui.checkBox_upp, SIGNAL(clicked(bool)), this, SLOT(toggleUpP(bool)));
	connect(ui.checkBox_downp, SIGNAL(clicked(bool)), this, SLOT(toggleDownP(bool)));
	connect(ui.checkBox_smooth, SIGNAL(clicked(bool)), this, SLOT(toggleSmooth(bool)));

	connect(ui.checkBox_upswing, SIGNAL(clicked(bool)), this, SLOT(sliderCallback(bool)));
	connect(ui.checkBox_downswing, SIGNAL(clicked(bool)), this, SLOT(sliderCallback(bool)));

	connect(ui.spinBox_optimize, SIGNAL(valueChanged(int)), this, SLOT(setThreshold(int)));
	connect(ui.checkBox_optimize, SIGNAL(clicked(bool)), this, SLOT(sliderCallback(bool)));
	connect(ui.checkBox_optimize2, SIGNAL(clicked(bool)), this, SLOT(sliderCallback(bool)));
	connect(ui.pushButton_saveclickedpt, SIGNAL(clicked()), this, SLOT(saveclickedPt()));

	connect(ui.spinBox_hought, SIGNAL(valueChanged(int)), this, SLOT(houghT(int)));
	connect(ui.doubleSpinBox_houghminlangth, SIGNAL(valueChanged(double)), this, SLOT(houghMinLangth(double)));
	connect(ui.doubleSpinBox_houghmaxgap, SIGNAL(valueChanged(double)), this, SLOT(houghMaxGap(double)));
	connect(ui.doubleSpinBox_anglethr, SIGNAL(valueChanged(double)), this, SLOT(angleThr(double)));
	connect(ui.doubleSpinBox_distthr, SIGNAL(valueChanged(double)), this, SLOT(distThr(double)));
	connect(ui.doubleSpinBox_gsize, SIGNAL(valueChanged(double)), this, SLOT(canny1(double)));
	connect(ui.doubleSpinBox_distthr2, SIGNAL(valueChanged(double)), this, SLOT(canny2(double)));

	connect(ui.verticalScrollBar_edit, SIGNAL(valueChanged(int)), this, SLOT(vsliderCallback(int)));
	connect(ui.pushButton_smoothingset, SIGNAL(clicked()), this, SLOT(smoothingset()));

	connect(ui.radioButton_dir, SIGNAL(clicked(bool)), this, SLOT(sliderCallback(bool)));
	connect(ui.radioButton_head, SIGNAL(clicked(bool)), this, SLOT(sliderCallback(bool)));


	// calibration setting
	Size calibsiz(5, 3);
	float squaresiz = 100;
	int calib_numimgs = 15;
	ui.spinBox_calibnumimg->setValue(15);
	ui.spinBox_calibwidth->setValue(7);
	ui.spinBox_calibheight->setValue(10);
	ui.doubleSpinBox_calibsquaresiz->setValue(100);
	setCalib(calibsiz, squaresiz, calib_numimgs);
	
	// camera part
#ifdef HAVE_CAMERA
	ui.label_camera1->setFixedSize(imgsize);
	ui.label_camera2->setFixedSize(imgsize);
	camera.exposure = 3000;
	camera.gain = 12;
	camera.fps = 30000;
	camera.countOfbuff = 150;
	camera.InitCamera(ui.label_camera1->winId(), ui.label_camera2->winId());
	initKinectSystem();
	sprintf_s(massge, "there are %d camera(s)\n", camera.num_cameras);
	ui.textEdit->insertPlainText(massge);
	ui.spinBox_fps->setValue(camera.fps);
	ui.spinBox_exposure->setMaximum(15000);
	ui.spinBox_exposure->setValue(camera.exposure);
	ui.spinBox_gain->setMaximum(12);
	ui.spinBox_gain->setValue(camera.gain);
	ui.spinBox_kinectmax->setValue(kinectFrameMax);
	ui.spinBox_delaytime->setValue(camera.record_delay);
	connect(ui.spinBox_kinectmax, SIGNAL(valueChanged(int)), this, SLOT(kinectMaxFrame(int)));
	connect(ui.spinBox_delaytime, SIGNAL(valueChanged(int)), this, SLOT(delayTime(int)));
	connect(ui.pushButton_camera_play, SIGNAL(clicked()), this, SLOT(cameraPlay()));
	connect(ui.pushButton_camera_record, SIGNAL(clicked()), this, SLOT(cameraRecord()));
	connect(ui.pushButton_camera_capture, SIGNAL(clicked()), this, SLOT(cameraCapture()));
	connect(ui.pushButton_camera_set, SIGNAL(clicked()), this, SLOT(setCamFPS()));
	connect(ui.pushButton_cal, SIGNAL(clicked()), this, SLOT(setCamFPS()));
	connect(ui.spinBox_exposure, SIGNAL(valueChanged(int)), this, SLOT(setCamParam(int)));	
	connect(ui.spinBox_gain, SIGNAL(valueChanged(int)), this, SLOT(setCamParam(int)));	
	connect(ui.pushButton_calib_camera, SIGNAL(clicked()), this, SLOT(calibCamera()));
	connect(ui.pushButton_calibset, SIGNAL(clicked()), this, SLOT(calibSet()));
	num_frame = camera.countOfbuff;
	ui.spinBox_maxframe->setMaximum(num_frame);
	ui.spinBox_maxframe->setValue(num_frame);
	ui.spinBox_curframe->setMaximum(num_frame);	
	ui.horizontalSlider->setMaximum(num_frame);
	ui.horizontalSlider->setValue(0);
 	ui.tabWidget->setCurrentIndex(0);
#endif
#ifndef HAVE_CAMERA
	ui.tabWidget->setCurrentIndex(1);
#endif

	// image display label size
	QSize siz_tem = imgsize*displayScale_camera;	
	ui.label_display1->setFixedSize(siz_tem);
	ui.label_display2->setFixedSize(siz_tem);
	
	// kinect display label size
	QSize vga(XN_VGA_X_RES, XN_VGA_Y_RES);
	ui.label_kinect1->setFixedSize(vga*displayScale_kinect);
	ui.label_kinect2->setFixedSize(vga*displayScale_kinect);	
	ui.groupBox_kinect->hide();

	// 3D viewer size
	m_QGLViewer->setMinimumSize(QSize(imgsize.width()*0.7, imgsize.height()));
	ui.horizontalLayout_view->addWidget(m_QGLViewer, 2);
 	m_QGLViewer->hide();

	// optimize display label size
	int optimize_max = 1000;
	ui.spinBox_optimize->setMaximum(optimize_max);
	ui.spinBox_optimize->setValue(tracker.coef_threshold);
	m_label_optimize->setMinimumSize(imgsize);
	ui.horizontalLayout_view->addWidget(m_label_optimize, 3);
	m_label_optimize->hide();

	// data setting
	ui.spinBox_datasetting->setMaximum(3);
	ui.spinBox_datasetting->setValue(0);	

	ui.checkBox_camera->click();
	ui.checkBox_club->click();
	ui.checkBox_endtr->click();
	ui.checkBox_club->click();
	ui.checkBox_endtr->click();
	ui.checkBox_smooth->click();
	ui.checkBox_upswing->click();
	ui.checkBox_downswing->click();	

	ui.doubleSpinBox_x->setDecimals(5);
	ui.doubleSpinBox_y->setDecimals(5);
	ui.doubleSpinBox_z->setDecimals(5);
	ui.doubleSpinBox_x->setMaximum(9999999);
	ui.doubleSpinBox_y->setMaximum(9999999);
	ui.doubleSpinBox_z->setMaximum(9999999);
	ui.doubleSpinBox_x->setMinimum(-9999999);
	ui.doubleSpinBox_y->setMinimum(-9999999);
	ui.doubleSpinBox_z->setMinimum(-9999999);
	
	ui.doubleSpinBox_gsize->setMaximum(500);
	ui.doubleSpinBox_distthr2->setMaximum(500);
	ui.spinBox_hought->setValue(tracker.hough_threshold);
	ui.doubleSpinBox_houghminlangth->setValue(tracker.hough_minlangth);
	ui.doubleSpinBox_houghmaxgap->setValue(tracker.hough_maxgap);
	ui.doubleSpinBox_anglethr->setValue(tracker.anglethr);
	ui.doubleSpinBox_distthr->setValue(tracker.distnthr1);
	ui.doubleSpinBox_gsize->setValue(tracker.gauss_siz);
	ui.doubleSpinBox_distthr2->setValue(tracker.distnthr2);
	
	// graph
	QSize graphsiz(graph_w, graph_h);
	ui.label_graphx->setFixedSize(graphsiz);
	ui.label_graphy->setFixedSize(graphsiz);
	ui.label_graphz->setFixedSize(graphsiz);
	ui.spinBox_edit->setMinimum(-600);
	ui.spinBox_edit->setMaximum(600);
	ui.verticalScrollBar_edit->setMinimum(-600);
	ui.verticalScrollBar_edit->setMaximum(600);
	ui.spinBox_edit->setValue(0);
	ui.verticalScrollBar_edit->setValue(0);
	ui.groupBox_graph->hide();
	ui.radioButton_dir->click();
	this->adjustSize();
}

MyClass::~MyClass()
{
#ifdef HAVE_CAMERA
	camera.RelaeaseCamera();
#endif
}

void MyClass::toggleTimer()
{
	if(m_QTimer->isActive()) m_QTimer->stop();
	else m_QTimer->start(timerate);
}
void MyClass::timerCallback()
{
	ui.horizontalSlider->setValue(ui.horizontalSlider->value()+1);
	if(ui.horizontalSlider->value() == num_frame-1)
	{
	 	ui.horizontalSlider->setValue(0);
		m_QTimer->stop();
	}
}
void MyClass::sliderCallback( int f)
{
	char filename[50];

	// camera part
	if (cam_imgs[0].size()>0)
	{
		main_display = cam_imgs[0][f].clone();
		sub_display = cam_imgs[1][f].clone();
		if (tracker.isTrackingFinished2d)
		{
			if(ui.checkBox_upswing->isChecked()) tracker.drawClubTrajectory2dup(main_display, sub_display, f, calib);
			if(ui.checkBox_downswing->isChecked()) tracker.drawClubTrajectory2ddown(main_display, sub_display, f, calib);
		}
		
		if (1)
		{
			vector<Vec3f> lines;
			vector<Point2f> pts(1);

			for (int i = 0; i<clickedPt_main.size(); ++i)
			{
				circle(main_display, clickedPt_main[i], 4, Scalar(0,0,255), 4);
			}
			for (int i = 0; i<clickedPt_sub.size(); ++i)
			{
				circle(sub_display, clickedPt_sub[i], 4, Scalar(0,255,255), 4);
			}
			
			if (1)
			{
				pts[0] = Point2f(clickedPoint1.x(), clickedPoint1.y());
				computeCorrespondEpilines(Mat(pts), 2, calib.F, lines);
				Point l1(0, -(1/lines[0][1])*(lines[0][0]*0+lines[0][2]));
				Point l2(700, -(1/lines[0][1])*(lines[0][0]*700+lines[0][2]));
				line(sub_display, l1, l2, Scalar(0,255,0), 2);
			}
			if (1)
			{
				pts[0] = Point(clickedPoint2.x(), clickedPoint2.y());
				computeCorrespondEpilines(Mat(pts), 1, calib.F, lines);
				Point l1(0, -(1/lines[0][1])*(lines[0][0]*0+lines[0][2]));
				Point l2(700, -(1/lines[0][1])*(lines[0][0]*700+lines[0][2]));
				line(main_display, l1, l2, Scalar(0,255,0), 2);
			}
		}

		
// 		if (tracker.isTrackingFinished3d)
// 		{
// 
// 			circle(main_display, reprojectedpt_main, 2, Scalar(0,0,255), 2);
// 			circle(sub_display, reprojectedpt_sub, 2, Scalar(0,0,255), 2);		
// 		}

		ui.label_display1->setPixmap(QPixmap::fromImage(QImage(main_display.datastart,main_display.cols,main_display.rows,QImage::Format_RGB888).scaled(ui.label_display1->size()).rgbSwapped()));
		ui.label_display2->setPixmap(QPixmap::fromImage(QImage(sub_display.datastart,sub_display.cols,sub_display.rows,QImage::Format_RGB888).scaled(ui.label_display2->size()).rgbSwapped()));
	}

	// kinect part
	if (kinect_player.IsValid())
	{
		int ikinect = kinect_indx[f];
		ui.spinBox_maxframe->setValue(ikinect);
		context.WaitAndUpdateAll();

		kinect_player.SeekToFrame(g_depth.GetName(), kinect_indx[f], XN_PLAYER_SEEK_SET);

		// color image	
		const XnRGB24Pixel* pColorMap = g_image.GetRGB24ImageMap();
		kinect_display1 = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3, (uchar*)pColorMap).clone();
		ui.label_kinect1->setPixmap(QPixmap::fromImage(QImage(kinect_display1.datastart,kinect_display1.cols,kinect_display1.rows,QImage::Format_RGB888).scaled(ui.label_kinect1->size())));
		// depth image	
		const XnDepthPixel* pDepthMap = g_depth.GetDepthMap();
		depth_buffer = Mat(XN_VGA_Y_RES, XN_VGA_X_RES, CV_16SC1, (unsigned short*)pDepthMap);
		double mx;
		minMaxLoc(depth_buffer, NULL, &mx);
		depth_buffer.convertTo(kinect_display2, CV_8U, 255/mx);
		cvtColor(kinect_display2, kinect_display2, CV_GRAY2BGR);
		for (int i = 0; i<clickedPt_depth.size(); ++i)
		{
			circle(kinect_display2, clickedPt_depth[i], 4, Scalar(0,255,255), 4);
		}
		
		if (tracker.isTrackingFinished3d)
		{
			XnPoint3D tem[1];
			tem[0].X = tracker.pt2_smoothed[f].x;
			tem[0].Y = tracker.pt2_smoothed[f].y;
			tem[0].Z = tracker.pt2_smoothed[f].z;
			XnPoint3D tem2[1];
			double temcolor = tem[0].Z*255/mx;
			g_depth.ConvertRealWorldToProjective(1, tem, tem2);

			circle(kinect_display2, Point(tem2[0].X, tem2[0].Y), 2, Scalar(temcolor, temcolor, temcolor), 2);
		}

		ui.label_kinect2->setPixmap(QPixmap::fromImage(QImage(kinect_display2.datastart,kinect_display2.cols,kinect_display2.rows,QImage::Format_RGB888).scaled(ui.label_kinect2->size())));
	}


	// 3dviewer part
	if (m_QGLViewer->isVisible())
	{
		m_QGLViewer->frame = f;

		if (kinect_player.IsValid())
		{
			clr.clear();
			pts.clear();
			npts.clear();
			clr.resize(XN_VGA_X_RES*XN_VGA_Y_RES);
			pts.resize(XN_VGA_X_RES*XN_VGA_Y_RES);
			npts.resize(XN_VGA_X_RES*XN_VGA_Y_RES);

// 			#pragma omp parallel for
			for (int i = 0; i<XN_VGA_X_RES*XN_VGA_Y_RES; ++i)
			{
				int x = i%XN_VGA_X_RES;
				int y = i/XN_VGA_X_RES;
				pts[i].X = XN_VGA_X_RES-x;
				pts[i].Y = y;
				pts[i].Z = depth_buffer.at<ushort>(y,x);
			}	
			g_depth.ConvertProjectiveToRealWorld(XN_VGA_X_RES*XN_VGA_Y_RES, &(pts[0]), &(npts[0]));			
		}
		m_QGLViewer->updateGL();

// 		sprintf_s(filename, "capture\\capture_%d.jpg", f+100);
// 		m_QGLViewer->saveSnapshot(QString(filename), true);
	}

	// optmize part
	if (m_label_optimize->isVisible())
	{
		bool debug;
		if (ui.checkBox_optimize->isChecked()) debug = true;
		else debug = false;
		
		Mat tem;

		if (ui.checkBox_optimize2->isChecked()) tem = tracker.VariableOptimization2(cam_imgs[0], cam_imgs[1], f, debug);
		else tem = tracker.VariableOptimization(cam_imgs[0], cam_imgs[1], f, debug);		

		m_label_optimize->setPixmap(QPixmap::fromImage(QImage(tem.datastart,tem.cols,tem.rows,QImage::Format_RGB888).scaled(m_label_optimize->size()).rgbSwapped()));
	}

	// graph part
	if (tracker.isTrackingFinished3d && !ui.groupBox_graph->isHidden())
	{
		setGraph(f);
		if (ui.radioButton_head->isChecked())
		{
			ui.label_graphx->setPixmap(QPixmap::fromImage(QImage(head_graph[0].datastart,head_graph[0].cols,head_graph[0].rows,QImage::Format_RGB888).scaled(ui.label_graphx->size()).rgbSwapped()));
			ui.label_graphy->setPixmap(QPixmap::fromImage(QImage(head_graph[1].datastart,head_graph[1].cols,head_graph[1].rows,QImage::Format_RGB888).scaled(ui.label_graphy->size()).rgbSwapped()));
			ui.label_graphz->setPixmap(QPixmap::fromImage(QImage(head_graph[2].datastart,head_graph[2].cols,head_graph[2].rows,QImage::Format_RGB888).scaled(ui.label_graphz->size()).rgbSwapped()));
		} 
		else
		{
			ui.label_graphx->setPixmap(QPixmap::fromImage(QImage(dir_graph[0].datastart,dir_graph[0].cols,dir_graph[0].rows,QImage::Format_RGB888).scaled(ui.label_graphx->size()).rgbSwapped()));
			ui.label_graphy->setPixmap(QPixmap::fromImage(QImage(dir_graph[1].datastart,dir_graph[1].cols,dir_graph[1].rows,QImage::Format_RGB888).scaled(ui.label_graphy->size()).rgbSwapped()));
			ui.label_graphz->setPixmap(QPixmap::fromImage(QImage(dir_graph[2].datastart,dir_graph[2].cols,dir_graph[2].rows,QImage::Format_RGB888).scaled(ui.label_graphz->size()).rgbSwapped()));
		}
	}

}

void MyClass::sliderCallback( bool b)
{
	sliderCallback(ui.horizontalSlider->value());
}

void MyClass::loadImg()
{
	if (calib.distCoeffs[0].at<double>(0) == 0)
	{
		QMessageBox msgBox;
		msgBox.setText("Load calib parameters.");
		msgBox.exec();
		return;
	}

	QStringList files = QFileDialog::getOpenFileNames(this,"Select one or more files to open", DATA_PATH, "Images (*.png *.bmp *.jpg)");

	if (files.size()==0) return;

	num_frame = files.size()*0.5;
	// load all images
	cam_imgs[0].clear();
	cam_imgs[1].clear();
	cam_imgs[0].resize(num_frame);
	cam_imgs[1].resize(num_frame);
#pragma omp parallel for num_threads(8)
	for (int i = 0; i<num_frame; ++i)
	{
// 		cam_imgs[1][i] = calib.undistImage(imread(files[i].toStdString()), 0);
// 		cam_imgs[0][i] = calib.undistImage(imread(files[i+num_frame].toStdString()), 1);
		cam_imgs[1][i] = imread(files[i].toStdString());
		cam_imgs[0][i] = imread(files[i+num_frame].toStdString());
	}

	imgsize = QSize(cam_imgs[0][0].cols, cam_imgs[0][0].rows);
	tracker.Initialize(num_frame, Size(imgsize.width(), imgsize.height()));

	ui.label_display1->setFixedSize(imgsize*displayScale_camera);
	ui.label_display2->setFixedSize(imgsize*displayScale_camera);
	m_QGLViewer->setFixedSize(imgsize);
	m_label_optimize->setFixedSize(imgsize);
	m_QGLViewer->hide();
	m_label_optimize->hide();
	ui.groupBox_kinect->hide();
	this->adjustSize();

	ui.spinBox_maxframe->setMaximum(num_frame-1);
	ui.spinBox_maxframe->setValue(num_frame-1);
	ui.spinBox_curframe->setMaximum(num_frame-1);
	ui.horizontalSlider->setMaximum(num_frame-1);
	ui.horizontalSlider->setValue(0);
	sliderCallback(0);
}
void MyClass::loadKinect()
{
	QStringList files = QFileDialog::getOpenFileNames(this,"Select one or more files to open", DATA_PATH, "kinect (*.oni *.txt)");
	if(files.size()==0) return;
	
	context.Release();
	context.Init();
	context.OpenFileRecording(files[0].toStdString().c_str(), kinect_player);
	g_image.Create(context);
	g_depth.Create(context);
	context.StartGeneratingAll();

	if (files.size()==2)
	{
		inputData(kinect_indx, files[1].toStdString().c_str());
	}
	else
	{
		inputData(highcamTime, files[1].toStdString().c_str());
		inputData(kinectTime, files[2].toStdString().c_str());

		kinect_indx.resize(highcamTime.size());
		for (unsigned i = 0; i<highcamTime.size(); ++i)
		{
			double min_val = 9999;
			for (unsigned j = 0; j<kinectTime.size(); ++j)
			{
				double gap = abs(highcamTime[i] - kinectTime[j]);
				if (gap < min_val)
				{
					min_val = gap;					
					kinect_indx[i] = min(j+3, kinectTime.size()-1);
				}
			}
		}
	}
	
	ui.groupBox_kinect->show();
	m_QGLViewer->show();
	this->adjustSize();

	sliderCallback(ui.horizontalSlider->value());
}
void MyClass::loadPara()
{
	QString file = QFileDialog::getOpenFileName(this,"Select calib parameter file", PARA_PATH, "yml (*.yml)");
	if (file==NULL) return;
	
	calib.initCalib();
	calib.loadPara(file.toStdString().c_str());
}
void MyClass::optimize()
{
	if (m_label_optimize->isHidden())
	{
		if (num_frame == 0)
		{
			QMessageBox msgBox;
			msgBox.setText("Load images.");
			msgBox.exec();
			return;
		}
	
		m_QGLViewer->hide();
		m_label_optimize->show();
		this->adjustSize();
		
		sliderCallback(ui.horizontalSlider->value());
	}
	else
	{
		m_label_optimize->hide();
		this->adjustSize();
	}
}
void MyClass::tracking2d()
{
	if (num_frame == 0)
	{
		QMessageBox msgBox;
		msgBox.setText("Load images.");
		msgBox.exec();
		return;
	}

	double start = 0;
	char endtime[100];
	start = clock();

	// traking all frames	
	tracker.coef_threshold = ui.spinBox_optimize->value();
	tracker.Initialize(num_frame, Size(imgsize.width(), imgsize.height()));
	tracker.Tracking2D(cam_imgs[0], cam_imgs[1], calib);
	tracker.dataconverting(ui.spinBox_datasetting->value());

	sprintf_s(endtime, "time = %f", (clock() - start)/CLOCKS_PER_SEC);	
	QMessageBox msgBox2;
	msgBox2.setText(endtime);
	msgBox2.exec();
	
	m_label_optimize->hide();
	this->adjustSize();

	ui.horizontalSlider->setValue(0);
	sliderCallback(0);
}
void MyClass::tracking3d()
{
	if (!tracker.isTrackingFinished2d)
	{
		QMessageBox msgBox;
		msgBox.setText("tracking 2d first!");
		msgBox.exec();
		return;
	}
	else if (calib.distCoeffs[0].at<double>(0) == 0)
	{
		QMessageBox msgBox;
		msgBox.setText("Load calib parameters.");
		msgBox.exec();
		return;
	}

	tracker.Tracking3D(calib);
	tracker.smoothing(200, 0.015, 1.2, calib);
	tracker.saveTrackingResult();

	m_QGLViewer->setCamerCorner();

	m_QGLViewer->up_start = tracker.up_start;
	m_QGLViewer->up_end = tracker.up_end;
	m_QGLViewer->down_start = tracker.down_start;
	m_QGLViewer->down_end = tracker.down_end;
	m_QGLViewer->isEmpty_indx = tracker.isEmpty_indx;
	m_QGLViewer->nonEmpty_indx = tracker.nonEmpty_indx;
	
	m_QGLViewer->main_pt0 = tracker.main_pt0;
	m_QGLViewer->main_pt1 = tracker.main_pt1;
	m_QGLViewer->main_pt2 = tracker.main_pt2;
	m_QGLViewer->main_n = tracker.main_n;

	m_QGLViewer->sub_pt0 = tracker.sub_pt0;
	m_QGLViewer->sub_pt1 = tracker.sub_pt1;
	m_QGLViewer->sub_pt2 = tracker.sub_pt2;
	m_QGLViewer->sub_n = tracker.sub_n;

	// tracking result points
	m_QGLViewer->pt1 = tracker.pt1;
	m_QGLViewer->pt2 = tracker.pt2;
	m_QGLViewer->pt1_smoothed = tracker.pt1_smoothed;
	m_QGLViewer->pt2_smoothed = tracker.pt2_smoothed;
	
	m_QGLViewer->show();
	m_label_optimize->hide();
	this->adjustSize();

	setGraphData();
	ui.groupBox_graph->show();

	sliderCallback(ui.horizontalSlider->value());
}
void MyClass::datasetting()
{
	if (!tracker.isTrackingFinished2d)
	{
		QMessageBox msgBox;
		msgBox.setText("tracking 2d first!");
		msgBox.exec();
		return;
	}

	tracker.dataconverting(ui.spinBox_datasetting->value());
	sliderCallback(ui.horizontalSlider->value());
}
void MyClass::capture()
{
	if (m_QGLViewer->isVisible()) m_QGLViewer->saveSnapshot();	
	imwrite("main_img.png",main_display);
	imwrite("sub_img.png",sub_display);
}

void MyClass::toggleCamera(bool b){m_QGLViewer->isDrawCamer = b;m_QGLViewer->updateGL();};
void MyClass::toggleClub(bool b){m_QGLViewer->isDrawClub= b;m_QGLViewer->updateGL();};
void MyClass::toggleEndT(bool b){m_QGLViewer->isDrawEnd_trjtory= b;m_QGLViewer->updateGL();};
void MyClass::toggleShaftT(bool b){m_QGLViewer->isDrawShaft_trjtory= b;m_QGLViewer->updateGL();};
void MyClass::toggleUpP(bool b){m_QGLViewer->isDrawUpswingplane = b;m_QGLViewer->updateGL();};
void MyClass::toggleDownP(bool b){m_QGLViewer->isDrawDownswingplane= b;m_QGLViewer->updateGL();};
void MyClass::toggleSmooth( bool b ){m_QGLViewer->isDrawSmooth= b;m_QGLViewer->updateGL();};

void MyClass::initKinectSystem()
{
	XnStatus nRetVal = XN_STATUS_OK;
	xn::EnumerationErrors errors;

	// Initialize context object
	nRetVal = context.Init();
	if (nRetVal != XN_STATUS_OK)
	{
		sprintf_s(massge, "kinect init error!\n");
		ui.textEdit->insertPlainText(massge);
	}
	nRetVal = g_depth.Create(context);
	if (nRetVal != XN_STATUS_OK)
	{
		sprintf_s(massge, "depth generator error!\n");
		ui.textEdit->insertPlainText(massge);
	}

	nRetVal = g_image.Create(context);
	if (nRetVal != XN_STATUS_OK)
	{
		sprintf_s(massge, "image generator error!\n");
		ui.textEdit->insertPlainText(massge);
	}

	XnMapOutputMode mapMode;  
	mapMode.nXRes = XN_VGA_X_RES; 
	mapMode.nYRes = XN_VGA_Y_RES; 
	mapMode.nFPS = 30; 

	g_depth.SetMapOutputMode(mapMode);
	g_image.SetMapOutputMode(mapMode);	
	
	kinect_recorder.Create(context);
	kinect_recorder.SetDestination(XN_RECORD_MEDIUM_FILE, SAVE_KINECT_PATH);
	kinect_recorder.AddNodeToRecording(g_image);
	kinect_recorder.AddNodeToRecording(g_depth);
	context.StartGeneratingAll();
}

#ifdef HAVE_CAMERA
void MyClass::cameraPlay()
{
	camera.ToggleCamera();
}
void MyClass::cameraRecord()
{
	// record start--->
	sprintf_s(massge, "record start!\n");
	ui.textEdit->insertPlainText(massge);
	camera.Record();
	for (int i = 0; i<kinectFrameMax; ++i)
	{
		context.WaitAndUpdateAll();
		kinectTime[i] = (double)clock() /CLOCKS_PER_SEC;				
		Sleep(10);
	}
	if(camera.isOnRecord) camera.RecordStop();
	context.StopGeneratingAll();
	kinect_recorder.Release();
	g_image.Release();
	g_depth.Release();
	context.Release();
	ui.textEdit->insertPlainText("record stop!\n");
	sprintf_s(massge, "camera : %d\nkinect : %d\n", camera.num_record_frame, kinectFrameMax);
	ui.textEdit->insertPlainText(massge);
	//<-- record end
	
	// save high cam data	
	num_frame = camera.num_record_frame;
	highcamTime.resize(num_frame);
	std::copy(camera.record_time.begin(), camera.record_time.begin()+num_frame, highcamTime.begin());
	camera.out2OpenCV(cam_imgs);

	// cam-kinect frame µ¿±âÈ­
	kinect_indx.clear();
	kinect_indx.resize(highcamTime.size());
	for (unsigned i = 0; i<highcamTime.size(); ++i)
	{
		double min_val = 9999;
		for (unsigned j = 0; j<kinectTime.size(); ++j)
		{
			double gap = abs(highcamTime[i] - kinectTime[j]);
			if (gap < min_val)
			{
				min_val = gap;					
				kinect_indx[i] = min(j+2, kinectTime.size()-1);
			}
		}
	}
	saveData(kinect_indx, "data//kinect_indx.txt");
	saveData(kinectTime, "data//time_kinect.txt");
	saveData(highcamTime, "data//time_cam.txt");

//#pragma omp parallel for
	for (int i = 0; i<num_frame; ++i)
	{
		sprintf_s(filename, "data//%d.bmp", i+2000);
		imwrite(filename, cam_imgs[0][i]);
		sprintf_s(filename, "data//%d.bmp", i+1000);
		imwrite(filename, cam_imgs[1][i]);
	}
#pragma omp parallel for
	for (int i = 0; i<cam_imgs[0].size(); ++i)
	{
		Mat tem;
		tem = cam_imgs[0][i].clone();
		cam_imgs[0][i] = calib.undistImage(tem, 1);
		tem = cam_imgs[1][i].clone();
		cam_imgs[1][i] = calib.undistImage(tem, 0);
	}

	// openni player setting
	context.Init();
	context.OpenFileRecording(SAVE_KINECT_PATH, kinect_player);
	g_image.Create(context);
	g_depth.Create(context);
	context.StartGeneratingAll();

	// slider setting
	int num = num_frame -1;
	ui.spinBox_curframe->setMaximum(num);
	ui.spinBox_maxframe->setMaximum(num);
	ui.horizontalSlider->setMaximum(num);

	//ui.tabWidget->setCurrentIndex(1);
}
void MyClass::cameraCapture()
{
	camera.capture();
}
void MyClass::setCamParam(int)
{	
	camera.exposure = ui.spinBox_exposure->value();
	camera.gain = ui.spinBox_gain->value();
	camera.updataParam();
}
void MyClass::setCamFPS()
{
	camera.fps = ui.spinBox_fps->value();
	camera.updataFPS();
}
void MyClass::calibSet()
{
	Size calibsiz(ui.spinBox_calibwidth->value(),ui.spinBox_calibheight->value());
	float squaresiz = ui.doubleSpinBox_calibsquaresiz->value();
	int calib_numimgs = ui.spinBox_calibnumimg->value();
	setCalib(calibsiz, squaresiz, calib_numimgs);
}
void MyClass::calibCamera()
{
	Sleep(3000);
	while (1)
	{
		Mat g[2];
		camera.capture2OpenCV();
		cvtColor(camera.capture_buf[0], g[0], CV_RGBA2GRAY);
		cvtColor(camera.capture_buf[1], g[1], CV_RGBA2GRAY);			
		if(calib.GetCorners_stereo(g[1],g[0]))
		{
			ui.textEdit->insertPlainText("camera calibration is done!\n");
			sprintf_s(filename, CALIB_PATH, "para");
			calib.savePara(filename);
			cvDestroyAllWindows();
		}
		char text[100];
		sprintf_s(text, "stereo_sub_%d.bmp", calib.stereo_success_count+100);
		imwrite(text, g[1]);
		sprintf_s(text, "stereo_main_%d.bmp", calib.stereo_success_count+100);
		imwrite(text, g[0]);

		sprintf_s(text, "%2d/%d\r", calib.stereo_success_count, calib.num_images);
		ui.textEdit->insertPlainText(text);
		if (calib.stereo_success_count == calib.num_images) break;
		Sleep(500);
	}

	updataCalibinform();
}
void MyClass::calibKinect()
{
	Mat cam, kinect;
	initKinectSystem();
	g_depth.Release();
	g_image.StartGenerating();
	while (1)
	{
		g_image.WaitAndUpdateData();
		const XnRGB24Pixel* pColorMap = g_image.GetRGB24ImageMap();
		Mat tem(XN_VGA_Y_RES, XN_VGA_X_RES, CV_8UC3, (uchar*)pColorMap);
		cvtColor(tem, kinect, CV_RGB2GRAY);

// 		imshow("a", kinect);
// 		char c = waitKey(10);
// 		if (c==27)
// 		{
// 			cvDestroyAllWindows();
// 			break;
// 		}

		camera.capture2OpenCV();		
		cvtColor(camera.capture_buf[0], cam, CV_RGBA2GRAY);
		
		if(calib_cam_knt.GetCorners_stereo(cam, kinect))
		{
			ui.textEdit->insertPlainText("camera calibration is done!\n");
			sprintf_s(filename, CALIB_PATH, "cam_kinect");
			calib_cam_knt.savePara(filename);
			cvDestroyAllWindows();
		}

		char text[100];
		sprintf_s(text, "stereo_kinect_%d.bmp", calib_cam_knt.stereo_success_count+100);
		imwrite(text, kinect);
		sprintf_s(text, "stereo_cam_%d.bmp", calib_cam_knt.stereo_success_count+100);
		imwrite(text, cam);
		sprintf_s(text, "%2d/%d\r", calib_cam_knt.stereo_success_count, calib_cam_knt.num_images);
		ui.textEdit->insertPlainText(text);
		if (calib_cam_knt.stereo_success_count == calib_cam_knt.num_images) break;
		Sleep(500);
	}

	g_image.Release();
	context.Release();

	updataCalibinform();
}
#endif

void MyClass::changeTab(int indx)
{
	if (indx==0)
	{
#ifdef HAVE_CAMERA
		// record setting
		kinectTime.clear();
		kinectTime.resize(kinectFrameMax);
		initKinectSystem();

		cam_imgs[0].clear();
		cam_imgs[1].clear();
		if (!camera.isCameraActive)
		{
			camera.InitCamera(ui.label_camera1->winId(), ui.label_camera2->winId());
		}
		
#endif
	}
	else if (indx==1)
	{
#ifdef HAVE_CAMERA
		camera.RelaeaseCamera();
#endif
		sliderCallback(0);
	}
}

void MyClass::updataCalibinform()
{
	Mat transform_cam_kinect(4,4,CV_64F);
	Mat transform_rgb_depth(4,4,CV_64F);
	Rect r_roi(0, 0, 3, 3);
	Rect t_roi(3, 0, 1, 3);
	Mat r_part, t_part;

	r_part = Mat(transform_cam_kinect, r_roi);
	t_part = Mat(transform_cam_kinect, t_roi);
	Mat(calib_cam_knt.R).copyTo(r_part);
	Mat(calib_cam_knt.T).copyTo(t_part);

	r_part = Mat(transform_rgb_depth, r_roi);
	t_part = Mat(transform_rgb_depth, t_roi);
	Mat(calib_kinect.R).copyTo(r_part);
	Mat(calib_kinect.T).copyTo(t_part);
	
	calib.toWorld = calib.toWorld*transform_cam_kinect;
}
bool MyClass::isInsideWidget( const QPoint& pt, const QPoint& pt_widget, const QSize& siz_widget)
{
	QPoint tem = pt - pt_widget;

	if (0<=tem.x() && tem.x()<siz_widget.width() && 0<=tem.y() && tem.y()<siz_widget.height())
	{
		return true;
	}
	else
	{
		return false;
	}
}
void MyClass::mousePressEvent( QMouseEvent* e )
{
	if (e->button()==Qt::MouseButton::RightButton)
	{
	}
}
void MyClass::mouseReleaseEvent( QMouseEvent* e )
{
	QPoint tem = e->pos();
	QPoint tem2	= ui.label_display1->mapTo(this,QPoint(0,0));
	QPoint tem3	= ui.label_display2->mapTo(this,QPoint(0,0));
	QPoint tem4	= ui.label_kinect2->mapTo(this,QPoint(0,0));

	isclicked1 = isInsideWidget(tem, tem2, ui.label_display1->size());
	isclicked2 = isInsideWidget(tem, tem3, ui.label_display2->size());
	isclicked3 = isInsideWidget(tem, tem4, ui.label_kinect2->size());

	if (e->button()==Qt::MouseButton::LeftButton)
	{
		if (isclicked1)
		{
			clickedPoint1 = (tem - tem2)*1/displayScale_camera;
			if (clickedPt_main.size()>max_pt-1) clickedPt_main.clear();
			clickedPt_main.push_back(Point2d(clickedPoint1.x(), clickedPoint1.y()));
		}
		else if (isclicked2)
		{
			clickedPoint2 = (tem - tem3)*1/displayScale_camera;			
			if (clickedPt_sub.size()>max_pt-1) clickedPt_sub.clear();
			clickedPt_sub.push_back(Point2d(clickedPoint2.x(), clickedPoint2.y()));
		}
		else if (isclicked3)
		{
			clickedPoint3 = (tem - tem4)*1/displayScale_kinect;
			if (clickedPt_depth.size()>max_pt-1) clickedPt_depth.clear();
			clickedPt_depth.push_back(Point2d(clickedPoint3.x(), clickedPoint3.y()));

			clickedindx  = clickedPoint3.x() + clickedPoint3.y()*XN_VGA_X_RES;
			if (clickedPt_kinect.size()>max_pt-1) clickedPt_kinect.clear();
			clickedPt_kinect.push_back(Point3d(npts[clickedindx].X, npts[clickedindx].Y, npts[clickedindx].Z));			
		}
	}
	if (e->button()==Qt::MouseButton::RightButton)
	{
		if (isclicked1)
		{
			if (!clickedPt_main.empty()) clickedPt_main.pop_back();			
		}
		else if (isclicked2)
		{
			if (!clickedPt_sub.empty()) clickedPt_sub.pop_back();
		}
		else if (isclicked3)
		{
			if (!clickedPt_depth.empty()) clickedPt_depth.pop_back();
			if (!clickedPt_kinect.empty()) clickedPt_kinect.pop_back();
		}
	}

	calc3D();
	sliderCallback(ui.horizontalSlider->value());
}
void MyClass::calc3D()
{
	main_origin = calib.cvtmain2world(Point3d(0,0,0));
	sub_origin = calib.cvtsub2world(Point3d(0,0,0));
	clickedPt_camera.clear();
	clickedPt_camera.resize(clickedPt_main.size());
	int n = min(clickedPt_main.size(), clickedPt_sub.size());
	for (int i = 0; i<n; ++i)
	{
		Point3d temp;
		double ex, ey, ez;
		temp = calib.calc_ref_MainCoord(clickedPt_sub[i],clickedPt_main[i],ex,ey,ez);
		clickedPt_camera[i] = calib.cvtmain2world(temp);

		ui.doubleSpinBox_x->setValue(ex);
		ui.doubleSpinBox_y->setValue(ey);
		ui.doubleSpinBox_z->setValue(ez);
	}
	clickedPt_camera = transform(trnsfrm, clickedPt_camera);
}
void MyClass::registration()
{
// 	saveclickedPt();
	clickedPt_camera.resize(9);
	clickedPt_kinect.resize(9);
	sprintf_s(filename, SAVE_CLICKED_PATH, "camera_pt");
	ifstream fin(filename);
	sprintf_s(filename, SAVE_CLICKED_PATH, "kinect_pt");
	ifstream fin2(filename);
	for (int i = 0; i<9; ++i)
	{
		fin>>clickedPt_camera[i].x>>clickedPt_camera[i].y>>clickedPt_camera[i].z;
		fin2>>clickedPt_kinect[i].x>>clickedPt_kinect[i].y>>clickedPt_kinect[i].z;
	}
	fin.close();
	fin2.clear();

	trnsfrm = Mat::eye(4,4,CV_64FC1);
	std::vector<Point3d> tempt = clickedPt_camera;
	for (int i = 0; i<5; ++i)
	{
		Mat tem = getAffine(tempt, clickedPt_kinect);
		tempt = transform(tem, tempt);
		trnsfrm = tem*trnsfrm;		
	}


	FileStorage fs("transformMatrix.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "M" << trnsfrm ;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";



	clickedPt_camera = transform(trnsfrm, clickedPt_camera);

	m_QGLViewer->pt1 = transform(trnsfrm, m_QGLViewer->pt1);
	m_QGLViewer->pt2 = transform(trnsfrm, m_QGLViewer->pt2);
	m_QGLViewer->pt1_smoothed = transform(trnsfrm, m_QGLViewer->pt1_smoothed);
	m_QGLViewer->pt2_smoothed = transform(trnsfrm, m_QGLViewer->pt2_smoothed);

	clickedPt_camera.clear();
	clickedPt_kinect.clear();
	m_QGLViewer->updateGL();
}
void MyClass::saveclickedPt()
{
	sprintf_s(filename, SAVE_CLICKED_PATH, "camera_pt");
	ofstream fout(filename);
	sprintf_s(filename, SAVE_CLICKED_PATH, "kinect_pt");
	ofstream fout2(filename);
	for (int i = 0; i<4; ++i)
	{
		fout<<clickedPt_camera[i].x<<"\t"<<clickedPt_camera[i].y<<"\t"<<clickedPt_camera[i].z<<endl;
		fout2<<clickedPt_kinect[i].x<<"\t"<<clickedPt_kinect[i].y<<"\t"<<clickedPt_kinect[i].z<<endl;
	}
	fout.close();
	fout2.clear();
}

std::vector<cv::Point3d> MyClass::transform( const cv::Mat& affine, std::vector<cv::Point3d>& pts )
{
	vector<Point3d> npts(pts.size());
	for (int i = 0; i<pts.size(); ++i)
	{
		Mat t(4,1,CV_64FC1);
		t.at<double>(0,0) = pts[i].x;
		t.at<double>(1,0) = pts[i].y;
		t.at<double>(2,0) = pts[i].z;
		t.at<double>(3,0) = 1;

		Mat t2 = affine*t;

		npts[i].x = t2.at<double>(0,0);
		npts[i].y = t2.at<double>(1,0);
		npts[i].z = t2.at<double>(2,0);
	}

	return npts;
}
std::vector<cv::Point3f> MyClass::transform( const cv::Mat& affine, std::vector<cv::Point3f>& pts )
{
	vector<Point3f> npts(pts.size());
	for (int i = 0; i<pts.size(); ++i)
	{
		Mat t(4,1,CV_64FC1);
		t.at<double>(0,0) = pts[i].x;
		t.at<double>(1,0) = pts[i].y;
		t.at<double>(2,0) = pts[i].z;
		t.at<double>(3,0) = 1;

		Mat t2 = affine*t;

		npts[i].x = t2.at<double>(0,0);
		npts[i].y = t2.at<double>(1,0);
		npts[i].z = t2.at<double>(2,0);
	}
	return npts;
}
cv::Mat MyClass::getAffine( const std::vector<cv::Point3d>& pp, const std::vector<cv::Point3d>& qq )
{
	if (pp.size()!=qq.size() || pp.empty())
	{
		return Mat::eye(4,4,CV_64FC1);
	}

	// registration p -> q
	int n = pp.size();
	Mat p = Mat(pp).clone();
	Mat q = Mat(qq).clone();

	Scalar p_center = mean(p);
	Scalar q_center = mean(q);

	Mat p_ = p - p_center;
	Mat q_ = q - q_center;

	Mat H(3,3, CV_64FC1, Scalar());
	for (int i = 0; i<n; ++i)
	{
		Mat tem = p_.row(i).reshape(1).t()*q_.row(i).reshape(1);
		H += p_.row(i).reshape(1).t()*q_.row(i).reshape(1);
	}

	cv::SVD svd(H.clone());

	Mat R = svd.vt.t()*svd.u.t();

	double det = cv::determinant(R);

	if (abs(det+1)<0.0001)
	{
		Mat tem_w = Mat::eye(3,3,CV_64FC1);
		tem_w.at<double>(2,2) = -1;
		R = svd.vt.t()*tem_w*svd.u.t();
	}

	Mat T1 = Mat(1,1, CV_64FC3, -p_center).reshape(1).t();
	Mat T2 = Mat(1,1, CV_64FC3, q_center).reshape(1).t();
	Mat T = R*T1 + T2;

	Mat affT = Mat::eye(4,4, CV_64FC1);
	R.copyTo(Mat(affT, Rect(0,0,3,3)));
	T.copyTo(Mat(affT, Rect(3,0,1,3)));

	return affT;
}

void MyClass::setCalib( Size calibsiz, float squaresiz, int numimgs )
{
	calib.initCalib(calibsiz, squaresiz, numimgs);
	calib_kinect.initCalib();
	calib_cam_knt.initCalib(calibsiz, squaresiz, numimgs);

	sprintf_s(filename, CALIB_PATH, "kinect");
	calib_kinect.loadPara(filename);
	sprintf_s(filename, CALIB_PATH, "camera_intrinsic");
	calib.loadIntrinsicPara(filename);


	sprintf_s(filename, CALIB_PATH, "para");
	calib.loadPara(filename);
	if (calib.T.at<double>(0)==0)
	{
		ui.textEdit->insertPlainText("need to calibrate two cameras.\n");
	}

	sprintf_s(filename, CALIB_PATH, "cam_kinect");
	calib_cam_knt.loadPara(filename);	
	if (calib_cam_knt.T.at<double>(0)==0)
	{
		ui.textEdit->insertPlainText("need to calibrate with kinect.\n");
		calib_cam_knt.cameraMatrix[0] = calib.cameraMatrix[1].clone();
		calib_cam_knt.cameraMatrix[1] = calib_kinect.cameraMatrix[0].clone();
		calib_cam_knt.distCoeffs[0] = calib.distCoeffs[1].clone();
		calib_cam_knt.distCoeffs[1] = calib_kinect.distCoeffs[0].clone();
	}
	else
	{
		updataCalibinform();		
	}
}

cv::Point2d MyClass::reproject( const cv::Point3d& pt3d, const CalibClass& calib )
{
// 	// pt3d befor registration
// 	Point3d pt3d2;
// 
// 	Mat t(4,1,CV_64FC1);
// 	t.at<double>(0,0) = pt3d.x;
// 	t.at<double>(1,0) = pt3d.y;
// 	t.at<double>(2,0) = pt3d.z;
// 	t.at<double>(3,0) = 1;
// 
// 	Mat t2 = trnsfrm.inv()*t;
// 
// 	pt3d2.x = t2.at<double>(0,0);
// 	pt3d2.y = t2.at<double>(1,0);
// 	pt3d2.z = t2.at<double>(2,0);
// 
// 
// 	calib.
	return Point2d();
}

void MyClass::hide_show_kinect()
{
	if (ui.groupBox_kinect->isHidden())
	{
		ui.groupBox_kinect->show();		
	}
	else
	{
		ui.groupBox_kinect->hide();
	}
	this->adjustSize();
}

void MyClass::hide_show_3D()
{
	if (m_QGLViewer->isHidden() && ( kinect_player.IsValid() || tracker.isTrackingFinished3d ) )
	{
		m_QGLViewer->show();		
	}
	else
	{
		m_QGLViewer->hide();
	}
	this->adjustSize();
}

void MyClass::setGraphData()
{
	headup.clear();
	headdown.clear();
	sheadup.clear();
	sheaddown.clear();
	dirup.clear();
	dirdown.clear();
	sdirup.clear();
	sdirdown.clear();
	for(int i = 0; i < tracker.num_frame; ++i)
	{
		if (tracker.isEmpty_indx[i]==1)
		{
			headup.push_back(tracker.pt2[i]);
			dirup.push_back(tracker.dir[i]);
			sheadup.push_back(tracker.pt2_smoothed[i]);
			sdirup.push_back(tracker.dir_smoothed[i]);
		}
		if (tracker.isEmpty_indx[i]==2)
		{
			headdown.push_back(tracker.pt2[i]);
			dirdown.push_back(tracker.dir[i]);
			sheaddown.push_back(tracker.pt2_smoothed[i]);
			sdirdown.push_back(tracker.dir_smoothed[i]);
		}
	}
}

void MyClass::setGraph(int f)
{
	Mat head_data[3], shead_data[3], dir_data[3], sdir_data[3];
	int curf;
	double dx;

	if(tracker.isEmpty_indx[f]==0) return;
	else if (tracker.isEmpty_indx[f]==1)
	{
		split(Mat(headup), head_data);
		split(Mat(sheadup), shead_data);
		split(Mat(dirup), dir_data);
		split(Mat(sdirup), sdir_data);
		dx = double(graph_w)/double(tracker.up_end - tracker.up_start+1);
		curf = f-tracker.nonEmpty_indx[tracker.up_start];
	}
	else if (tracker.isEmpty_indx[f]==2)
	{
		split(Mat(headdown), head_data);
		split(Mat(sheaddown), shead_data);
		split(Mat(dirdown), dir_data);
		split(Mat(sdirdown), sdir_data);
		dx = double(graph_w)/double(tracker.down_end - tracker.down_start+1);
		curf = f-tracker.nonEmpty_indx[tracker.down_start];
	}

	Scalar c1(0,0,255);
	Scalar c2(255,0,0);

	for (int k = 0; k<3; ++k)
	{
		head_graph[k] = Mat(graph_h, graph_w, CV_8UC3, Scalar(255,255,255));
		dir_graph[k] = Mat(graph_h, graph_w, CV_8UC3, Scalar(255,255,255));
		
		if (tracker.isEmpty_indx[f]==1)
		{
			for (unsigned i = 0; i<tracker.outlier_up_indx.size(); ++i)
			{
				int indx = tracker.outlier_up_indx[i]-tracker.nonEmpty_indx[tracker.up_start];
				Point2f p1(dx*indx, 0), p2(dx*indx, graph_h);
				line(head_graph[k], p1, p2, Scalar(0,255,255),2);
				line(dir_graph[k], p1, p2, Scalar(0,255,255),2);
			}
		} 
		else
		{
			for (unsigned i = 0; i<tracker.outlier_down_indx.size(); ++i)
			{
				int indx = tracker.outlier_down_indx[i]-tracker.nonEmpty_indx[tracker.down_start];
				Point2f p1(dx*indx, 0), p2(dx*indx, graph_h);
				line(head_graph[k], p1, p2, Scalar(0,255,255),2);
				line(dir_graph[k], p1, p2, Scalar(0,255,255),2);
			}
		}

// 		if (ui.checkBox_graphorigin->isChecked())
// 		{
// 			drawGraph(head_graph[k], head_data[k], c1);
// 			drawGraph(dir_graph[k], dir_data[k], c1);
// 		}
// 		if (ui.checkBox_graphsmooth->isChecked())
// 		{
// 			drawGraph(head_graph[k], shead_data[k], c2);
// 			drawGraph(dir_graph[k], sdir_data[k], c2);
// 		}
		drawGraph(head_graph[k], head_data[k], shead_data[k]);
		drawGraph(dir_graph[k], dir_data[k], sdir_data[k]);

		Point2f p1(dx*curf, 0), p2(dx*curf, graph_h);
		line(head_graph[k], p1, p2, Scalar(0,255,0));
		line(dir_graph[k], p1, p2, Scalar(0,255,0));
	}
}

void MyClass::drawGraph( cv::Mat& canvas, const cv::Mat& data, const cv::Scalar clr)
{
	int num_data = data.rows;
	int w = canvas.cols;
	int h = canvas.rows;

	double minval, maxval;
	minMaxLoc(data, &minval, &maxval);


	double dx = double(w)/double(num_data);
	double dy = h*0.8/(maxval-minval);

	vector<Point2f> pt(num_data);
	for (int i = 0; i<num_data; ++i)
	{		
		double d = data.at<float>(i,0);
		pt[i] = Point2f(i*dx, h*0.9-((d-minval)*dy));
		circle(canvas, pt[i], 1, clr, 2);
	}


	for (int i = 1; i<num_data; ++i)
	{		
		line(canvas, pt[i-1], pt[i], clr);
	}

}

void MyClass::drawGraph( cv::Mat& canvas, const cv::Mat& data, const cv::Mat& sdata)
{
	int num_data = data.rows;
	int w = canvas.cols;
	int h = canvas.rows;

	double minval, maxval;
	minMaxLoc(data, &minval, &maxval);

	double dx = double(w)/double(num_data);
	double dy = h*0.8/(maxval-minval);

	vector<Point2f> pt(num_data);
	vector<Point2f> spt(num_data);
	for (int i = 0; i<num_data; ++i)
	{		
		double d = data.at<float>(i,0);
		double sd = sdata.at<float>(i,0);
		pt[i] = Point2f(i*dx, h*0.9-((d-minval)*dy));
		spt[i] = Point2f(i*dx, h*0.9-((sd-minval)*dy));

		circle(canvas, pt[i], 1, Scalar(0,0,255), 2);
		circle(canvas, spt[i], 1, Scalar(255,0,0), 2);
	}


	for (int i = 1; i<num_data; ++i)
	{		
		line(canvas, pt[i-1], pt[i], Scalar(0,0,255), 2);
		line(canvas, spt[i-1], spt[i], Scalar(255,0,0), 2);
	}

}

void MyClass::loaddata()
{
	if (num_frame == 0)
	{
		QMessageBox msgBox;
		msgBox.setText("Load images.");
		msgBox.exec();
		return;
	}
	if (calib.distCoeffs[0].at<double>(0) == 0)
	{
		QMessageBox msgBox;
		msgBox.setText("Load calib parameters.");
		msgBox.exec();
		return;
	}

	QStringList files = QFileDialog::getOpenFileNames(this,"Select one or more files to open", "", "data (*.txt)");

	if (files.size()!=8) return;
	else
	{		
		inputData(tracker.dir, files[0].toStdString().c_str());
		inputData(tracker.pt2, files[1].toStdString().c_str());
		inputData(tracker.main_pt1_2d , files[2].toStdString().c_str());
		inputData(tracker.main_pt2_2d, files[3].toStdString().c_str());
		inputData(tracker.dir_smoothed, files[4].toStdString().c_str());
		inputData(tracker.pt2_smoothed, files[5].toStdString().c_str());
		inputData(tracker.sub_pt1_2d, files[6].toStdString().c_str());
		inputData(tracker.sub_pt2_2d, files[7].toStdString().c_str());
		tracker.isTrackingFinished2d = true;
		tracker.isTrackingFinished3d= true;

		m_label_optimize->hide();
		m_QGLViewer->show();

		ui.horizontalSlider->setValue(0);
		sliderCallback(0);
		this->adjustSize();
		setGraphData();
	}
}

void MyClass::vsliderCallback( int f )
{
	if (!tracker.isTrackingFinished3d) return;

	int cur_frame = ui.horizontalSlider->value();

	double t = f*0.01;

	tracker.main_pt2_2d[cur_frame].y += t;
	tracker.main_pt1_2d[cur_frame].y -= t;
	tracker.Tracking3D(calib);

	setGraphData();
	tracker.dadtaUpdate();
	sliderCallback(ui.horizontalSlider->value());
}

void MyClass::smoothingset()
{
	if (!tracker.isTrackingFinished3d) return;

	int cur_frame = ui.horizontalSlider->value();

	double t = ui.verticalScrollBar_edit->value()*0.01;
	tracker.main_lines[cur_frame][3] += t;
	tracker.main_lines[cur_frame][1] -= t;
	tracker.dadtaUpdate();
	tracker.Tracking3D(calib);
	setGraphData();

	ui.verticalScrollBar_edit->setValue(0);

	sliderCallback(ui.horizontalSlider->value());
}

void MyClass::hide_show_graph()
{
	if (ui.groupBox_graph->isHidden())
	{
		ui.groupBox_graph->show();
	}
	else
	{
		ui.groupBox_graph->hide();
	}

	this->adjustSize();	
}
