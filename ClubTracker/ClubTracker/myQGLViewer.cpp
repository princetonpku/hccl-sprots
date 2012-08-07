#include "myQGLViewer.h"
#include "MyClass.h"
#include "Imgprocessing.h"

using namespace cv;

myQGLViewer::myQGLViewer(QWidget *parent) : QGLViewer(parent), frame(0)
	, isDrawCamer(true), isDrawClub(true), isDrawEnd_trjtory(true), isDrawShaft_trjtory(false)
	, isDrawUpswingplane(false), isDrawDownswingplane(false)
	, isDrawSmooth(false)
{

// 	qglviewer::Quaternion a(qglviewer::Vec(), 0);
// 	qglviewer::Vec tem(); 
}

void drawSphere(float rad, int lon_seg = 10, int lat_seg = 5)
{
	const float PI = acos(-1.0);

	float lon_step = 2*PI/lon_seg;
	float lat_step = PI/lat_seg;

	// Draw upper face
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, 1.0);
	glVertex3d(0.0, 0.0, rad);
	for(register int i = 0; i <= lon_seg; i++)
	{
		glNormal3d(cos(lon_step*i)*sin(lat_step), sin(lon_step*i)*sin(lat_step), cos(lat_step));
		glVertex3d(rad*cos(lon_step*i)*sin(lat_step), rad*sin(lon_step*i)*sin(lat_step), rad*cos(lat_step));
	}
	glEnd();

	// Draw bottom face
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(0.0, 0.0, -rad);
	for(register int i = lon_seg; i >= 0; i--)
	{
		glNormal3d(cos(lon_step*i)*sin(lat_step), sin(lon_step*i)*sin(lat_step), cos(lat_step*(lat_seg-1)));
		glVertex3d(rad*cos(lon_step*i)*sin(lat_step), rad*sin(lon_step*i)*sin(lat_step), rad*cos(lat_step*(lat_seg-1)));
	}
	glEnd();

	// Draw side face
	for(int j = 2; j <= lat_seg-1; j++)
	{
		glBegin(GL_QUAD_STRIP);
		// 		glNormal3d(sin(lat_step*(j-1)), 0.0, cos(lat_step*(j-1)));
		// 		glVertex3d(rad*sin(lat_step*(j-1)), 0.0, rad*cos(lat_step*(j-1)));
		// 		glNormal3d(sin(lat_step*j), 0.0, cos(lat_step*j));
		// 		glVertex3d(rad*sin(lat_step*j), 0.0, rad*cos(lat_step*j));
		for(register int i = 0; i <= lon_seg; i++)
		{
			glNormal3d(cos(lon_step*i)*sin(lat_step*(j-1)), sin(lon_step*i)*sin(lat_step*(j-1)), cos(lat_step*(j-1)));
			glVertex3d(rad*cos(lon_step*i)*sin(lat_step*(j-1)), rad*sin(lon_step*i)*sin(lat_step*(j-1)), rad*cos(lat_step*(j-1)));
			glNormal3d(cos(lon_step*i)*sin(lat_step*j), sin(lon_step*i)*sin(lat_step*j), cos(lat_step*j));
			glVertex3d(rad*cos(lon_step*i)*sin(lat_step*j), rad*sin(lon_step*i)*sin(lat_step*j), rad*cos(lat_step*j));
		}
		glEnd();
	}

}
void drawSpherePT( const cv::Point3f pt, const float r )
{
	glPushMatrix();
	glTranslated(pt.x, pt.y, pt.z);
	drawSphere(r);
	glPopMatrix();
}

void myQGLViewer::draw()
{
// 	drawAxis(1200);

	if (parent->tracker.isTrackingFinished3d)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		if (parent->tracker.isTrackingFinished2d)
		{
			if (isDrawCamer)
			{
				drawCamera(frame);
				drawPlane(frame);
			}				
			if (isDrawClub)drawClub(frame);
			if (isDrawEnd_trjtory)drawClubTrajectory(frame);
			if (isDrawShaft_trjtory)drawShaftTrajectory(frame);
			if (isDrawUpswingplane)drawUpswingPlane();
			if (isDrawDownswingplane)drawDownswingPlane();
			if (isDrawSmooth)drawSmooth(frame);		

		}
		glDisable(GL_BLEND);
		glPopAttrib();
	}

	int thr = 5000;

	glDisable(GL_LIGHTING);
	glPointSize(1.0);
	glBegin(GL_POINTS);
	for(int i = 0; i < parent->npts.size(); i++)
	{
		glColor3d(0.5,0.5,0.5);
		if(parent->npts[i].Z<thr)glVertex3d(parent->npts[i].X, parent->npts[i].Y, parent->npts[i].Z);
	}
	glEnd();
	glEnable(GL_LIGHTING);


	if (1)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
	
		glColor3d(1,0,0);
		for (int i = 0; i<parent->clickedPt_camera.size(); ++i)
		{
			drawSpherePT(parent->clickedPt_camera[i], 30);
		}	
	
		if (parent->npts.size()>0)
		{
			glColor3d(0,0,1);			
			for (int i = 0; i<parent->clickedPt_kinect.size(); ++i)
			{
				drawSpherePT(parent->clickedPt_kinect[i], 30);
			}			
		}
	
		glPopAttrib();
	}

	if(0)
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		cv::Point3d m_origin  = parent->calib.cvtmain2world(cv::Point3d(0,0,0));
		cv::Point3d s_origin  = parent->calib.cvtsub2world(cv::Point3d(0,0,0));
		glColor3d(1,0,0);
		drawSpherePT(m_origin, 30);
		glColor3d(0,1,0);
		drawSpherePT(s_origin, 30);

		cv::Point3d tem_m = parent->calib.getDir(cv::Point2d(parent->clickedPoint1.x(), parent->clickedPoint1.y()), 0);
		cv::Point3d tem_s = parent->calib.getDir(cv::Point2d(parent->clickedPoint2.x(), parent->clickedPoint2.y()), 1);
		cv::Point3d dir_m = tem_m-m_origin;
		cv::Point3d dir_s = tem_s-s_origin;
		cv::Point3d ptm = m_origin + 10000*dir_m;
		cv::Point3d pts = s_origin + 10000*dir_s;
		glBegin(GL_LINES);
		glColor3d(1,0,0);
		glVertex3d(m_origin.x, m_origin.y, m_origin.z);
		glVertex3d(ptm.x, ptm.y, ptm.z);
		glColor3d(0,1,0);
		glVertex3d(s_origin.x, s_origin.y, s_origin.z);
		glVertex3d(pts.x, pts.y, pts.z);
		glEnd();
		glPopAttrib();
	}
}

void myQGLViewer::init()
{
	setSceneRadius(10000);

	camera()->setPosition(qglviewer::Vec(0,0,-3000));
	camera()->lookAt(qglviewer::Vec(0,0,1));
	camera()->setUpVector(qglviewer::Vec(0,1,0));
	
	
// 	camera()->setPosition(qglviewer::Vec(8000,2000,500));
// 	camera()->lookAt(qglviewer::Vec(0,0,0));
// 	camera()->setUpVector(qglviewer::Vec(0,0,1));

 	setBackgroundColor(QColor(255,255,255));

//     restoreStateFromFile();
}
void myQGLViewer::setCamerCorner()
{
	Point3f t_rect[4];
	t_rect[0] = Point3f(-parent->tracker.imgsize.width*0.5, -parent->tracker.imgsize.height*0.5, 0);
	t_rect[1] = Point3f(-parent->tracker.imgsize.width*0.5, parent->tracker.imgsize.height*0.5, 0);
	t_rect[2] = Point3f(parent->tracker.imgsize.width*0.5, parent->tracker.imgsize.height*0.5, 0);
	t_rect[3] = Point3f(parent->tracker.imgsize.width*0.5, -parent->tracker.imgsize.height*0.5, 0);
	for(int i = 0; i < 4; ++i)
	{
		m_rect[0][i] = parent->calib.cvtmain2world(t_rect[i]);
		m_rect[1][i] = parent->calib.cvtsub2world(t_rect[i]);
	}
}
void myQGLViewer::drawCamera(int frame)
{
	glDisable(GL_LIGHTING);
	glLineWidth(3);

	// draw main camera
	glColor3d(1.0,0,0);
	glBegin(GL_LINE_STRIP);
	glVertex3d(m_rect[0][0].x, m_rect[0][0].y, m_rect[0][0].z);
	glVertex3d(m_rect[0][1].x, m_rect[0][1].y, m_rect[0][1].z);
	glVertex3d(m_rect[0][2].x, m_rect[0][2].y, m_rect[0][2].z);
	glVertex3d(m_rect[0][3].x, m_rect[0][3].y, m_rect[0][3].z);
	glVertex3d(m_rect[0][0].x, m_rect[0][0].y, m_rect[0][0].z);
	glEnd();

	glBegin(GL_LINES);
	glVertex3d(m_rect[0][0].x, m_rect[0][0].y, m_rect[0][0].z);
	glVertex3d(main_pt0.x, main_pt0.y, main_pt0.z);
	glVertex3d(m_rect[0][1].x, m_rect[0][1].y, m_rect[0][1].z);
	glVertex3d(main_pt0.x, main_pt0.y, main_pt0.z);
	glVertex3d(m_rect[0][2].x, m_rect[0][2].y, m_rect[0][2].z);
	glVertex3d(main_pt0.x, main_pt0.y, main_pt0.z);
	glVertex3d(m_rect[0][3].x, m_rect[0][3].y, m_rect[0][3].z);
	glVertex3d(main_pt0.x, main_pt0.y, main_pt0.z);
	glEnd();

	// draw sub camera
	glColor3d(0,1.0,0);
	glBegin(GL_LINE_STRIP);
	glVertex3d(m_rect[1][0].x, m_rect[1][0].y, m_rect[1][0].z);
	glVertex3d(m_rect[1][1].x, m_rect[1][1].y, m_rect[1][1].z);
	glVertex3d(m_rect[1][2].x, m_rect[1][2].y, m_rect[1][2].z);
	glVertex3d(m_rect[1][3].x, m_rect[1][3].y, m_rect[1][3].z);
	glVertex3d(m_rect[1][0].x, m_rect[1][0].y, m_rect[1][0].z);
	glEnd(); 

	glBegin(GL_LINES);
	glVertex3d(m_rect[1][0].x, m_rect[1][0].y, m_rect[1][0].z);
	glVertex3d(sub_pt0.x, sub_pt0.y, sub_pt0.z);
	glVertex3d(m_rect[1][1].x, m_rect[1][1].y, m_rect[1][1].z);
	glVertex3d(sub_pt0.x, sub_pt0.y, sub_pt0.z);
	glVertex3d(m_rect[1][2].x, m_rect[1][2].y, m_rect[1][2].z);
	glVertex3d(sub_pt0.x, sub_pt0.y, sub_pt0.z);
	glVertex3d(m_rect[1][3].x, m_rect[1][3].y, m_rect[1][3].z);
	glVertex3d(sub_pt0.x, sub_pt0.y, sub_pt0.z);
	glEnd();

	glEnable(GL_LIGHTING);

	// draw point of images
	glColor3d(1,0,0);
	drawSpherePT(main_pt0, 15);
	drawSpherePT(main_pt1[frame], 15);
	drawSpherePT(main_pt2[frame], 15);

	glColor3d(0,1.0,0);
	drawSpherePT(sub_pt0, 15);
	drawSpherePT(sub_pt1[frame], 15);
	drawSpherePT(sub_pt2[frame], 15);
}
void myQGLViewer::drawClub(int frame)
{
	// draw club of current frame
	glColor3d(0,0,1);
	drawSpherePT(pt1[frame], 30);
	glColor3d(0,0,1);
	drawSpherePT(pt2[frame], 30);
	glLineWidth(2);
	glDisable(GL_LIGHTING);
	glColor3d(1.0, 0.5, 0.5);
	glBegin(GL_LINES);
	glVertex3d(pt1[frame].x, pt1[frame].y, pt1[frame].z);
	glVertex3d(pt2[frame].x, pt2[frame].y, pt2[frame].z);
	glEnd();
	glEnable(GL_LIGHTING);
}
void myQGLViewer::drawClubTrajectory( int frame )
{
	int indx_pre = 0;

	// draw trajectory of pt1
	glLineWidth(2);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);	
	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (indx_pre)
			{				
				if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
				else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
				glVertex3d(pt2[indx_pre].x, pt2[indx_pre].y, pt2[indx_pre].z);
				glVertex3d(pt2[i].x, pt2[i].y, pt2[i].z);				
			}
			indx_pre = i;
		}
	}
	indx_pre = 0;
	glEnd();
	glEnable(GL_LIGHTING);

	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
			else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
			glColor3d(1.0,0,0);
			drawSpherePT(pt1[i], 15);
			drawSpherePT(pt2[i], 15);
		}
	}

}
void myQGLViewer::drawShaftTrajectory( int frame )
{
	glLineWidth(2);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);	
	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
			else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
			glVertex3d(pt1[i].x, pt1[i].y, pt1[i].z);				
			glVertex3d(pt2[i].x, pt2[i].y, pt2[i].z);				
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
}
void myQGLViewer::drawSmooth( int frame )
{
	int indx_pre = 0;

	// draw trajectory of pt1
	glLineWidth(4);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);	
	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (indx_pre)
			{				
				if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
				else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
				glVertex3d(pt2_smoothed[indx_pre].x, pt2_smoothed[indx_pre].y, pt2_smoothed[indx_pre].z);
				glVertex3d(pt2_smoothed[i].x, pt2_smoothed[i].y, pt2_smoothed[i].z);				
			}
			indx_pre = i;
		}
	}
	indx_pre = 0;
	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (indx_pre)
			{				
				if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
				else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
				glVertex3d(pt1_smoothed[indx_pre].x, pt1_smoothed[indx_pre].y, pt1_smoothed[indx_pre].z);
				glVertex3d(pt1_smoothed[i].x, pt1_smoothed[i].y, pt1_smoothed[i].z);				
			}
			indx_pre = i;
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);

	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
			else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
			glColor3d(1.0,1.0,0);
			drawSpherePT(pt1_smoothed[i], 20);
			drawSpherePT(pt2_smoothed[i], 20);
		}
	}

	glLineWidth(4);
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);	
	for (int i = 0; i<=frame; ++i)
	{
		if (isEmpty_indx[i])
		{
			if (nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end]) glColor3d(0, 1, 0.8);
			else if(nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end]) glColor3d(1, 0.8, 0);
			glVertex3d(pt1_smoothed[i].x, pt1_smoothed[i].y, pt1_smoothed[i].z);				
			glVertex3d(pt2_smoothed[i].x, pt2_smoothed[i].y, pt2_smoothed[i].z);				
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);

}
void myQGLViewer::drawUpswingPlane()
{
// 	glColor4d(0, 1, 0.8, 0.2);
// 	Point3f pro;
// 	glDisable(GL_LIGHTING);
// 	glBegin(GL_TRIANGLE_STRIP);
// 	glNormal3d(-plane_upswing[0],-plane_upswing[1], -plane_upswing[2]);
// 	for(int i = 0; i < num_frame; ++i)
// 	{
// 		if (isEmpty_indx[i] && nonEmpty_indx[up_start]<=i && i<=nonEmpty_indx[up_end])
// 		{
// 			pro = PointProjection(pt2[i], plane_upswing);
// 			glVertex3d(pro.x, pro.y, pro.z);
// 			pro = PointProjection(pt1[i], plane_upswing);
// 			glVertex3d(pro.x, pro.y, pro.z);
// 		}
// 	}
// 	glEnd();
// 	glEnable(GL_LIGHTING);
}
void myQGLViewer::drawDownswingPlane()
{
// 	glColor4d(1, 0.8, 0, 0.2);
// 	Point3f pro;
// 	glDisable(GL_LIGHTING);
// 	glBegin(GL_TRIANGLE_STRIP);
// 	glNormal3d(-plane_downswing[0],-plane_downswing[1], -plane_downswing[2]);
// 	for(int i = 0; i < num_frame; ++i)
// 	{
// 		if (isEmpty_indx[i] && nonEmpty_indx[down_start]<=i && i<=nonEmpty_indx[down_end])
// 		{
// 			pro = PointProjection(pt2[i], plane_downswing);
// 			glVertex3d(pro.x, pro.y, pro.z);
// 			pro = PointProjection(pt1[i], plane_downswing);
// 			glVertex3d(pro.x, pro.y, pro.z);
// 		}
// 	}
// 	glEnd();
// 	glEnable(GL_LIGHTING);
}
void myQGLViewer::drawPlane( int frame )
{	
	int len = norm(main_pt0-pt1[frame])*1.2;
	glColor4d(1, 0, 0, 0.2);
	drawPlane(main_pt0, main_pt1[frame], main_pt2[frame], len);
	glColor4d(0, 0, 1, 0.2);
	drawPlane(sub_pt0, sub_pt1[frame], sub_pt2[frame], len);
}
void myQGLViewer::drawPlane( cv::Point3f p0, cv::Point3f p1, cv::Point3f p2, int len )
{
	Point3f temp1, temp2, dir1, dir2;

	dir1 = p1-p0;Normalize(dir1);
	dir2 = p2-p0;Normalize(dir2);

	temp1 = p0+len*(dir1);
	temp2 = p0+len*(dir2);

	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);	
	glVertex3d(p0.x, p0.y, p0.z);
	glVertex3d(temp1.x, temp1.y, temp1.z);
	glVertex3d(temp2.x, temp2.y, temp2.z);
	glEnd();
	glEnable(GL_LIGHTING);
}
