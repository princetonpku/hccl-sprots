#include "glPrimitives.h"

#ifdef QT_OPENGL_LIB
#include <qgl.h>
#else
#include <gl/GL.h>
#include <gl/GLU.h>
#endif

void glNormalv(Vector3d vec) { glNormal3d(vec[0], vec[1], vec[2]); }
void glTranslatev(Vector3d vec) { glTranslated(vec[0], vec[1], vec[2]); }
void glRotatev(double ang, Vector3d vec) { glRotated(ang, vec[0], vec[1], vec[2]); }
void glVertexv(Vector3d vec) { glVertex3d(vec[0], vec[1], vec[2]); }

void DrawEllipse(double r1, double r2, unsigned int segments/* = 10*/)
{
	double step = 360.0/segments;

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, 1.0);
	glVertex3d(0.0, 0.0, 0.0);
	for(double ang = 0.0; ang <= 360.0; ang += step)
		glVertex3d(r1*cos(ang*DEG2RAD), r2*sin(ang*DEG2RAD), 0.0);
	glEnd();
}

void DrawArrow(double length, double thickness)
{
	glPushMatrix();
	DrawCylinder(thickness, length);
	glTranslated(0, 0, length);
	DrawCone(thickness*2, thickness*3);
	glPopMatrix();
}

void DrawAxis(Vector3d length)
{
	Vector3d vStep = length;

	// x-axis
	glPushMatrix();
	glRotatef(90, 0, 1, 0);
	glColor3d(0.8, 0.3, 0.2);
	DrawCylinder(vStep[0]*0.09, vStep[0]);
	glTranslated(0, 0, vStep[0]);
	DrawCone(vStep[0]*0.15, vStep[0]*0.3);
	glPopMatrix();
	// y-axis
	glPushMatrix();
	glRotatef(-90, 1, 0, 0);
	glColor3d(0.3, 0.7, 0.2);
	DrawCylinder(vStep[1]*0.09, vStep[1]);
	glTranslated(0, 0, vStep[1]);
	DrawCone(vStep[1]*0.15, vStep[1]*0.3);
	glPopMatrix();
	// z-axis
	glPushMatrix();
	glColor3d(0.2, 0.4, 0.8);
	DrawCylinder(vStep[2]*0.09, vStep[2]);
	glTranslated(0, 0, vStep[2]);
	DrawCone(vStep[2]*0.15, vStep[2]*0.3);
	glPopMatrix();
}


void DrawGrid( float fExtent, float fStep )
{
	// Grid Lines
	glBegin(GL_LINES);
	for(GLfloat i = -fExtent; i <= fExtent + 0.1; i += fStep)
	{
		glVertex3f(i, 0, fExtent);
		glVertex3f(i, 0, -fExtent);
		glVertex3f(fExtent, 0, i);
		glVertex3f(-fExtent, 0, i);
	}
	glEnd();

}

void DrawCylinder(double r, double height, unsigned int segments/* = 8*/)
{
	double step = 360.0/segments;

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(0.0, 0.0, 0.0);
	for(double ang = 360.0; ang >= 0.0; ang -= step)
		glVertex3d(r*cos(ang*DEG2RAD), r*sin(ang*DEG2RAD), 0.0);
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0, 0, 1);
	glVertex3d(0.0, 0.0, height);
	for(double ang = 0.0; ang <= 360.0; ang += step)
		glVertex3d(r*cos(ang*DEG2RAD), r*sin(ang*DEG2RAD), height);
	glEnd();

	glBegin(GL_QUAD_STRIP);
	for(double ang = 0.0; ang <= 360.0; ang += step)
	{
		glNormal3d(cos(ang*DEG2RAD), sin(ang*DEG2RAD), 0);
		glVertex3d(r*cos(ang*DEG2RAD), r*sin(ang*DEG2RAD), height);
		glVertex3d(r*cos(ang*DEG2RAD), r*sin(ang*DEG2RAD), 0.0);
	}
	glEnd();
}

void DrawEllipCylinder(double r1, double r2, double height, unsigned int segments/* = 8*/)
{
	double step = 360.0/segments;

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(0.0, 0.0, 0.0);
	for(double ang = 360.0; ang >= 0.0; ang -= step)
		glVertex3d(r1*cos(ang*DEG2RAD), r2*sin(ang*DEG2RAD), 0.0);
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0, 0, 1);
	glVertex3d(0.0, 0.0, height);
	for(double ang = 0.0; ang <= 360.0; ang += step)
		glVertex3d(r1*cos(ang*DEG2RAD), r2*sin(ang*DEG2RAD), height);
	glEnd();

	glBegin(GL_QUAD_STRIP);
	for(double ang = 0.0; ang <= 360.0; ang += step)
	{
		glNormal3d(cos(ang*DEG2RAD), sin(ang*DEG2RAD), 0);
		glVertex3d(r1*cos(ang*DEG2RAD), r2*sin(ang*DEG2RAD), height);
		glVertex3d(r1*cos(ang*DEG2RAD), r2*sin(ang*DEG2RAD), 0.0);
	}
	glEnd();
}

void DrawCone(double r, double height, unsigned int segments/* = 8*/)
{
	double step = 360.0/segments;

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(0.0, 0.0, 0.0);
	for(double ang = 360.0; ang >= 0.0; ang -= step)
		glVertex3d(r*cos(ang*DEG2RAD), r*sin(ang*DEG2RAD), 0.0);
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, 1.0);
	glVertex3d(0.0, 0.0, height);
	for(double ang = 0.0; ang <= 360.0; ang += step)
	{
		glNormal3d(cos(ang*DEG2RAD), sin(ang*DEG2RAD), 0);
		glVertex3d(r*cos(ang*DEG2RAD), r*sin(ang*DEG2RAD), 0.0);
	}
	glEnd();
}

void DrawSphere(double r, int lon_seg/* = 10*/, int lat_seg/* = 5*/)
{
	double lon_step = 2*M_PI/lon_seg;
	double lat_step = M_PI/lat_seg;

	// Draw upper face
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, 1.0);
	glVertex3d(0.0, 0.0, r);
	for(register int i = 0; i <= lon_seg; i++)
	{
		glNormal3d(cos(lon_step*i)*sin(lat_step), sin(lon_step*i)*sin(lat_step), cos(lat_step));
		glVertex3d(r*cos(lon_step*i)*sin(lat_step), r*sin(lon_step*i)*sin(lat_step), r*cos(lat_step));
	}
	glEnd();

	// Draw bottom face
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(0.0, 0.0, -r);
	for(register int i = lon_seg; i >= 0; i--)
	{
		glNormal3d(cos(lon_step*i)*sin(lat_step), sin(lon_step*i)*sin(lat_step), cos(lat_step*(lat_seg-1)));
		glVertex3d(r*cos(lon_step*i)*sin(lat_step), r*sin(lon_step*i)*sin(lat_step), r*cos(lat_step*(lat_seg-1)));
	}
	glEnd();

	// Draw side face
	for(int j = 2; j <= lat_seg-1; j++)
	{
		glBegin(GL_QUAD_STRIP);
		glNormal3d(sin(lat_step*(j-1)), 0.0, cos(lat_step*(j-1)));
		glVertex3d(r*sin(lat_step*(j-1)), 0.0, r*cos(lat_step*(j-1)));
		glNormal3d(sin(lat_step*j), 0.0, cos(lat_step*j));
		glVertex3d(r*sin(lat_step*j), 0.0, r*cos(lat_step*j));
		for(register int i = 0; i <= lon_seg; i++)
		{
			glNormal3d(cos(lon_step*i)*sin(lat_step*(j-1)), sin(lon_step*i)*sin(lat_step*(j-1)), cos(lat_step*(j-1)));
			glVertex3d(r*cos(lon_step*i)*sin(lat_step*(j-1)), r*sin(lon_step*i)*sin(lat_step*(j-1)), r*cos(lat_step*(j-1)));
			glNormal3d(cos(lon_step*i)*sin(lat_step*j), sin(lon_step*i)*sin(lat_step*j), cos(lat_step*j));
			glVertex3d(r*cos(lon_step*i)*sin(lat_step*j), r*sin(lon_step*i)*sin(lat_step*j), r*cos(lat_step*j));
		}
		glEnd();
	}

}

void DrawEllipsoid(double a, double b, double c, unsigned int lon_seg/* = 10*/, unsigned int lat_seg/* = 10*/)
{
	double lon_step = 2*M_PI/lon_seg;
	double lat_step = M_PI/lat_seg;

	// Draw upper face
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, 1.0);
	glVertex3d(0.0, 0.0, c);
	for(register int i = 0; i <= (int)lon_seg; i++)
	{
		glNormal3d(cos(lon_step*i)*sin(lat_step), sin(lon_step*i)*sin(lat_step), cos(lat_step));
		glVertex3d(a*cos(lon_step*i)*sin(lat_step), b*sin(lon_step*i)*sin(lat_step), c*cos(lat_step));
	}
	glEnd();

	// Draw bottom face
	glBegin(GL_TRIANGLE_FAN);
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(0.0, 0.0, -c);
	for(register int i = lon_seg; i >= 0; i--)
	{
		glNormal3d(cos(lon_step*i)*sin(lat_step), sin(lon_step*i)*sin(lat_step), cos(lat_step*(lat_seg-1)));
		glVertex3d(a*cos(lon_step*i)*sin(lat_step), b*sin(lon_step*i)*sin(lat_step), c*cos(lat_step*(lat_seg-1)));
	}
	glEnd();

	// Draw side face
	for(int j = 2; j <= (int)lat_seg-1; j++)
	{
		glBegin(GL_QUAD_STRIP);
		//glNormal3d(sin(lat_step*(j-1)), 0.0, cos(lat_step*(j-1)));
		//glVertex3d(a*cos(lat_step*(j-1))*sin(lat_step*(j-1)), 0.0, r*cos(lat_step*(j-1)));
		//glNormal3d(sin(lat_step*j), 0.0, cos(lat_step*j));
		//glVertex3d(r*sin(lat_step*j), 0.0, r*cos(lat_step*j));
		for(register int i = 0; i <= (int)lon_seg; i++)
		{
			glNormal3d(cos(lon_step*i)*sin(lat_step*(j-1)), sin(lon_step*i)*sin(lat_step*(j-1)), cos(lat_step*(j-1)));
			glVertex3d(a*cos(lon_step*i)*sin(lat_step*(j-1)), b*sin(lon_step*i)*sin(lat_step*(j-1)), c*cos(lat_step*(j-1)));
			glNormal3d(cos(lon_step*i)*sin(lat_step*j), sin(lon_step*i)*sin(lat_step*j), cos(lat_step*j));
			glVertex3d(a*cos(lon_step*i)*sin(lat_step*j), b*sin(lon_step*i)*sin(lat_step*j), c*cos(lat_step*j));
		}
		glEnd();
	}
}

void DrawBox(double cx, double cy, double cz)
{
	double w = cx*0.5;
	double h = cy*0.5;
	double d = cz*0.5;
	glBegin(GL_QUADS);
	// Front Face
	glNormal3d(0.0, 0.0, 1.0);
	glVertex3d(-w, -h,  d);
	glVertex3d( w, -h,  d);
	glVertex3d( w,  h,  d);
	glVertex3d(-w,  h,  d);
	// Back Face
	glNormal3d(0.0, 0.0, -1.0);
	glVertex3d(-w, -h, -d);
	glVertex3d(-w,  h, -d);
	glVertex3d( w,  h, -d);
	glVertex3d( w, -h, -d);
	// Top Face
	glNormal3d(0.0, 1.0, 0.0);
	glVertex3d(-w,  h, -d);
	glVertex3d(-w,  h,  d);
	glVertex3d( w,  h,  d);
	glVertex3d( w,  h, -d);
	// Bottom Face
	glNormal3d(0.0, -1.0, 0.0);
	glVertex3d(-w, -h, -d);
	glVertex3d( w, -h, -d);
	glVertex3d( w, -h,  d);
	glVertex3d(-w, -h,  d);
	// Right face
	glNormal3d(1.0, 0.0, 0.0);
	glVertex3d( w, -h, -d);
	glVertex3d( w,  h, -d);
	glVertex3d( w,  h,  d);
	glVertex3d( w, -h,  d);
	// Left Face
	glNormal3d(-1.0, 0.0, 0.0);
	glVertex3d(-w, -h, -d);
	glVertex3d(-w, -h,  d);
	glVertex3d(-w,  h,  d);
	glVertex3d(-w,  h, -d);
	glEnd();
}