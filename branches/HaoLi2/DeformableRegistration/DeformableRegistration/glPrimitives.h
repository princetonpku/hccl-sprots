#pragma once

#include "VectorQuaternion.h"

void glNormalv(Vector3d vec);
void glTranslatev(Vector3d vec);
void glRotatev(double ang, Vector3d vec);
void glVertexv(Vector3d vec);

// 2D Drawings
void DrawEllipse(double r1, double r2, unsigned int segments = 10);

// 3D Drawings
void DrawArrow(double length, double thickness);
void DrawAxis(Vector3d length);
void DrawGrid(float fExtent, float fStep);
void DrawCylinder(double r, double height, unsigned int segments = 10);
void DrawEllipCylinder(double r1, double r2, double height, unsigned int segments = 10);
void DrawCone(double r, double height, unsigned int segments = 10);
void DrawSphere(double r, int lon_seg = 20, int lat_seg = 10);
void DrawEllipsoid(double a, double b, double c, unsigned int lon_seg = 20, unsigned int lat_seg = 20);
void DrawBox(double cx, double cy, double cz);