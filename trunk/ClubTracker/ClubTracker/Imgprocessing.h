#include <opencv2/opencv.hpp>

static cv::Scalar colors[7] = 
{
	cv::Scalar(0,22,255),
	cv::Scalar(0,255,22),
	cv::Scalar(255,10,30),
	cv::Scalar(10,255,255),
	cv::Scalar(255,255,10),
	cv::Scalar(255,10,255),
	cv::Scalar(255,255,255)
};

void Normalize(cv::Point2f& p);
void Normalize(cv::Point3f& p);

cv::Vec3f pt2line(const cv::Point2f& pt1, const cv::Point2f& pt2);
cv::Vec3f pt2line(const cv::Vec4i& line);
float pt2linDist(const cv::Point2f& pt, const cv::Vec3f& line);
cv::Vec2d line2para(const cv::Vec3f& line);


cv::Point3f PointProjection(const cv:: Point3f& pt, const cv::Vec4f& pln);
cv::Point3f PointProjection(const cv:: Point3f& pt, const cv::Vec6f& lin);
cv::Point3f PointProjection(const cv:: Point3f& pt, const cv:: Point3f& dir, const cv::Vec6f& lin);

cv::Vec4f GetPlane(const cv::Point3f& pt1, const cv::Point3f& pt2, const cv::Point3f& pt3);
cv::Point3f GetPlaneNormal(const cv::Point3f& pt1, const cv::Point3f& pt2, const cv::Point3f& pt3);
cv::Vec6f GetCrossLine(const cv::Vec4f plane1, const cv::Vec4f plane2);
cv::Point2f GetCrossPoint(const cv::Vec3f& line1, const cv::Vec3f& line2);

void MakeSubMat(cv::Mat& patent, std::vector<cv::Mat>& children, int row, int col);
cv::Mat update_brightcont(const cv::Mat &src_image, uchar* lut, int brightness, int contrast);
cv::Mat HistoImg(cv::Mat &src);
void LineScan(const cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, int num, cv::Point2f& dpt1, int a, int b);

std::vector<float> cubic_spline(const std::vector<float>& x_series, const std::vector<float>& y_series, const std::vector<float>& destX);

void poly_polar_fitting(cv::Mat& pts, const int n);
void poly_fitting(cv::Mat& pts, const int n);
std::vector<float> poly_model(const cv::Mat& pts, const int n);
cv::Mat bezier_model(const cv::Mat& pts, const std::vector<float>& param, const int n);
cv::Mat bezier_model2(const cv::Mat& pts, const std::vector<float>& param, const int n);
cv::Mat bezier_val(const cv::Mat& model, const std::vector<float>& param);
std::vector<float> chord_legth(const cv::Mat& pts);
float Bernstein_polynomial(const int n, const int i, const float t);
int combination(int a, int b);
void cliping_bezier(const cv::Mat& model, const double t, cv::Mat& dst1, cv::Mat& dst2);
void Bezier_intersection2d(const cv::Mat& model, const cv::Vec4f& line_seg, std::vector<cv::Point2f>& result_pts);
bool box_lineseg_intersection2d(const cv::Rect_<float>& r, const cv::Vec4f& line_seg);
bool lineseg_intersection2d(const cv::Vec4f& l1, const cv::Vec4f& l2);
bool CCW_test2d(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3);

void drawVector(cv::Mat& img, const std::vector<cv::Point>& vecm, const cv::Scalar& clr, int width = 1, bool show = false);
void drawVector(cv::Mat& img, const std::vector<cv::Point2f>& vecm, const cv::Scalar& clr, int width = 1);
void drawVector(cv::Mat& img, const cv::Mat& m, const cv::Scalar& clr, int width = 1);

void drawLines(cv::Mat& m, const std::vector<cv::Vec4i>& lines, const cv::Scalar& clr, int width = 1);
void drawLines(cv::Mat& m, const std::vector<cv::Vec4f>& lines, const cv::Scalar& clr, int width = 1);
void drawLine(cv::Mat& m, const cv::Vec3f& line, const cv::Scalar& clr, int width = 1);
void drawLine(cv::Mat& m, const cv::Vec4f& line, const cv::Scalar& clr, int width = 1);
void drawLineSegment(cv::Mat& m, const cv::Vec4f& line, const cv::Scalar& clr, int width = 1);

float getContourDistance(const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2, bool show = false);
float getContourDistance2(const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2, bool show = false);

cv::Scalar registration2D(cv::Mat& m_sml, const cv::Mat& m_lrg, cv::Scalar& tr_output, bool show = false);
cv::Mat coupling(cv::Mat& m_sml, const cv::Mat& m_lrg);
float getSignedAngle(const cv::Point2f& v1, const cv::Point2f& v2);
float getUnsignedAngle(const cv::Point2f& v1, const cv::Point2f& v2);
float getUnsignedAngle(const cv::Point3f& v1, const cv::Point3f& v2);

float getAveragRotAngle( const cv::Mat& m1, const cv::Mat& m2);
cv::Mat getRot2Dmtrx(float radian);


void makeLineGroup(std::vector<cv::Vec4i>& lines, const float thr = 5);
bool isParrallelLine(const cv::Vec4i& line1, const cv::Vec4i& line2, const float thr);
bool isSameLine(const cv::Vec4i& line1, const cv::Vec4i& line2, const float thr);
cv::Vec4f mergeLine(const cv::Vec4f& line1, const cv::Vec4f& line2);
cv::Vec4f mergeLine(const std::vector<cv::Vec4f>& lines);
cv::Vec4f rotLineSegment(const cv::Vec4f& lin, const float angle, const cv::Point2f center = cv::Point2f(0,0));
float calLineSegDist(const cv::Vec4f& l1, const cv::Vec4f& l2);


void saveContour(const char* filename, const std::vector<cv::Point>& contour);
void saveLine(const char* filename, const std::vector<cv::Vec4i>& line);
void saveLine(const char* filename, const std::vector<std::vector<std::vector<cv::Vec4i>>>& line);


std::vector<cv::Vec4i> extractLineSegment(const std::vector<cv::Point>& contour, const float thr);


float getLineDistance(const cv::Vec4f& l1, const cv::Vec4f& l2);


std::vector<cv::Point2f> dataPostProcessing(std::vector<cv::Point2f>& pt1, std::vector<cv::Point2f>& pt2, const int n_iter = 3, const float r = 0);
void dataPostProcessing(cv::Mat& lines, const int n_iter = 3, const float r = 0);
void temfunc3(std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir);
void temfunc3(std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir, std::vector<int>& indx);
void temfunc4(std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir, const float r = 0.5);
void temfunc4(std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir, std::vector<int>& indx, const float r = 0.5);


void drawGraph( cv::Mat& canvas, const cv::Mat& data);
