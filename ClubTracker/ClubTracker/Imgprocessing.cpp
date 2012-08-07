#include "Imgprocessing.h"
#include <math.h>
#include <fstream>

using namespace cv;
using namespace std;


void Normalize(cv::Point2f& p)
{
	double n = norm(p);
	p.x = p.x/n;
	p.y = p.y/n;
}
void Normalize(cv::Point3f& p)
{
	double n = norm(p);
	p.x = p.x/n;
	p.y = p.y/n;
	p.z = p.z/n;
}
cv::Point3f PointProjection( const cv:: Point3f& pt, const cv::Vec4f& pln )
{
	return pt - (Point3f(pln[0], pln[1], pln[2]).ddot(pt) + pln[3])*Point3f(pln[0], pln[1], pln[2]);
}
cv::Point3f PointProjection( const cv:: Point3f& pt, const cv::Vec6f& lin )
{
	Point3f lindir = Point3f(lin[0], lin[1], lin[2]);
	Point3f linpt = Point3f(lin[3], lin[4], lin[5]);
	Point3f ref_vec = pt-linpt;
	float n1 = norm(ref_vec);
	float n2 = norm(lindir);

	float costheta = lindir.ddot(ref_vec)/n1/n2;

	return (linpt + n1*costheta*lindir);
}
cv::Point3f PointProjection( const cv:: Point3f& pt, const cv:: Point3f& dir, const cv::Vec6f& lin )
{
	Point3f linpt = Point3f(lin[3], lin[4], lin[5]);
	Point3f lindir = Point3f(lin[0], lin[1], lin[2]);


	Mat a(2,2,CV_32FC1);
	a.at<float>(0,0) = -dir.y;
	a.at<float>(0,1) = dir.x;
	a.at<float>(1,0) = -lindir.y;
	a.at<float>(1,1) = lindir.x;

	float d = determinant(a);

	Mat b(2,1,CV_32FC1);
	b.at<float>(0,0) = pt.x-linpt.x;	
	b.at<float>(1,0) = pt.y-linpt.y;


	Mat c = a*b;

	float t = c.at<float>(0,0)*(1/d);

	return t*lindir+linpt;



	// 	Point3f ref_vec = pt-linpt;
	// 
	// 	float sintheta1 = norm(dir.cross(-ref_vec))/norm(dir)/norm(ref_vec);
	// 	float sintheta2 = norm(lindir.cross(ref_vec))/norm(lindir)/norm(ref_vec);	
	// 	float costheta1 = dir.ddot(-ref_vec)/norm(dir)/norm(ref_vec);
	// 	float costheta2 = lindir.ddot(ref_vec)/norm(lindir)/norm(ref_vec);
	// 
	// 	float l = norm(ref_vec)*sintheta2/(sintheta1*costheta2+costheta1*sintheta2);
	// 
	// 	return pt+(l/norm(dir))*dir;
}


cv::Vec4f GetPlane(const cv::Point3f& pt1, const cv::Point3f& pt2, const cv::Point3f& pt3)
{
	// 	cv::Point3f n_vector(pt1.y*(pt2.z - pt3.z) + pt2.y*(pt3.z - pt1.z) + pt3.y*(pt1.z - pt2.z),
	// 		pt1.z*(pt2.x - pt3.x) + pt2.z*(pt3.x - pt1.x) + pt3.z*(pt1.x - pt2.x),
	// 		pt1.x*(pt2.y - pt3.y) + pt2.x*(pt3.y - pt1.y) + pt3.x*(pt1.y - pt2.y));
	cv::Point3f n_vector = (pt2-pt1).cross(pt3-pt2);
	float d = -( pt1.x*(pt2.y*pt3.z - pt3.y*pt2.z) + pt2.x*(pt3.y*pt1.z - pt1.y*pt3.z) + pt3.x * (pt1.y*pt2.z - pt2.y*pt1.z) );

	return cv::Vec4f(n_vector.x, n_vector.y, n_vector.z, d);
}
cv::Point3f GetPlaneNormal(const cv::Point3f& pt1, const cv::Point3f& pt2, const cv::Point3f& pt3)
{
	// 	cv::Point3f n_vector(pt1.y*(pt2.z - pt3.z) + pt2.y*(pt3.z - pt1.z) + pt3.y*(pt1.z - pt2.z),
	// 		pt1.z*(pt2.x - pt3.x) + pt2.z*(pt3.x - pt1.x) + pt3.z*(pt1.x - pt2.x),
	// 		pt1.x*(pt2.y - pt3.y) + pt2.x*(pt3.y - pt1.y) + pt3.x*(pt1.y - pt2.y));

	cv::Point3f n_vector = (pt2-pt1).cross(pt3-pt2);


	float val = cv::norm(n_vector);	

	return cv::Point3f(n_vector.x/val, n_vector.y/val, n_vector.z/val);
}

void MakeSubMat(cv::Mat& patent, std::vector<cv::Mat>& children, int row, int col)
{
	int w = patent.cols;
	int h = patent.rows;

	children.clear();
	children.resize(row*col);

	int imgx = w/col;
	int imgy = h/row;

	for (int i = 0; i<row; ++i)
	{for (int j = 0; j<col; ++j)
	{
		children[i*col + j] = cv::Mat(patent, cv::Rect(j*imgx,i*imgy, imgx, imgy));
	}}

}

cv::Mat update_brightcont(const cv::Mat &src_image, uchar* lut, int brightness, int contrast)
{
	brightness -= 100;
	contrast -= 100;

	int i, bin_w;
	float max_value = 0;

	if( contrast > 0 )
	{
		float delta = 127.*contrast/100;
		float a = 255./(255. - delta*2);
		float b = a*(brightness - delta);
		for( i = 0; i < 256; i++ )
		{
			int v = cvRound(a*i + b);
			if( v < 0 )
				v = 0;
			if( v > 255 )
				v = 255;
			lut[i] = (uchar)v;
		}
	}
	else
	{
		float delta = -128.*contrast/100;
		float a = (256.-delta*2)/255.;
		float b = a*brightness + delta;
		for( i = 0; i < 256; i++ )
		{
			int v = cvRound(a*i + b);
			if( v < 0 )
				v = 0;
			if( v > 255 )
				v = 255;
			lut[i] = (uchar)v;
		}
	}

	cv::Mat lut_mat( 1, 256, CV_8UC1, lut);

	cv::Mat dst;
	cv::LUT(src_image, lut_mat, dst);

	return dst;
}
cv::Mat HistoImg(cv::Mat &src)
{
	cv::MatND hist;
	int channels[] = {0};

	int histSize= 256;	
	float granges[] = { 0, 256 };
	const float* ranges[] = {granges};

	if (src.channels() == 1)
	{
		calcHist(&src, 1, channels, cv::Mat(), hist, 1, &histSize, ranges);
	}
	else
	{
		cv::Mat g;
		cvtColor(src, g, CV_RGB2GRAY);
		calcHist(&g, 1, channels, cv::Mat(), hist, 1, &histSize, ranges);
	}	

	cv::Mat histImg = cv::Mat::zeros(256, histSize, CV_8UC3);

	double maxval;
	minMaxLoc(hist, NULL, &maxval);

	double scale = 256/maxval;


	for( int i = 0; i < histSize; i++ )
	{
		float binVal = hist.at<float>(i, 0);
		//rectangle( histImg, Point(i,0), Point(i+1, binVal*scale), Scalar(0,0,0),1);
		line( histImg, cv::Point(i,histImg.rows), cv::Point(i, histImg.rows - binVal*scale), cv::Scalar(0,0,255),1);
	}

	return histImg;
}


void LineScan(const cv::Mat& img, const cv::Point2f& p1, const cv::Point2f& p2, int num, cv::Point2f& dpt, int a, int b)
{
	Point2f cntr = (p1+p2)*0.5;
	Point2f dir = p1-p2;

	Normalize(dir);

	Mat rslt(1, num, CV_32FC3);
	Scalar temsclr;

	Point2f tem = cntr + dir*num;

	Mat sh = img.clone();

	for (int i = 0; i<num; ++i)
	{
		Point2f tem = cntr + dir*i;

		for (int j = 0; j<3; ++j)
		{
			rslt.at<float>(0,i*3+j) = img.at<uchar>(int(tem.y), int(tem.x)*3+j);
		}
	}
	
	Mat rr;
	blur(rslt, rr, Size(3,1));
// 	convertScaleAbs(rslt,sh);
// 	imshow("a", sh);

	blur(rr, rr, Size(3,1));
// 	convertScaleAbs(rr,sh);
// 	imshow("b", sh);

	blur(rr, rr, Size(3,1));
// 	convertScaleAbs(rr,sh);
// 	imshow("c", sh);


// 	Mat bil;
// 	bilateralFilter(rslt, bil, -1, a*0.1, b*0.1);
// 
// 	convertScaleAbs(bil,sh);
// 	imshow("c", sh);


	Mat knl(1,3,CV_32FC1);
	knl.at<float>(0,0) = -1;
	knl.at<float>(0,1) = 0;
	knl.at<float>(0,2) = 1;

	Mat ff;
	filter2D(rr, ff, -1, knl);
	
	Mat sp[3];
	split(ff, sp);

	Mat sm;
	sm = abs(sp[0]+sp[1]+sp[2]);

	double mxval;
	Point mxpt;
	minMaxLoc(sm, NULL, &mxval, NULL, &mxpt);

//	cout<< mxval<<endl;

	int vv = 150;
	Mat graph(vv, num, CV_32FC3, Scalar(0,0,0));
	for (int i = 0; i<num; ++i)
	{
		line(graph, Point(i, 499), Point(i, vv-sm.at<float>(0,i)), Scalar(0,0,255));
// 		line(graph, Point(i, 499), Point(i, (500-sm.at<float>(0,i)*500.0/mxval)), Scalar(0,0,255));
// 		graph.at<float>(500-(sm.at<float>(0,i)*500.0/mxval),i) = 1;
	}

// 	imshow("graph", graph);
	dpt= cntr+mxpt.x*dir;
}

cv::Vec6f GetCrossLine(const cv::Vec4f plane1, const cv::Vec4f plane2)
{
	int n = 0;
	vector<Point3f> ref_pt;
	if (plane1[0]!=0) ref_pt.push_back(Point3f(-plane1[3]/plane1[0],0,0)); n = 0;
	if (plane1[1]!=0) ref_pt.push_back(Point3f(0,-plane1[3]/plane1[1],0)); n = 1;
	if (plane1[2]!=0) ref_pt.push_back(Point3f(0,0,-plane1[3]/plane1[2])); n = 2;

	if (ref_pt.size()<2)
	{
		if (plane1[0]!=0) ref_pt.push_back(Point3f(-(plane1[3]+plane1[1]+plane1[2])/plane1[0],1,1));
		if (plane1[1]!=0) ref_pt.push_back(Point3f(1,-(plane1[3]+plane1[0]+plane1[2])/plane1[1],1));
		if (plane1[2]!=0) ref_pt.push_back(Point3f(1,1,-(plane1[3]+plane1[0]+plane1[1])/plane1[2]));
	}

	Point3f ref_dir = ref_pt[0] - ref_pt[1];

	Point3f norm_vec1(plane1[0], plane1[1], plane1[2]);
	Point3f norm_vec2(plane2[0], plane2[1], plane2[2]);

	float tem1 = norm_vec2.ddot(ref_dir);
	float tem2 = norm_vec2.ddot(ref_pt[0]) + plane2[3];

	if (tem1 == 0) return cv::Vec6f();
	else
	{
		float t = -tem2/tem1;

		Point3f onpoint = ref_pt[0] + t*ref_dir;
		Point3f dir = norm_vec1.cross(norm_vec2);
		float n = norm(dir);

		return Vec6f(dir.x/n, dir.y/n, dir.z/n, onpoint.x, onpoint.y, onpoint.z);
	}	
}
std::vector<float> cubic_spline(const std::vector<float>& x_series, const std::vector<float>& y_series, const std::vector<float>& destX)
{   
	int n = min((int)x_series.size()-1, (int)y_series.size()-1);

	// Step 1.
	float *h = new float[n+1];
	float *alpha = new float[n+1];
	int i = 0;
	for(i = 0; i<=n-1; i++){
		h[i] = (x_series)[i+1] - (x_series)[i];
	}

	// Step 2.
	for(i = 1; i<=n-1;i++){
		alpha[i]= 3*((y_series)[i+1]-(y_series)[i])/h[i]-3*((y_series)[i]-(y_series)[i-1])/h[i-1];
	}

	// Step 3.
	float *l = new float[n+1];
	float *u = new float[n+1];
	float *z = new float[n+1];
	float *c = new float[n+1];
	float *b = new float[n+1];
	float *d = new float[n+1];

	l[0] = 1; u[0] = 0; z[0] = 0;

	// Step 4.
	for(i = 1; i<=n-1; i++){
		l[i] = 2*((x_series)[i+1] - (x_series)[i-1]) - h[i-1]*u[i-1];
		u[i] = h[i]/l[i];
		z[i] = (alpha[i] - h[i-1]*z[i-1]) / l[i];
	}

	// Step 5.
	l[n] = 1;     z[n] = 0;     c[n] = 0;

	// Step 6.
	for(i = n-1; i>=0; i--){
		c[i] = z[i] - u[i]*c[i+1];
		b[i] = ((y_series)[i+1] - (y_series)[i])/h[i] - h[i]*(c[i+1] + 2*c[i])/3;
		d[i] = (c[i+1] - c[i]) / (3*h[i]);
	}


	std::vector<float> out(destX.size());
	int cnt = 0;
	for (int ii = 0; ii<destX.size(); ++ii)
	{
		int i = destX[ii] - cnt -1;

		float x = destX[ii];
		float x_offset = x - (x_series)[i];
		float Sx = (y_series)[i] + b[i]*x_offset + c[i]*x_offset*x_offset + d[i]*x_offset*x_offset*x_offset;

		out[ii] = Sx;

		++cnt;
	}

	delete [] h;
	delete [] alpha;
	delete [] l;
	delete [] u;
	delete [] z;
	delete [] c;
	delete [] b;
	delete [] d;

	return out;
}

void poly_polar_fitting( cv::Mat& pts, const int n )
{
	bool angleIndegree = false;

	if (!n) return;

	int row = pts.rows;

	Mat rho, theta;
	cartToPolar(pts.col(0), pts.col(1), rho, theta, angleIndegree);

	float a1, a2, d, t1 = CV_PI*1.6, t2 = CV_PI*2;

	if (angleIndegree)
	{
		t1 = 270;
		t2 = 360;
	}

	for (int i = 1; i<theta.rows; ++i)
	{
		a1 = theta.at<float>(i,0);
		a2 = theta.at<float>(i-1,0);
		d = a1 - a2;

		if (d<-t1) theta.at<float>(i,0) += t2;
		else if (d>t1) theta.at<float>(i,0) -= t2;		
	}


	Mat A(row, n+1, CV_32FC1);
	A.col(0) = Scalar(1);
	theta.copyTo(A.col(1));
	for (int i = 2; i<n+1; ++i)
	{
		if (i%2)
		{
			Mat mask = theta < 0;
			pow(theta, i, A.col(i));
			subtract(Scalar::all(0), A.col(i), A.col(i), mask);
		}
		else pow(theta, i, A.col(i));
	}	

	Mat x;
	solve(A, rho, x, DECOMP_NORMAL);
	Mat new_rho = A*x;
	polarToCart(new_rho, theta, pts.col(0), pts.col(1), angleIndegree);
}
void poly_fitting( cv::Mat& pts, const int n )
{
	if (!n) return;
	int row = pts.rows;

	pts = pts.reshape(1);
	Mat A(row, n+1, CV_32FC1);
	A.col(0) = Scalar(1);
	pts.col(0).copyTo(A.col(1));

	for (int i = 2; i<n+1; ++i)
	{
		if (i%2)
		{
			Mat mask = pts.col(0) < 0;
			pow(pts.col(0), i, A.col(i));
			subtract(Scalar::all(0), A.col(i), A.col(i), mask);
		}
		else pow(pts.col(0), i, A.col(i));
	}

	Mat coef;
	solve(A, pts.col(1), coef, DECOMP_SVD || DECOMP_NORMAL);
		
	Mat(A*coef).copyTo(pts.col(1));
}
std::vector<float> poly_model( const cv::Mat& pts, const int n )
{
	if (!n) return vector<float>(n);
	int row = pts.rows;

	Mat tem = pts.reshape(1).clone();
	Mat A(row, n+1, CV_32FC1);
	A.col(0) = Scalar(1);
	tem.col(0).copyTo(A.col(1));

	for (int i = 2; i<n+1; ++i)
	{
		if (i%2)
		{
			Mat mask = tem.col(0) < 0;
			pow(tem.col(0), i, A.col(i));
			subtract(Scalar::all(0), A.col(i), A.col(i), mask);
		}
		else pow(tem.col(0), i, A.col(i));
	}
	
	vector<float> coef;
	solve(A, tem.col(1), Mat(coef), DECOMP_SVD || DECOMP_NORMAL);

	return coef;
}

cv::Mat bezier_model( const cv::Mat& pts, const std::vector<float>& param, const int n )
{
	int row = pts.rows;
	int channel = pts.channels();

	if (n<2) return Mat(n+1,1,CV_32FC(channel));

	Mat model_pt(n+1,1,CV_32FC(channel));

	Mat b(row, n+1, CV_32FC1);
	for (int i = 0; i<row; ++i)
		for (int j = 0; j<n+1; ++j)
			b.at<float>(i,j) = Bernstein_polynomial(n, j, param[i]);

	for (int i = 0; i<channel; ++i)
		solve(b, pts.reshape(1).col(i).clone(), model_pt.reshape(1).col(i), DECOMP_SVD || DECOMP_NORMAL);

	return model_pt;
}

cv::Mat bezier_model2( const cv::Mat& pts, const std::vector<float>& param, const int n )
{
	int row = pts.rows;
	int channel = pts.channels();

	if (n<2) return Mat(n+1,1,CV_32FC(channel));

	Mat model_pt(n+1,1,CV_32FC(channel));
	Mat model_pt_tem(n+1-2,1,CV_32FC(channel));

	Mat b(row, n-1, CV_32FC1);
	for (int i = 0; i<row; ++i)	
		for (int j = 0; j<n-1; ++j)
			b.at<float>(i,j) = Bernstein_polynomial(n, j+1, param[i]);

	for (int i = 0; i<channel; ++i)
	{
		Mat tem = pts.reshape(1).col(i).clone();
		float f1 = tem.at<float>(0,0);
		float f2 = tem.at<float>(row-1,0);
		for (int j = 0; j<row; ++j)
		{
			tem.at<float>(j,0) -= Bernstein_polynomial(n, 0, param[i])*f1+Bernstein_polynomial(n, n, param[i])*f2;
		}
		solve(b, tem, model_pt_tem.reshape(1).col(i), DECOMP_SVD || DECOMP_NORMAL);
	}

	pts.row(0).copyTo(model_pt.row(0));
	pts.row(row-1).copyTo(model_pt.row(n));
	model_pt_tem.copyTo(model_pt.rowRange(1, n));

	return model_pt;
}

cv::Mat bezier_val( const cv::Mat& model, const std::vector<float>& param )
{
	int row = param.size();
	int channel = model.channels();
	int n = model.rows-1;

	Mat pts(row,1,CV_32FC(channel));

	Mat before, after;
	for (int i = 0; i<row; ++i)
	{
		before = model.clone();
		after = before.clone();

		for (int j = 0; j<n; ++j)
		{
			after.pop_back();

			for (int k = 0; k<before.rows-1; ++k)
				after.row(k) = (1-param[i])*before.row(k)+param[i]*before.row(k+1);

			before = after.clone();
		}
		after.copyTo(pts.row(i));
	}
	return pts;
}
std::vector<float> chord_legth( const cv::Mat& pts )
{
	int row = pts.rows;	
	vector<float> param(row);	
	for (int i = 1; i<row; ++i)
		param[i] = param[i-1] + norm(pts.row(i)-pts.row(i-1));
	for (int i = 1; i<row; ++i)
		param[i] = (param[i]-param[0])/(param[row-1]-param[0]);

	return param;
}
float Bernstein_polynomial( const int n, const int i , const float t)
{
	if (0<=i && i<=n)
	{
		return combination(n, i)*pow(float(1-t), n-i)*pow(float(t), i);
	}
	else return 0;	
}
int combination(int a, int b)
{
	if (b<=a)
	{
		if (a-b<b) b = a-b;
		int tem1 = 1, tem2 = 1;

		for (int i = 0; i<b; ++i)
		{
			tem1 = tem1*(a-i);
			tem2 = tem2*(b-i);
		}
		return tem1/tem2;
	}
	else return -1;
}
void cliping_bezier( const cv::Mat& model, const double t, cv::Mat& dst1, cv::Mat& dst2 )
{
	int degree = model.rows-1;
	int i, j;

	dst1 = model.clone();
	dst2 = model.clone();
	Mat tem = model.clone();

	for ( i = 0; i<degree; ++i)
	{		
		for ( j = 0; j<degree-i; ++j)
		{
			Mat((1-t)*tem.row(j)+t*tem.row(j+1)).copyTo(tem.row(j));
		}

		tem.row(0).copyTo(dst1.row(i+1));
		tem.row(degree-i-1).copyTo(dst2.row(degree-i-1));
	}

	flip(dst2, dst2, 1);
}

void Bezier_intersection2d( const cv::Mat& model, const cv::Vec4f& line_seg, std::vector<Point2f>& result_pts )
{	

	Rect_<float> rect;	
	bool isinside;
	
	rect = boundingRect(model);

// 	Mat c = imread("t.png");
// 	rectangle(c, rect, Scalar(255,0,0));
// 	imshow("a", c);
// 	waitKey(0);

 	isinside = box_lineseg_intersection2d(rect, line_seg);

	if (isinside)
	{
// 		rectangle(c, rect, Scalar(0,255,0));
// 		imshow("a", c);
// 		waitKey(0);

		if (rect.area() < 4)
		{
			result_pts.push_back(Point2f(rect.x+0.5*rect.width, rect.y+0.5*rect.height));
		} 
		else
		{
			Mat part1, part2;
			cliping_bezier(model, 0.5, part1, part2);

			Bezier_intersection2d(part1, line_seg, result_pts);
			Bezier_intersection2d(part2, line_seg, result_pts);
		}
	}

// 	rectangle(c, rect, Scalar(0,0,255));
// 	imshow("a", c);
// 	waitKey(0);

}

bool box_lineseg_intersection2d( const cv::Rect_<float>& r, const cv::Vec4f& line_seg )
{
	Point2f pa(line_seg[0], line_seg[1]);
	Point2f pb(line_seg[2], line_seg[3]);

	if (r.contains(pa) && r.contains(pb)) return true;

	Vec4f rl[4];
	rl[0] = Vec4f(r.x, r.y, r.x+r.width, r.y);
	rl[1] = Vec4f(r.x, r.y, r.x, r.y+r.height);
	rl[2] = Vec4f(r.x+r.width, r.y+r.height, r.x+r.width, r.y);
	rl[3] = Vec4f(r.x+r.width, r.y+r.height, r.x, r.y+r.height);

	for (int i = 0; i<4; ++i) if (lineseg_intersection2d(line_seg, rl[i])) return true;

	return false;
}

bool lineseg_intersection2d( const cv::Vec4f& l1, const cv::Vec4f& l2 )
{
	Point2f a(l1[0], l1[1]);
	Point2f b(l1[2], l1[3]);
	Point2f c(l2[0], l2[1]);
	Point2f d(l2[2], l2[3]);

	return (CCW_test2d(a,c,d)!=CCW_test2d(b,c,d) && CCW_test2d(a,b,c)!=CCW_test2d(a,b,d));
}
bool CCW_test2d( const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& p3 )
{
	Point2f d1 = p2-p1;
	Point2f d2 = p3-p2;	

	if (d1.x*d2.y-d1.y*d2.x > 0) return true;
	else return false;
}



cv::Vec3f pt2line( const Point2f& pt1, const Point2f& pt2 )
{
	Point2f dir = pt2-pt1;

	float a = dir.y;
	float b = -dir.x;
	float c = dir.x*pt1.y-dir.y*pt1.x;
	float n = norm(dir);

	return Vec3f(a/n, b/n, c/n);
}

cv::Vec3f pt2line( const cv::Vec4i& line )
{
	Point2f pt1(line[0], line[1]);
	Point2f pt2(line[2], line[3]);
	Point2f dir = pt2-pt1;

	float a = dir.y;
	float b = -dir.x;
	float c = dir.x*pt1.y-dir.y*pt1.x;
	float n = norm(dir);

	return Vec3f(a/n, b/n, c/n);
}

float pt2linDist( const Point2f& pt, const Vec3f& l )
{
	Vec2d tem(l[0], l[1]);
	return abs(l[0]*pt.x+l[1]*pt.y+l[2])/norm(tem);
}

cv::Vec2d line2para( const Vec3f& line )
{
	float theata = atan2(-line[1], line[0]);
	return Vec2d(pt2linDist(Point2f(0,0), line), theata*180/CV_PI);
}


void drawVector( cv::Mat& img, const std::vector<cv::Point>& vecm, const cv::Scalar& clr, int width /*= 1*/, bool show /*= false*/ )
{
	if (show) circle(img, vecm[0], width, Scalar(0,255,255), width);
	for (int i = 1; i<vecm.size(); ++i)
	{
		line(img, vecm[i], vecm[i-1], clr, width);
		if (show) circle(img, vecm[i], width, Scalar(0,255,255), width);
	}
	line(img, vecm[0], vecm[vecm.size()-1], clr, width);
}

void drawVector( cv::Mat& img, const std::vector<cv::Point2f>& vecm, const cv::Scalar& clr, int width )
{
	for (int i = 1; i<vecm.size(); ++i)
	{
		line(img, vecm[i], vecm[i-1], clr, width);
	}
// 	line(img, vecm[0], vecm[vecm.size()-1], clr, width);
}

void drawVector( cv::Mat& img, const Mat& m, const cv::Scalar& clr, int width )
{
	vector<Point2f> vecm = m;
	for (int i = 1; i<vecm.size(); ++i)
	{
		line(img, vecm[i], vecm[i-1], clr, width);
	}
	line(img, vecm[0], vecm[vecm.size()-1], clr, width);
}

void drawLines( cv::Mat& m, const std::vector<cv::Vec4i>& lines, const cv::Scalar& clr, int width /*= 1*/ )
{
	for (int i = 0; i<lines.size(); ++i)
	{	
		Point p1 = Point(lines[i][0], lines[i][1]);
		Point p2 = Point(lines[i][2], lines[i][3]);

		line(m, p1, p2, clr, width);
		circle(m, p1, width, Scalar(0,255,0), width);
		circle(m, p2, width, Scalar(255,0,0), width);
	}
}

void drawLines( cv::Mat& m, const std::vector<cv::Vec4f>& lines, const cv::Scalar& clr, int width /*= 1*/ )
{
	for (int i = 0; i<lines.size(); ++i)
	{	
		Point p1 = Point(lines[i][0], lines[i][1]);
		Point p2 = Point(lines[i][2], lines[i][3]);

		line(m, p1, p2, clr, width);
		circle(m, p1, width, Scalar(0,255,0), width);
		circle(m, p2, width, Scalar(255,0,0), width);
	}
}

float getContourDistance( const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2 , bool sh)
{	
	Mat m_sml, m_lrg, m_cpl, m_origin, show;
	int n1 = c1.size(), n2 = c2.size(), n;		

	if (n1 > n2)
	{
		m_sml = Mat(c2);
		m_lrg = Mat(c1);
		n = n2;
	}
	else
	{
		m_sml = Mat(c1);
		m_lrg = Mat(c2);
		n = n1;
	}

	m_sml.convertTo(m_origin, CV_64FC2);
	m_sml.convertTo(m_sml, CV_64FC2);
	m_lrg.convertTo(m_lrg, CV_64FC2);

	if (sh)
	{
		show = Mat(800, 800, CV_8UC3, Scalar(255, 255, 255));
		drawVector(show, m_sml, Scalar(0,0,255));
		drawVector(show, m_lrg, Scalar(255,0,0));
		imshow("before", show);
		waitKey(0);
		imwrite("before.png", show);
	}

	int cnt = 0;
	Point2f translation_vec(0,0);
	Scalar trform;
	float rot_angle, diff;

	while(1)
	{
		Scalar tem_tr;
		trform = registration2D(m_sml, m_lrg, tem_tr, 0);		
		translation_vec += Point2f(tem_tr.val[0],tem_tr.val[1]);
		rot_angle = trform.val[2];
		cnt++;

// 		cout<<tem.val[2]<<endl;
// 		show = Scalar(255,255,255);
// 		drawVector(m_sml, show, Scalar(0,0,255));
// 		drawVector(m_lrg, show, Scalar(255,0,0));
// 		imshow("show", show);
// 		waitKey(30);

		if((abs(rot_angle) < 1.0e-10 || cnt > 10) && rot_angle!=0) break;		
	}

	if (sh)
	{
		show = Scalar(255,255,255);
		drawVector(show, m_sml, Scalar(0,0,255));
		drawVector(show, m_lrg, Scalar(255,0,0));
		imshow("after", show);
		waitKey(0);
		imwrite("after.png", show);
	}

	diff = norm(Point2f(trform.val[0],trform.val[1]));
	return diff/* + norm(translation_vec)*/;
}

float getContourDistance2( const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2, bool sh)
{
	Mat mc1 = Mat(c1);
	Mat mc2 = Mat(c2);

	float shape_similarity = matchShapes(mc1, mc2, CV_CONTOURS_MATCH_I3, 0);

	mc1.convertTo(mc1, CV_64FC2);
	mc2.convertTo(mc2, CV_64FC2);
	Scalar center_distance = mean(mc1) - mean(mc2);

	float distance = norm(Point2f(center_distance.val[0], center_distance.val[1]));

	// 	cout<<shape_similarity<<"+"<<distance<<"="<<shape_similarity + distance<<endl;

	if (sh)
	{
		int img_w = 800, img_h = 800;
		Mat show = Mat(img_h, img_w, CV_8UC3, Scalar(255, 255, 255));
		drawVector(show, c1, Scalar(0,0,255), 1);
		drawVector(show, c2, Scalar(255,0,0), 1);

		// 		Mat sh1(show, Rect(0,0,img_w,img_h));
		// 		Mat sh2(show, Rect(img_w,0,img_w,img_h));
		// 		drawVector(c1, sh1, Scalar(0,0,255), 2);
		// 		drawVector(c2, sh2, Scalar(255,0,0), 2);
		imshow("show", show);
		waitKey(0);
	}


	return shape_similarity + distance*0.4;
}

cv::Scalar registration2D( cv::Mat& m_sml, const cv::Mat& m_lrg, cv::Scalar& tr_output, bool sh )
{
 	Mat show(800, 800, CV_8UC3, Scalar(255,255,255));	
	Mat m_cpl = coupling(m_sml, m_lrg);

	Scalar center_sml, center_cpl, translation;

	center_sml = mean(m_sml);
	center_cpl= mean(m_cpl);

	if (sh)
	{
		show = Scalar(255,255,255);
		drawVector(m_sml, show, Scalar(0,0,255));
		drawVector(m_cpl, show, Scalar(255,0,0));
		circle(show, Point(center_sml.val[0], center_sml.val[1]), 1, Scalar(125,125,125),2);
		circle(show, Point(center_cpl.val[0], center_cpl.val[1]), 1, Scalar(125,125,125),2);
		imshow("a", show);
		waitKey(0);
	}

	tr_output = center_cpl - center_sml;
	m_sml -= center_sml;
	m_cpl -= center_cpl;

	if (sh)
	{
		Mat tem = Mat(m_sml + center_cpl).clone();
		Mat tem2 = Mat(m_cpl + center_cpl).clone();
		show = Scalar(255,255,255);
		drawVector(tem, show, Scalar(0,0,255));
		drawVector(tem2, show, Scalar(255,0,0));
		circle(show, Point(center_sml.val[0], center_sml.val[1]), 1, Scalar(125,125,125),2);
		circle(show, Point(center_cpl.val[0], center_cpl.val[1]), 1, Scalar(125,125,125),2);
		imshow("a", show);
		waitKey(0);
	}

  	float avgRotAngle = getAveragRotAngle(m_sml, m_cpl);

	Mat m_rot = getRot2Dmtrx(-avgRotAngle);
	m_sml = m_sml.reshape(1);
	m_sml = m_sml*m_rot;
	m_sml = m_sml.reshape(2);
	m_sml += center_cpl;
	m_cpl += center_cpl;

	if (sh)
	{
		show = Scalar(255,255,255);
		drawVector(m_sml, show, Scalar(0,0,255));
		drawVector(m_cpl, show, Scalar(255,0,0));
		circle(show, Point(center_sml.val[0], center_sml.val[1]), 1, Scalar(125,125,125),2);
		circle(show, Point(center_cpl.val[0], center_cpl.val[1]), 1, Scalar(125,125,125),2);
		imshow("a", show);
		waitKey(0);
	}

	Scalar tem = mean(abs(m_sml-m_cpl));
	tem.val[2] = avgRotAngle;

	return tem;
}

float getSignedAngle( const cv::Point2f& v1, const cv::Point2f& v2 )
{
	if (norm(v1)==0 || norm(v2)==0) return 0;
	
	cv::Point3f vv1(v1.x, v1.y, 0);
	cv::Point3f vv2(v2.x, v2.y, 0);

	float tem = vv1.cross(vv2).z;
	if (tem >= 0) tem = 1;
	else tem = -1;

	float tem2 = v1.ddot(v2)/norm(v1)/norm(v2);
	if (tem2 > 1) tem2 = 1;
	else if (tem2 < -1) tem2 = -1;

	return tem*acos(tem2);
}

cv::Mat getRot2Dmtrx( float a )
{
	return Mat_<float>(2, 2) << cos(a), -sin(a), sin(a), cos(a);
}

cv::Mat coupling( cv::Mat& m_sml, const cv::Mat& m_lrg )
{
	vector<Point2f> v_sml, v_lrg, v_cpl;
	
	v_sml = m_sml;
	v_lrg = m_lrg;

	v_cpl.resize(v_sml.size());
	float distncs, tem_d;
	for (int i = 0; i<v_sml.size(); ++i)
	{
		distncs = 1000000;
		for (int j = 0; j<v_lrg.size(); ++j)
		{
			tem_d = norm(v_sml[i] - v_lrg[j]);
			if (tem_d < distncs)
			{
				distncs = tem_d;
				v_cpl[i] = v_lrg[j];
			}						
		}
	}

	return Mat(v_cpl).clone();
}

float getAveragRotAngle( const cv::Mat& m1, const cv::Mat& m2 )
{
	vector<Point2f> v1 = m1, v2 = m2;

	float angleSum = 0;
	for (int i = 0; i<v1.size(); ++i)
	{
		float t = getSignedAngle(v1[i], v2[i]);
		angleSum += t;
	}
		

	return angleSum/float(v1.size());
}

void makeLineGroup( std::vector<cv::Vec4i>& lines, const float thr )
{
	vector<vector<Vec4i>> new_lines;






	for (int i = lines.size()-1; i>0; --i)
	{
		if (isSameLine(lines[i], lines[i-1], thr))
		{
			lines[i-1] = mergeLine(lines[i], lines[i-1]);
			lines.pop_back();
		}
	}	
}

bool isParrallelLine( const cv::Vec4i& line1, const cv::Vec4i& line2, const float thr )
{
	Point2f p1(line1[0],line1[1]);
	Point2f p2(line1[2],line1[3]);
	Point2f p3(line2[0],line2[1]);
	Point2f p4(line2[2],line2[3]);
	
	Point2f dir1 = p1-p2;
	Point2f dir2 = p3-p4;

	float angle = getUnsignedAngle(dir1, dir2)*180/CV_PI;

	if (angle>90) angle = 180-angle;

	if(angle < thr) return true;
	else return false;
}

bool isSameLine( const cv::Vec4i& line1, const cv::Vec4i& line2, const float thr )
{
	Vec4i line3(0.5*(line1[0]+line1[2]), 0.5*(line1[1]+line1[3]), 0.5*(line2[0]+line2[2]), 0.5*(line2[1]+line2[3]));

	if (isParrallelLine(line1, line3, thr)&&isParrallelLine(line2, line3, thr)) return true;
	else return false;
}

cv::Vec4f mergeLine( const cv::Vec4f& line1, const cv::Vec4f& line2 )
{
	vector<Point> p(4);
	p[0] = Point(line1[0],line1[1]);
	p[1] = Point(line1[2],line1[3]);
	p[2] = Point(line2[0],line2[1]);
	p[3] = Point(line2[2],line2[3]);

	Point dir = p[1] - p[0];

	Rect r = boundingRect(Mat(p));

	if (dir.x*dir.y > 0)
	{
		return Vec4f(r.x, r.y, r.x+r.width-1, r.y+r.height-1);
	}
	else
	{
		return Vec4f(r.x+r.width-1, r.y, r.x, r.y+r.height-1);
	}	
}
cv::Vec4f mergeLine( const std::vector<cv::Vec4f>& lines )
{
	Point dir(lines[0][2]-lines[0][0], lines[0][3]-lines[0][1]);
	Mat a(lines);

	a = a.reshape(1);

	Mat t1 = a.colRange(0, 2);
	Mat t2 = a.colRange(2, 4);

	t1.push_back(t2);

	t1 = t1.reshape(2);

	vector<Point> p(t1);

	Rect r = boundingRect(Mat(p));
	
	if (dir.x*dir.y > 0)
	{
		return Vec4f(r.x, r.y, r.x+r.width-1, r.y+r.height-1);
	}
	else
	{
		return Vec4f(r.x+r.width-1, r.y, r.x, r.y+r.height-1);
	}	
}

float getUnsignedAngle( const cv::Point2f& v1, const cv::Point2f& v2 )
{	
	float t1 = (v1.ddot(v2));
	float n1 = norm(v1);
	float n2 = norm(v2);
	float t2 = t1/n1/n2;

	if (t2>1) t2 = 1;
	else if (t2<-1) t2 = -1;

	return acos(t2);
}

float getUnsignedAngle( const cv::Point3f& v1, const cv::Point3f& v2 )
{
	float t1 = (v1.ddot(v2));
	float n1 = norm(v1);
	float n2 = norm(v2);
	float t2 = t1/n1/n2;

	if (t2>1) t2 = 1;
	else if (t2<-1) t2 = -1;

	return acos(t2);
}

void saveContour( const char* filename, const std::vector<cv::Point>& contour )
{
	ofstream fout(filename);
	
	std::for_each(contour.begin(), contour.end(),[&fout](const Point& pt)
	{
		fout<<pt.x<<"\t"<<pt.y<<endl;
	});	
	fout.close();
}
void saveLine( const char* filename, const std::vector<cv::Vec4i>& line )
{
	ofstream fout(filename);

	std::for_each(line.begin(), line.end(),[&fout](const Vec4i& l)
	{
		fout<<l[0]<<"\t"<<l[1]<<"\t"<<l[2]<<"\t"<<l[3]<<endl;
	});	
	fout.close();
}
void saveLine( const char* filename, const std::vector<std::vector<std::vector<cv::Vec4i>>>& lines )
{
	ofstream fout(filename);
	for (int i = 0; i<lines.size(); ++i)
	{
		for (int j = 0; j<lines[i].size(); ++j)
		{
			for (int k = 0; k<lines[i][j].size(); ++k)
			{
				fout<<i<<"\t"<<lines[i][j][k][0]<<"\t"<<lines[i][j][k][1]<<"\t"<<lines[i][j][k][2]<<"\t"<<lines[i][j][k][3]<<endl;
			}			
		}
	}	
	fout.close();

}


std::vector<cv::Vec4i> extractLineSegment( const std::vector<cv::Point>& contour, const float thr )
{
	int n = contour.size();
	vector<Vec4i> lines;
	lines.reserve(n);

	Point pre_p1 = contour[0];
	Point pre_p2 = contour[1];
	Point pre_dir = pre_p2-pre_p1;

	for (int i = 1; i<n-1; ++i)
	{
		Point dir = contour[i+1] - contour[i];

		if (getUnsignedAngle(pre_dir, dir)*180/CV_PI < thr)
		{
			pre_p2 = contour[i+1];
			pre_dir = pre_p2-pre_p1;
		}
		else
		{
			lines.push_back(Vec4i(pre_p1.x, pre_p1.y, pre_p2.x, pre_p2.y));
			pre_p1 = contour[i];
			pre_p2 = contour[i+1];
			pre_dir = pre_p2-pre_p1;
		}
	}

	Point dir = contour[0] - contour[n-1];
	if (getUnsignedAngle(pre_dir, dir)*180/CV_PI < thr)
	{
		pre_p2 = contour[0];
		pre_dir = pre_p2-pre_p1;
	}
	else
	{
		lines.push_back(Vec4i(pre_p1.x, pre_p1.y, pre_p2.x, pre_p2.y));
		pre_p1 = contour[n-1];
		pre_p2 = contour[0];
		pre_dir = pre_p2-pre_p1;
	}
	return lines;
}


float getLineDistance( const cv::Vec4f& l1, const cv::Vec4f& l2 )
{
	Point2f p1(l1[0], l1[1]);
	Point2f p2(l1[2], l1[3]);
	Point2f p3(l2[0], l2[1]);
	Point2f p4(l1[2], l1[3]);

	Point2f dir1 = p2-p1;
	Point2f dir2 = p4-p3;

	float angl1 = atan2(dir1.y,dir1.x)*180/CV_PI;
	if (angl1 < 0) angl1 += 180;

	float angl2 = atan2(dir2.y,dir2.x)*180/CV_PI;
	if (angl2 < 0) angl2 += 180;

	float angle_diff = abs(angl1-angl2);

	Point2f center1 = (p1+p2)*0.5;
	Point2f center2 = (p3+p4)*0.5;
	
	//float distnc_diff = abs(dir2.y*(center1.x-p3.x)-dir2.x*(center1.y-p3.y))/norm(dir2);
	float distnc_diff = norm(center1-center2);

	return angle_diff+distnc_diff;
}

std::vector<cv::Point2f> dataPostProcessing( std::vector<cv::Point2f>& pt1, std::vector<cv::Point2f>& pt2, const int n_iter/* = 3*/, const float r/* = 0*/ )
{
	int n1 = pt1.size(),
		n2 = 0,
		i = 0;
		
	vector<int> indx(n1), n_indx;
	vector<Point2f> n_pt1, n_pt2;
	
	n_indx.reserve(n1);
	n_pt1.reserve(n1);
	n_pt2.reserve(n1);
	
	// eliminate zero elements
	for (i = 0; i<n1;  ++i)
	{
		indx[i] = i;
		if (pt1[i].x!=0 && pt1[i].y!=0 && pt2[i].x!=0 && pt2[i].y!=0)
		{
			n_indx.push_back(i);
			n_pt1.push_back(pt1[i]);
			n_pt2.push_back(pt2[i]);
		}
	}	

	n2 = n_indx.size();	

	// get direction vectors
	Mat mpt1(n_pt1);
	Mat mpt2(n_pt2);
	Mat mdir;
	subtract(mpt2, mpt1, mdir);
	vector<Point2f> dir = mdir;

	for (i = 0; i<n_iter; ++i) temfunc4(n_pt2, dir, r);
	
	vector<Point2f> new_pt2(n1);
	for (i = 0; i<n2; ++i) new_pt2[n_indx[i]] = n_pt2[i];

	return new_pt2;
}

void dataPostProcessing( cv::Mat& lines, const int n_iter /*= 3*/, const float r /*= 0*/ )
{
	Mat pt1 = lines.reshape(1).colRange(0,2);
	Mat pt2 = lines.reshape(1).colRange(2,4);
	Mat dir = Mat(pt2-pt1).reshape(2);

	vector<Point2f> n_pt2 = pt2.reshape(2).clone();
	for (int i = 0; i<n_iter; ++i) temfunc4(n_pt2, dir, r);
	Mat(n_pt2).reshape(1).copyTo(pt2);

	vector<Point2f> n_pt1 = pt1.reshape(2).clone();
	for (int i = 0; i<n_iter; ++i) temfunc4(n_pt1, dir, r);
	Mat(n_pt1).reshape(1).copyTo(pt1);
}

void temfunc3( std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir )
{
	for (int i = 1; i<pt2.size()-1; ++i)
	{
		Point2f tem_dir = pt2[i+1] - pt2[i-1];

		Mat a(2,2,CV_32FC1);
		a.at<float>(0,0) = -dir[i].y;
		a.at<float>(0,1) = dir[i].x;
		a.at<float>(1,0) = -tem_dir.y;
		a.at<float>(1,1) = tem_dir.x;

		float d = determinant(a);

		Mat b(2,1,CV_32FC1);
		b.at<float>(0,0) = pt2[i].x-pt2[i-1].x;	
		b.at<float>(1,0) = pt2[i].y-pt2[i-1].y;

		Mat c = a*b;
		float t = c.at<float>(1,0)*(1/d);

		if (t>0)
		{
			pt2[i] = pt2[i] + t*dir[i];
		}
	}
}
void temfunc3( std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir, std::vector<int>& indx )
{
	for (int i = 1; i<pt2.size()-1; ++i)
	{
		Point2f tem_dir = pt2[i+1] - pt2[i-1];

		Mat a(2,2,CV_32FC1);
		a.at<float>(0,0) = -dir[i].y;
		a.at<float>(0,1) = dir[i].x;
		a.at<float>(1,0) = -tem_dir.y;
		a.at<float>(1,1) = tem_dir.x;

		float d = determinant(a);

		Mat b(2,1,CV_32FC1);
		b.at<float>(0,0) = pt2[i].x-pt2[i-1].x;	
		b.at<float>(1,0) = pt2[i].y-pt2[i-1].y;

		Mat c = a*b;
		float t = c.at<float>(1,0)*(1/d);

		if (t>0)
		{
			Point2f tem = pt2[i] + t*dir[i];
			if (norm(tem-pt2[i])>1)
			{
				indx[i] = 1;
			}
			pt2[i] = tem;
		}			
	}
}
void temfunc4( std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir, const float r/* = 0.5*/ )
{
	Point2f tem_dir;
	Mat a(2,2,CV_32FC1);
	Mat b(2,1,CV_32FC1);
	Mat c;
	float d, t1, t2;

	for (int i = 2; i<pt2.size()-1; ++i)
	{
		tem_dir = pt2[i+1] - pt2[i-1];
		a.at<float>(0,0) = -dir[i].y;
		a.at<float>(0,1) = dir[i].x;
		a.at<float>(1,0) = -tem_dir.y;
		a.at<float>(1,1) = tem_dir.x;
		d = determinant(a);
		b.at<float>(0,0) = pt2[i].x-pt2[i-1].x;	
		b.at<float>(1,0) = pt2[i].y-pt2[i-1].y;
		c = a*b;
		t1 = c.at<float>(1,0)*(1/d);
		
		tem_dir = pt2[i-1] - pt2[i-2];
		a.at<float>(0,0) = -dir[i].y;
		a.at<float>(0,1) = dir[i].x;
		a.at<float>(1,0) = -tem_dir.y;
		a.at<float>(1,1) = tem_dir.x;
		d = determinant(a);
		b.at<float>(0,0) = pt2[i].x-pt2[i-1].x;	
		b.at<float>(1,0) = pt2[i].y-pt2[i-1].y;
		c = a*b;
		t2 = c.at<float>(1,0)*(1/d);

		float t = t1*(1-r)+t2*r;
		if (t>0)
		{
			pt2[i] = pt2[i] + t*dir[i];
		}
	}
}
void temfunc4( std::vector<cv::Point2f>& pt2, const std::vector<cv::Point2f>& dir, std::vector<int>& indx, const float r /*= 0.5*/ )
{
	Point2f tem_dir;
	Mat a(2,2,CV_32FC1);
	Mat b(2,1,CV_32FC1);
	Mat c;
	float d, t1, t2;

	for (int i = 2; i<pt2.size()-1; ++i)
	{
		if (indx[i])
		{
			tem_dir = pt2[i+1] - pt2[i-1];
			a.at<float>(0,0) = -dir[i].y;
			a.at<float>(0,1) = dir[i].x;
			a.at<float>(1,0) = -tem_dir.y;
			a.at<float>(1,1) = tem_dir.x;
			d = determinant(a);
			b.at<float>(0,0) = pt2[i].x-pt2[i-1].x;	
			b.at<float>(1,0) = pt2[i].y-pt2[i-1].y;
			c = a*b;
			t1 = c.at<float>(1,0)*(1/d);
	
			tem_dir = pt2[i-1] - pt2[i-2];
			a.at<float>(0,0) = -dir[i].y;
			a.at<float>(0,1) = dir[i].x;
			a.at<float>(1,0) = -tem_dir.y;
			a.at<float>(1,1) = tem_dir.x;
			d = determinant(a);
			b.at<float>(0,0) = pt2[i].x-pt2[i-1].x;	
			b.at<float>(1,0) = pt2[i].y-pt2[i-1].y;
			c = a*b;
			t2 = c.at<float>(1,0)*(1/d);
	
			float t = t1*(1-r)+t2*r;
			if (t>0)
			{
				pt2[i] = pt2[i] + t*dir[i];
			}
		}
	}

}

cv::Point2f GetCrossPoint( const cv::Vec3f& line1, const cv::Vec3f& line2 )
{
	Mat a(2,2,CV_32FC1);
	a.at<float>(0,0) = line2[1];
	a.at<float>(0,1) = -line1[1];
	a.at<float>(1,0) = -line2[0];
	a.at<float>(1,1) = line1[0];

	Mat b(2,1,CV_32FC1);
	b.at<float>(0,0) = -line1[2];
	b.at<float>(1,0) = -line2[2];

	float d = determinant(a);

	if (d==0) return Point2f();
	else
	{
		Mat x = a*b;
		return Point2f(x.at<float>(0,0)/d, x.at<float>(1,0)/d);
	}
}

void drawLine( cv::Mat& m, const cv::Vec3f& l, const cv::Scalar& clr, int width /*= 1*/ )
{
	Point p1, p2;
	if (l[0]!=0 && l[1]!=0)
	{
		p1 = Point(0, -l[2]/l[1]);
		p2 = Point(m.cols, -(l[2]+m.cols*l[0])/l[1]);
		
	}
	else if (l[0]==0 && l[1]!=0)
	{
		p1 = Point(0, -l[2]/l[1]);
		p2 = Point(m.cols, -l[2]/l[1]);
	}
	else if (l[0]!=0 && l[1]==0)
	{
		p1 = Point(-l[2]/l[0], 0);
		p2 = Point(-l[2]/l[0], m.rows);
	}
	else return;
	
	line(m, p1, p2, clr , width);
}
void drawLine( cv::Mat& m, const cv::Vec4f& l, const cv::Scalar& clr, int width /*= 1*/ )
{
	Vec3f t = pt2line(l);
	drawLine(m, t, clr, width);
}
void drawLineSegment( cv::Mat& m, const cv::Vec4f& l, const cv::Scalar& clr, int width /*= 1*/ )
{
	Point p1(l[0], l[1]), p2(l[2], l[3]);
	line(m, p1, p2, clr, width);
	circle(m, p1, width, Scalar(0,255,0), width);
	circle(m, p2, width, Scalar(255,0,0), width);
}

void drawGraph( cv::Mat& canvas, const cv::Mat& data )
{
	Mat tem;
	data.convertTo(tem, CV_64F);

	int num_data = data.rows;
	int w = canvas.cols;
	int h = canvas.rows;

	double minval, maxval;
	minMaxLoc(tem, &minval, &maxval);

	double dx = double(w)/double(num_data);
	double dy = h*0.8/(maxval-minval);
	vector<Point2f> pt(num_data);
	
	for (int i = 0; i<num_data; ++i)
	{		
		double d = tem.at<double>(i,0);
		pt[i] = Point2f(i*dx, h*0.9-((d-minval)*dy));

		circle(canvas, pt[i], 1, Scalar(0,0,255), 2);
	}

	for (int i = 1; i<num_data; ++i)
	{		
		line(canvas, pt[i-1], pt[i], Scalar(0,0,255));
	}

}

cv::Vec4f rotLineSegment( const cv::Vec4f& lin, const float angle, const cv::Point2f center /*= cv::Point2f(0,0)*/ )
{
	Mat rot = getRotationMatrix2D(center, angle, 1);
	Mat pts(3,2,CV_64FC1, Scalar(1));
	pts.at<double>(0,0) = lin[0];
	pts.at<double>(1,0) = lin[1];
	pts.at<double>(0,1) = lin[2];
	pts.at<double>(1,1) = lin[3];
	
	Mat r = rot*pts;

	return Vec4f(r.at<double>(0,0), r.at<double>(1,0), r.at<double>(0,1), r.at<double>(1,1));
}

float calLineSegDist( const cv::Vec4f& line1, const cv::Vec4f& line2 )
{
	vector<Point> p(4);
	p[0] = Point(line1[0],line1[1]);
	p[1] = Point(line1[2],line1[3]);
	p[2] = Point(line2[0],line2[1]);
	p[3] = Point(line2[2],line2[3]);

	vector<float> v(4);
	v[0] = norm(p[0] - p[2]);
	v[1] = norm(p[0] - p[3]);
	v[2] = norm(p[1] - p[2]);
	v[3] = norm(p[1] - p[3]);

	int min = min_element(v.begin(), v.end())-v.begin();
	
	return  v[min];
}
