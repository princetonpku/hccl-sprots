#ifndef LINESEGMENT_H__
#define LINESEGMENT_H__

#include <opencv2/opencv.hpp>

template <class T> inline bool CCW_test2d( const cv::Point_<T>& p1, const cv::Point_<T>& p2, const cv::Point_<T>& p3 )
{
	cv::Point_<T> d1 = p2-p1;
	cv::Point_<T> d2 = p3-p2;	

	if (d1.x*d2.y-d1.y*d2.x > 0) return true;
	else return false;
}

const double linesegment_theta_thr = 1.0;
const double linesegment_rho_thr = 1.0;

template <class T>
class LineSegment2D
{
public:
	// Consturctor
	LineSegment2D()
	{		
		Update();
		length = 0;
	}
	LineSegment2D(const cv::Vec<T, 4>& line_)
	{
		line = line_;
		Update();
		double dx = double(line[0]-line[2]);
		double dy = double(line[1]-line[3]);
		length = sqrt(dx*dx+dy*dy);
	}
	LineSegment2D(const cv::Point_<T>& p1, const cv::Point_<T>& p2)
	{
		line = cv::Vec<T, 4>(p1.x, p1.y, p2.x, p2.y);
		Update();
		double dx = double(line[0]-line[2]);
		double dy = double(line[1]-line[3]);
		length = sqrt(dx*dx+dy*dy);
	}
 	~LineSegment2D()
	{
	}

	cv::Point_<T> P1() const
	{
		return cv::Point_<T>(line[0], line[1]);
	}
	cv::Point_<T> P2() const
	{
		return cv::Point_<T>(line[2], line[3]);
	}

	void Reverse()
	{
		cv::Vec<T, 4> tem = line;
		line[0] = tem[2];
		line[1] = tem[3];
		line[2] = tem[0];
		line[3] = tem[1];
		Update();
	}
	void Rotate(double angle)
	{
		cv::Mat rot = getRotationMatrix2D(cv::Point2f(center.x, center.y), -angle, 1);
		cv::Mat pts(3,2,CV_64FC1, Scalar(1));
		pts.at<double>(0,0) = double(line[0]);
		pts.at<double>(1,0) = double(line[1]);
		pts.at<double>(0,1) = double(line[2]);
		pts.at<double>(1,1) = double(line[3]);
		cv::Mat r = rot*pts;

		line[0] = T(r.at<double>(0,0));
		line[1] = T(r.at<double>(1,0));
		line[2] = T(r.at<double>(0,1));
		line[3] = T(r.at<double>(1,1));

		Update();
	}
	void Rotate(double angle, cv::Point2f center_)
	{
		cv::Mat rot = getRotationMatrix2D(center_, -angle, 1);
		cv::Mat pts(3,2,CV_64FC1, Scalar(1));
		pts.at<double>(0,0) = double(line[0]);
		pts.at<double>(1,0) = double(line[1]);
		pts.at<double>(0,1) = double(line[2]);
		pts.at<double>(1,1) = double(line[3]);
		cv::Mat r = rot*pts;

		line[0] = T(r.at<double>(0,0));
		line[1] = T(r.at<double>(1,0));
		line[2] = T(r.at<double>(0,1));
		line[3] = T(r.at<double>(1,1));

		Update();
	}
	void Translate(const T x, const T y)
	{
		line[0] += x;
		line[2] += x;
		line[1] += y;
		line[3] += y;

		Update2();		
	}
	void Translate(const cv::Point_<T>& vec)
	{
		line[0] += vec.x;
		line[2] += vec.x;
		line[1] += vec.y;
		line[3] += vec.y;

		Update2();		
	}
	double MinDistFromPt(const cv::Point_<T>& pt) const
	{
		cv::Point_<T> d1, d2;
		if (norm(pt - P1()) < norm(pt - P2()))
		{
			d1 = P1()-pt;
			d2 = P2()-P1();
		}
		else
		{
			d1 = P2()-pt;
			d2 = P1()-P2();
		}
		
		return d1.ddot(d2)>0 ? norm(d1) : abs(double(pt.x)*implicit_param[0]+double(pt.y)*implicit_param[1]+implicit_param[2]);
	}
	double NormalDistFromPt(const cv::Point_<T>& pt) const
	{
		return  abs(double(pt.x)*implicit_param[0]+double(pt.y)*implicit_param[1]+implicit_param[2]);
	}

	cv::Vec<T, 4> line;	
	cv::Point_<T> dir;
	cv::Point_<T> center;

	cv::Vec3d implicit_param;
 	double rho;
 	double theta;
	double length;

// 	LineSegment2D<T> operator=(const LineSegment2D<T>& line_)							// assign operator
// 	{
// 		this->line  = line_.line;
// 		this->dir = line_.dir;
// 		this->center = line_.center;
// 
// 		this->implicit_param = line_.implicit_param;
// 		this->rho  = line_.rho;
// 		this->theta = line_.theta;
// 		this->length = line_.length;
// 
// 		return *this;
// 	}


private:
	void Update()
	{
		dir = P2() - P1();
		double n = norm(dir);
		dir.x /= n;
		dir.y /= n;

		center = 0.5*(P2() + P1());
		theta = atan2(double(dir.y),double(dir.x))*180/CV_PI;
		
		implicit_param[0] = double(-dir.y);
		implicit_param[1] = double(dir.x);
		implicit_param[2] = double(-dir.x*P1().y+dir.y*P1().x);
		rho = abs(implicit_param[2]);
	}
	void Update2() // update when only translate
	{
		center = 0.5*(P2() + P1());		
		implicit_param[2] = double(-dir.x*P1().y+dir.y*P1().x);
		rho = abs(implicit_param[2]);
	}
};

template <class T> inline LineSegment2D<T> ReverseLine(LineSegment2D<T>& line)
{
	line.Reverse();
}
template <class T> inline LineSegment2D<T> RotateLine(const LineSegment2D<T>& line, double angle)
{
	LineSegment2D<T> tem = line;
	tem.Rotate(angle);

	return tem;
}
template <class T> inline LineSegment2D<T> TranslateLine(const LineSegment2D<T>& line, const T x, const T y)
{
	LineSegment2D<T> tem = line;
	tem.Translate(x, y);

	return tem;
}
template <class T> inline LineSegment2D<T> TranslateLine(const LineSegment2D<T>& line, const cv::Vec<T, 2>& vec)
{
	LineSegment2D<T> tem = line;
	tem.Translate(vec);

	return tem;
}

template <class T> inline bool IsIntersect(const LineSegment2D<T>& l1, const LineSegment2D<T>& l2)
{
	cv::Point_<T> a = l1.P1();
	cv::Point_<T> b = l1.P2();
	cv::Point_<T> c = l2.P1();
	cv::Point_<T> d = l2.P2();
	return (CCW_test2d(a,c,d)!=CCW_test2d(b,c,d) && CCW_test2d(a,b,c)!=CCW_test2d(a,b,d));
}

template <class T> inline bool IsParallel(const LineSegment2D<T>& l1, const LineSegment2D<T>& l2)
{
	return (abs(l1.theta-l2.theta)<linesegment_theta_thr) ? true : false;
}

template <class T> inline bool IsSame(const LineSegment2D<T>& l1, const LineSegment2D<T>& l2)
{	
	return (abs(l1.theta-l2.theta)<linesegment_theta_thr && abs(l1.rho-l2.rho)<linesegment_rho_thr) ? true : false;	
}

template <class T> inline double MindistLine(const LineSegment2D<T>& l1, const LineSegment2D<T>& l2)
{
	if (IsIntersect(l1, l2))
	{
		return 0;
	}

	double d_theta = abs(l1.theta-l2.theta);
	double d_rho = abs(l1.rho-l2.rho);
	std::vector<double> v(4);

	if (d_theta < linesegment_theta_thr && d_rho < linesegment_rho_thr)
	{
		Point_<T> d1 = l2.P1() - l1.P1();
		Point_<T> d2 = l2.P2() - l1.P1();
		Point_<T> d3 = l2.P1() - l1.P2();
		Point_<T> d4 = l2.P2() - l1.P2();

		T v1 = d1.dot(d2);
		T v2 = d3.dot(d4);
		if (v1<0 || v2<0)
		{
			return 0;
		}
		else
		{
			v[0] = norm(l1.P1()-l2.P1());
			v[1] = norm(l1.P1()-l2.P2());
			v[2] = norm(l1.P2()-l2.P1());
			v[3] = norm(l1.P2()-l2.P2());
			return *min_element(v.begin(), v.end());
		}
	}
	else
	{
		v[0] = l1.MinDistFromPt(l2.P1());
		v[1] = l1.MinDistFromPt(l2.P1());
		v[2] = l2.MinDistFromPt(l1.P2()); 
		v[3] = l2.MinDistFromPt(l1.P2());
		return *min_element(v.begin(), v.end());
	}
}


typedef  LineSegment2D<float> LineSegment2Df;
typedef  LineSegment2D<double> LineSegment2Dd;
typedef  LineSegment2D<int> LineSegment2Di;

#endif // LINESEGMENT_H__