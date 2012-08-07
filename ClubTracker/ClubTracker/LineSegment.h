#ifndef LINESEGMENT_H__
#define LINESEGMENT_H__

#include <opencv2/opencv.hpp>

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
		double dx = double(line_[0]-line_[2]);
		double dy = double(line_[1]-line_[3]);
		length = sqrt(dx*dx+dy*dy);
	}
 	~LineSegment2D()
	{
	}

	cv::Point_<T> P1()
	{
		return cv::Point_<T>(line[0], line[1]);
	}
	cv::Point_<T> P2()
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
	double DistFromPt(const cv::Point_<T>& pt)
	{
		return abs(double(pt.x)*implicit_param[0]+double(pt.y)*implicit_param[1]+implicit_param[2]);
	}

	cv::Vec<T, 4> line;	
	cv::Point_<T> dir;
	cv::Point_<T> center;

	cv::Vec3d implicit_param;
 	double rho;
 	double theta;
	double length;

	LineSegment2D<T> operator=(const LineSegment2D<T>& line_)							// assign operator
	{
		this->line  = line_.line;
		this->dir = line_.dir;
		this->center = line_.center;

		this->implicit_param = line_.implicit_param;
		this->rho  = line_.rho;
		this->theta = line_.theta;
		this->length = line_.length;

		return *this;
	}


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

template <class T> inline LineSegment2D<T> Reverse(LineSegment2D<T>& line)
{
	line.Reverse();
}
template <class T> inline LineSegment2D<T> Rotate(const LineSegment2D<T>& line, double angle)
{
	LineSegment2D<T> tem = line;
	tem.Rotate(angle);

	return tem;
}
template <class T> inline LineSegment2D<T> Translate(const LineSegment2D<T>& line, const T x, const T y)
{
	LineSegment2D<T> tem = line;
	tem.Translate(x, y);

	return tem;
}
template <class T> inline LineSegment2D<T> Translate(const LineSegment2D<T>& line, const cv::Vec<T, 2>& vec)
{
	LineSegment2D<T> tem = line;
	tem.Translate(vec);

	return tem;
}


typedef  LineSegment2D<float> LineSegment2Df;
typedef  LineSegment2D<double> LineSegment2Dd;
typedef  LineSegment2D<int> LineSegment2Di;

#endif // LINESEGMENT_H__