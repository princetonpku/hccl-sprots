#ifndef HCCL_VECTORQUATERNION_H_
#define HCCL_VECTORQUATERNION_H_

#include <memory>
#include <math.h>
#include <algorithm>

#include <iostream>

#ifndef M_PI
#define _MATH_DEFINES_DEFINED
#define M_PI 3.14159265358979323846
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif
#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif

template<class T> class Vector2;
template<class T> class Quaternion;

template <class T>
class Vector3
{
// Constructors
public:
	Vector3()															// zero vector
	{
		memset(val, 0, sizeof(T)*3);
	}
	Vector3(T x, T y, T z)												// (x, y, z)
	{
		val[0] = x;	val[1] = y;	val[2] = z;
	}
	Vector3(T* xyz)														// (xyz[0], xyz[1], xyz[2])
	{
		memcpy(val, xyz, sizeof(T)*3);
	}
	Vector3(const Vector3<T>& v)										// copy constructor
	{
		memcpy(this->val, v.val, sizeof(T)*3);
	}
	Vector3(const Quaternion<T>& q)										// typecast constructor
	{
		memcpy(this->val, q.val+1, sizeof(T)*3);
	}
	~Vector3(){}

// Methods (Algebra)
public:
	T Dot(const Vector3<T>& v) const									// this . v
	{
		return this->val[0]*v.val[0] + this->val[1]*v.val[1] + this->val[2]*v.val[2];
	}
	Vector3<T> Cross(const Vector3<T>& v) const							// this x v
	{
		return Vector3<T>(this->val[1]*v.val[2] - this->val[2]*v.val[1],
			this->val[2]*v.val[0] - this->val[0]*v.val[2],
			this->val[0]*v.val[1] - this->val[1]*v.val[0]);
	}
	T NormSquared() const												// |this|^2
	{
		return this->val[0]*this->val[0] + this->val[1]*this->val[1] + this->val[2]*this->val[2];
	}
	T Norm() const														// |this|
	{
		return sqrt(this->val[0]*this->val[0] + this->val[1]*this->val[1] + this->val[2]*this->val[2]);
	}
	Vector3<T> Normalize(void)											// this /= |this|
	{
		T n = this->Norm();
		this->val[0] /= n; this->val[1] /= n; this->val[2] /= n;
		return *this;
	}

	Vector3<T> Add(const Vector3<T>& v)									// this += v
	{
		this->val[0] += v.val[0];		this->val[1] += v.val[1];		this->val[2] += v.val[2];
		return *this;
	}
	Vector3<T> Add(const T& k)											// this += k
	{
		this->val[0] += k;		this->val[1] += k;		this->val[2] += k;
		return *this;
	}
	Vector3<T> Sub(const Vector3<T>& v)									// this -= v
	{
		this->val[0] -= v[0];		this->val[1] -= v[1];		this->val[2] -= v[2];
		return *this;
	}
	Vector3<T> Sub(const T& k)											// this -= k
	{
		this->val[0] -= k;		this->val[1] -= k;		this->val[2] -= k;
		return *this;
	}
	Vector3<T> Mul(const Vector3<T>& v)									// this *= v (element-wise)
	{
		this->val[0] *= v[0];		this->val[1] *= v[1];		this->val[2] *= v[2];
		return *this;
	}
	Vector3<T> Mul(const T& k)											// this *= k
	{
		this->val[0] *= k;		this->val[1] *= k;		this->val[2] *= k;
		return *this;
	}
	Vector3<T> Div(const Vector3<T>& v)									// this /= v (element-wise)
	{
		this->val[0] /= v[0];		this->val[1] /= v[1];		this->val[2] /= v[2];
		return *this;
	}
	Vector3<T> Div(const T& k)											// this /= k
	{
		this->val[0] /= k;		this->val[1] /= k;		this->val[2] /= k;
		return *this;
	}

// Operators
public:
	Vector3<T> operator=(const Vector3<T>& v)							// assign operator
	{
		memcpy(this->val, v.val, sizeof(T)*3);
		return *this;
	}

	bool operator==(const Vector3<T>& v) const							// compare operator
	{
		return (this->val[0] == v.val[0] && this->val[1] == v.val[1] && this->val[2] == v.val[2]);
	}
	bool operator!=(const Vector3<T>& v) const							// negative compare operator
	{	
		return (this->val[0] != v.val[0] || this->val[1] != v.val[1] || this->val[2] != v.val[2]);
	}

	Vector3<T> operator+=(const Vector3<T>& v)							// unary addition operator
	{
		this->val[0] += v.val[0];	this->val[1] += v.val[1];	this->val[2] += v.val[2];
		return *this;
	}
	Vector3<T> operator+=(const T& k)									// unary scalar addition operator
	{
		this->val[0] += k;		this->val[1] += k;		this->val[2] += k;
		return *this;
	}
	Vector3<T> operator-=(const Vector3<T>& v)							// unary subtraction operator
	{
		this->val[0] -= v.val[0];	this->val[1] -= v.val[1];	this->val[2] -= v.val[2];
		return *this;
	}
	Vector3<T> operator-=(const T& k)									// unary scalar subtraction operator
	{
		this->val[0] -= k;		this->val[1] -= k;		this->val[2] -= k;
		return *this;
	}
	Vector3<T> operator*=(const Vector3<T>& v)							// unary multiplication operator (element-wise)
	{
		this->val[0] *= v[0];		this->val[1] *= v[1];		this->val[2] *= v[2];
		return *this;
	}
	Vector3<T> operator*=(const T& k)									// unary scalar multiplication operator
	{
		this->val[0] *= k;		this->val[1] *= k;		this->val[2] *= k;
		return *this;
	}
	Vector3<T> operator/=(const Vector3<T>& v)							// unary division operator (element-wise)
	{
		this->val[0] /= v[0];		this->val[1] /= v[1];		this->val[2] /= v[2];
		return *this;
	}
	Vector3<T> operator/=(const T& k)									// unary scalar division operator
	{
		this->val[0] /= k;		this->val[1] /= k;		this->val[2] /= k;
		return *this;
	}

	Vector3<T> operator-() const										// unary negation operator
	{
		return Vector3<T>(-this->val[0], -this->val[1], -this->val[2]);
	}



// Methods (Geometry)
public:
	Vector3<T> Rotate(const T rad, const Vector3<T>& axis)				// rotate this vector by given angle(rad) along the axis(must be unit)
	{
	}
	Vector3<T> Rotate(const Vector3<T>& axis)							// rotate this vector by given angle(norm of axis) along the axis
	{
	}
	Vector3<T> Rotate(const T* R)										// rotate this vector by multiplying the rotation matrix R (column-stacked)
	{
	}
	Vector3<T> Rotate(const Quaternion<T>& q)
	{
	}
	Vector3<T> Translate(const Vector3<T>& dx)							// this += dx
	{
	}
	Vector3<T> Translate(const T mag, const Vector3<T>& dir)			// this += mag*dir
	{
	}

	static T Angle(const Vector3<T>& a, const Vector3<T>& b)			// get angle between two vectors
	{
	}
	T Angle(const Vector3<T>& a)										// get angle between this and a
	{
	}


// Methods (Auxiliary)
public:
	template <class T> friend std::istream& operator>>(std::istream& is, Vector3<T>& v)
	{
		char str[1024];
		char* tok;
		is >> str;
		tok = strtok(str, " ,()<>[]|:");
		v.val[0] = (T)atof(tok);
		tok = strtok(NULL, " ,()<>[]|:");
		v.val[1] = (T)atof(tok);
		tok = strtok(NULL, " ,()<>[]|:");
		v.val[2] = (T)atof(tok);
		return is;
	}
	template <class T> friend std::ostream& operator<<(std::ostream& os, const Vector3<T>& v)
	{
		os << "(" << v.val[0] << ", " << v.val[1] << ", " << v.val[2] << ")";
		return os;
	}

// Accessors
public:
	T& operator[](int i)	{	return val[i];	}
	const T& operator[](int i) const	{	return val[i];	}
	T X() const				{	return val[0];	}
	T Y() const				{	return val[1];	}
	T Z() const				{	return val[2];	}

	void SetX(T nx)			{	val[0] = nx;	}
	void SetY(T ny)			{	val[1] = ny;	}
	void SetZ(T nz)			{	val[2] = nz;	}

public:
	T val[3];
};

template <class T> inline T Dot(const Vector3<T>& a, const Vector3<T>& b)					// a . b
{
	return a.val[0]*b.val[0] + a.val[1]*b.val[1] + a.val[2]*b.val[2];
}
template <class T> inline Vector3<T> Cross(const Vector3<T>& a, const Vector3<T>& b)		// a x b
{
	return Vector3<T>(a.val[1]*b.val[2] - a.val[2]*b.val[1],
		a.val[2]*b.val[0] - a.val[0]*b.val[2],
		a.val[0]*b.val[1] - a.val[1]*b.val[0]);
}
template <class T> inline T NormSquared(const Vector3<T>& v)								// |v|^2
{
	return v.val[0]*v.val[0] + v.val[1]*v.val[1] + v.val[2]*v.val[2];
}
template <class T> inline T Norm(const Vector3<T>& v)										// |v|
{
	return sqrt(v.val[0]*v.val[0] + v.val[1]*v.val[1] + v.val[2]*v.val[2]);
}
template <class T> inline Vector3<T> Normalize(Vector3<T>& v)								// v / |v|
{
	T n = Norm(v);
	v.val[0] /= n; v.val[1] /= n; v.val[2] /= n;
	return v;
}

template <class T> inline Vector3<T> Add(const Vector3<T>& a, const Vector3<T>& b)			// a + b
{
	return Vector3<T>(a.val[0] + b.val[0], a.val[1] + b.val[1], a.val[2] + b.val[2]);
}
template <class T> inline Vector3<T> Add(const Vector3<T>& a, const T& k)					// a + k
{
	return Vector3<T>(a.val[0] + k, a.val[1] + k, a.val[2] + k);
}
template <class T> inline Vector3<T> Add(const T& k, const Vector3<T>& a)					// k + a
{
	return Vector3<T>(a.val[0] + k, a.val[1] + k, a.val[2] + k);
}
template <class T> inline Vector3<T> Sub(const Vector3<T>& a, const Vector3<T>& b)			// a - b
{
	return Vector3<T>(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2]);
}
template <class T> inline Vector3<T> Sub(const Vector3<T>& a, const T& k)					// a - k
{
	return Vector3<T>(a.val[0] - k, a.val[1] - k, a.val[2] - k);
}
template <class T> inline Vector3<T> Sub(const T& k, const Vector3<T>& a)					// k - a
{
	return Vector3<T>(k - a.val[0], k - a.val[1], k - a.val[2]);
}
template <class T> inline Vector3<T> Mul(const Vector3<T>& a, const Vector3<T>& b)			// a * b (element-wise)
{
	return Vector3<T>(a.val[0] * b.val[0], a.val[1] * b.val[1], a.val[2] * b.val[2]);
}
template <class T> inline Vector3<T> Mul(const Vector3<T>& a, const T& k)					// a * k
{
	return Vector3<T>(a.val[0] * k, a.val[1] * k, a.val[2] * k);
}
template <class T> inline Vector3<T> Mul(const T& k, const Vector3<T>& a)					// k * a
{
	return Vector3<T>(a.val[0] * k, a.val[1] * k, a.val[2] * k);
}
template <class T> inline Vector3<T> Div(const Vector3<T>& a, const Vector3<T>& b)			// a / b (element-wise)
{
	return Vector3<T>(a.val[0] / b.val[0], a.val[1] / b.val[1], a.val[2] / b.val[2]);
}
template <class T> inline Vector3<T> Div(const Vector3<T>& a, const T& k)					// a / k
{
	return Vector3<T>(a.val[0] / k, a.val[1] / k, a.val[2] / k);
}
template <class T> inline Vector3<T> Div(const T& k, const Vector3<T>& a)					// (k / a_x, k / a_y, k / a_z)
{
	return Vector3<T>(k / a.val[0], k / a.val[1], k / a.val[2]);
}

template <class T> inline Vector3<T> operator+(const Vector3<T>& a, const Vector3<T>& b)	// binary addition operator
{
	return Vector3<T>(a.val[0] + b.val[0], a.val[1] + b.val[1], a.val[2] + b.val[2]);
}
template <class T> inline Vector3<T> operator+(const Vector3<T>& a, const T& k)				// binary scalar addition operator
{
	return Vector3<T>(a.val[0] + k, a.val[1] + k, a.val[2] + k);
}
template <class T> inline Vector3<T> operator+(const T& k, const Vector3<T>& a)				// binary scalar addition operator
{
	return Vector3<T>(a.val[0] + k, a.val[1] + k, a.val[2] + k);
}
template <class T> inline Vector3<T> operator-(const Vector3<T>& a, const Vector3<T>& b)	// binary subtraction operator
{
	return Vector3<T>(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2]);
}
template <class T> inline Vector3<T> operator-(const Vector3<T>& a, const T& k)				// binary scalar subtraction operator
{
	return Vector3<T>(a.val[0] - k, a.val[1] - k, a.val[2] - k);
}
template <class T> inline Vector3<T> operator-(const T& k, const Vector3<T>& a)				// binary scalar subtraction operator
{
	return Vector3<T>(k - a.val[0], k - a.val[1], k - a.val[2]);
}
template <class T> inline Vector3<T> operator*(const Vector3<T>& a, const Vector3<T>& b)	// binary multiplication operator (element-wise)
{
	return Vector3<T>(a.val[0] * b.val[0], a.val[1] * b.val[1], a.val[2] * b.val[2]);
}
template <class T> inline Vector3<T> operator*(const Vector3<T>& v, const T& k)				// binary scalar multiplication operator
{
	return Vector3<T>(v.val[0] * k, v.val[1] * k, v.val[2] * k);
}
template <class T> inline Vector3<T> operator*(const T& k, const Vector3<T>& v)				// binary scalar multiplication operator
{
	return Vector3<T>(v.val[0] * k, v.val[1] * k, v.val[2] * k);
}
template <class T> inline Vector3<T> operator/(const Vector3<T>& a, const Vector3<T>& b)	// binary division operator (element-wise)
{
	return Vector3<T>(a.val[0] / b.val[0], a.val[1] / b.val[1], a.val[2] / b.val[2]);
}
template <class T> inline Vector3<T> operator/(const Vector3<T>& v, const T& k)				// binary scalar division operator
{
	return Vector3<T>(v.val[0] / k, v.val[1] / k, v.val[2] / k);
}
template <class T> inline Vector3<T> operator/(const T& k, const Vector3<T>& v)				// binary scalar division operator
{
	return Vector3<T>(k / v.val[0], k / v.val[1], k / v.val[2]);
}

typedef Vector3<int>	Vector3i;
typedef Vector3<float>	Vector3f;
typedef Vector3<double>	Vector3d;


template <class T>
class Vector2
{
// Constructors
public:
	Vector2()															// zero vector
	{
		memset(val, 0, sizeof(T)*2);
	}
	Vector2(T x, T y)													// (x, y)
	{
		val[0] = x;	val[1] = y;
	}
	Vector2(T* xy)														// (xy[0], xy[1])
	{
		memcpy(val, xyz, sizeof(T)*2);
	}
	Vector2(const Vector2<T>& v)										// copy constructor
	{
		memcpy(this->val, v.val, sizeof(T)*2);
	}
// 	Vector2(const Quaternion<T>& q)										// typecast constructor
// 	{
// 		memcpy(this->val, q.val+1, sizeof(T)*2);
// 	}
	~Vector2(){}

// Methods (Algebra)
public:
	T Dot(const Vector2<T>& v) const									// this . v
	{
		return this->val[0]*v.val[0] + this->val[1]*v.val[1];
	}
	T Cross(const Vector2<T>& v) const									// this x v
	{
		return this->val[0]*v.val[1] - this->val[1]*v.val[0];			
	}
	T NormSquared() const												// |this|^2
	{
		return this->val[0]*this->val[0] + this->val[1]*this->val[1];
	}
	T Norm() const														// |this|
	{
		return sqrt(this->val[0]*this->val[0] + this->val[1]*this->val[1]);
	}
	Vector2<T> Normalize(void)											// this /= |this|
	{
		T n = this->Norm();
		this->val[0] /= n; this->val[1] /= n;
		return *this;
	}

	Vector2<T> Add(const Vector2<T>& v)									// this += v
	{
		this->val[0] += v.val[0];		this->val[1] += v.val[1];
		return *this;
	}
	Vector2<T> Add(const T& k)											// this += k
	{
		this->val[0] += k;		this->val[1] += k;
		return *this;
	}
	Vector2<T> Sub(const Vector2<T>& v)									// this -= v
	{
		this->val[0] -= v.val[0];		this->val[1] -= v.val[1];
		return *this;
	}
	Vector2<T> Sub(const T& k)											// this -= k
	{
		this->val[0] -= k;		this->val[1] -= k;
		return *this;
	}
	Vector2<T> Mul(const Vector2<T>& v)									// this *= v (element-wise)
	{
		this->val[0] *= v.val[0];		this->val[1] *= v.val[1];
		return *this;
	}
	Vector2<T> Mul(const T& k)											// this *= k
	{
		this->val[0] *= k;		this->val[1] *= k;
		return *this;
	}
	Vector2<T> Div(const Vector2<T>& v)									// this /= v (element-wise)
	{
		this->val[0] /= v.val[0];		this->val[1] /= v.val[1];
		return *this;
	}
	Vector2<T> Div(const T& k)											// this /= k
	{
		this->val[0] /= k;		this->val[1] /= k;
		return *this;
	}

// Operators
public:
	Vector2<T> operator=(const Vector2<T>& v)							// assign operator
	{
		memcpy(this->val, v.val, sizeof(T)*2);
		return *this;
	}

	bool operator==(const Vector2<T>& v) const							// compare operator
	{
		return (this->val[0] == v.val[0] && this->val[1] == v.val[1]);
	}
	bool operator!=(const Vector2<T>& v) const							// negative compare operator
	{	
		return (this->val[0] != v.val[0] || this->val[1] != v.val[1]);
	}

	Vector2<T> operator+=(const Vector2<T>& v)							// unary addition operator
	{
		this->val[0] += v.val[0];		this->val[1] += v.val[1];
		return *this;
	}
	Vector2<T> operator+=(const T& k)									// unary scalar addition operator
	{
		this->val[0] += k;		this->val[1] += k;
		return *this;
	}
	Vector2<T> operator-=(const Vector2<T>& v)							// unary subtraction operator
	{
		this->val[0] -= v.val[0];		this->val[1] -= v.val[1];
		return *this;
	}
	Vector2<T> operator-=(const T& k)									// unary scalar subtraction operator
	{
		this->val[0] -= k;		this->val[1] -= k;
		return *this;
	}
	Vector2<T> operator*=(const Vector2<T>& v)							// unary multiplication operator (element-wise)
	{
		this->val[0] *= v.val[0];		this->val[1] *= v.val[1];
		return *this;
	}
	Vector2<T> operator*=(const T& k)									// unary scalar multiplication operator
	{
		this->val[0] *= k;		this->val[1] *= k;
		return *this;
	}
	Vector2<T> operator/=(const Vector2<T>& v)							// unary division operator (element-wise)
	{
		this->val[0] /= v.val[0];		this->val[1] /= v.val[1];
		return *this;
	}
	Vector2<T> operator/=(const T& k)									// unary scalar division operator
	{
		this->val[0] /= k;		this->val[1] /= k;
		return *this;
	}

	Vector2<T> operator-() const										// unary negation operator
	{
		return Vector2<T>(-this->val[0], -this->val[1]);
	}



// Methods (Geometry)
public:
	Vector2<T> Rotate(const T rad, const Vector2<T>& axis)				// rotate this vector by given angle(rad) along the axis(must be unit)
	{
	}
	Vector2<T> Rotate(const Vector2<T>& axis)							// rotate this vector by given angle(norm of axis) along the axis
	{
	}
	Vector2<T> Rotate(const T* R)										// rotate this vector by multiplying the rotation matrix R (column-stacked)
	{
	}
	Vector2<T> Rotate(const Quaternion<T>& q)
	{
	}
	Vector2<T> Translate(const Vector2<T>& dx)							// this += dx
	{
	}
	Vector2<T> Translate(const T mag, const Vector2<T>& dir)			// this += mag*dir
	{
	}

	static T Angle(const Vector2<T>& a, const Vector2<T>& b)			// get angle between two vectors
	{
	}
	T Angle(const Vector2<T>& a)										// get angle between this and a
	{
	}


// Methods (Auxiliary)
public:
	template <class T> friend std::istream& operator>>(std::istream& is, Vector2<T>& v)
	{
		char str[1024];
		char* tok;
		is >> str;
		tok = strtok(str, " ,()<>[]|:");
		v.val[0] = (T)atof(tok);
		tok = strtok(NULL, " ,()<>[]|:");
		v.val[1] = (T)atof(tok);		
		return is;
	}
	template <class T> friend std::ostream& operator<<(std::ostream& os, const Vector2<T>& v)
	{
		os << "(" << v.val[0] << ", " << v.val[1] << ")";
		return os;
	}

	// Accessors
public:
	T& operator[](int i)	{	return val[i];	}
	const T& operator[](int i) const	{	return val[i];	}
	T X() const				{	return val[0];	}
	T Y() const				{	return val[1];	}	

	void SetX(T nx)			{	val[0] = nx;	}
	void SetY(T ny)			{	val[1] = ny;	}	

public:
	T val[2];
};

template <class T> inline T Dot(const Vector2<T>& a, const Vector2<T>& b)					// a . b
{
	return sqrt(a.val[0]*b.val[0] + a.val[1]*b.val[1]);
}
template <class T> inline T NormSquared(const Vector2<T>& v)								// |v|^2
{
	return v.val[0]*v.val[0] + v.val[1]*v.val[1];
}
template <class T> inline T Norm(const Vector2<T>& v)										// |v|
{
	return sqrt(v.val[0]*v.val[0] + v.val[1]*v.val[1]);
}
template <class T> inline Vector2<T> Normalize(Vector2<T>& v)								// v / |v|
{
	T n = Norm(v);
	v.val[0] /= n; v.val[1] /= n;
	return v;
}

template <class T> inline Vector2<T> Add(const Vector2<T>& a, const Vector2<T>& b)			// a + b
{
	return Vector2<T>(a.val[0] + b.val[0], a.val[1] + b.val[1]);
}
template <class T> inline Vector2<T> Add(const Vector2<T>& a, const T& k)					// a + k
{
	return Vector2<T>(a.val[0] + k, a.val[1] + k);
}
template <class T> inline Vector2<T> Add(const T& k, const Vector2<T>& a)					// k + a
{
	return Vector2<T>(a.val[0] + k, a.val[1] + k);
}
template <class T> inline Vector2<T> Sub(const Vector2<T>& a, const Vector2<T>& b)			// a - b
{
	return Vector2<T>(a.val[0] - b.val[0], a.val[1] - b.val[1]);
}
template <class T> inline Vector2<T> Sub(const Vector2<T>& a, const T& k)					// a - k
{
	return Vector2<T>(a.val[0] - k, a.val[1] - k);
}
template <class T> inline Vector2<T> Sub(const T& k, const Vector2<T>& a)					// k - a
{
	return Vector2<T>(k - a.val[0], k - a.val[1]);
}
template <class T> inline Vector2<T> Mul(const Vector2<T>& a, const Vector2<T>& b)			// a * b (element-wise)
{
	return Vector2<T>(a.val[0] * b.val[0], a.val[1] * b.val[1]);
}
template <class T> inline Vector2<T> Mul(const Vector2<T>& a, const T& k)					// a * k
{
	return Vector2<T>(a.val[0] * k, a.val[1] * k);
}
template <class T> inline Vector2<T> Mul(const T& k, const Vector2<T>& a)					// k * a
{
	return Vector2<T>(a.val[0] * k, a.val[1] * k);
}
template <class T> inline Vector2<T> Div(const Vector2<T>& a, const Vector2<T>& b)			// a / b (element-wise)
{
	return Vector2<T>(a.val[0] / b.val[0], a.val[1] / b.val[1]);
}
template <class T> inline Vector2<T> Div(const Vector2<T>& a, const T& k)					// a / k
{
	return Vector2<T>(a.val[0] / k, a.val[1] / k);
}
template <class T> inline Vector2<T> Div(const T& k, const Vector2<T>& a)					// (k / a_x, k / a_y)
{
	return Vector2<T>(k / a.val[0], k / a.val[1]);
}

template <class T> inline Vector2<T> operator+(const Vector2<T>& a, const Vector2<T>& b)	// binary addition operator
{
	return Vector2<T>(a.val[0] + b.val[0], a.val[1] + b.val[1]);
}
template <class T> inline Vector2<T> operator+(const Vector2<T>& a, const T& k)				// binary scalar addition operator
{
	return Vector2<T>(a.val[0] + k, a.val[1] + k);
}
template <class T> inline Vector2<T> operator+(const T& k, const Vector2<T>& a)				// binary scalar addition operator
{
	return Vector2<T>(a.val[0] + k, a.val[1] + k);
}
template <class T> inline Vector2<T> operator-(const Vector2<T>& a, const Vector2<T>& b)	// binary subtraction operator
{
	return Vector2<T>(a.val[0] - b.val[0], a.val[1] - b.val[1]);
}
template <class T> inline Vector2<T> operator-(const Vector2<T>& a, const T& k)				// binary scalar subtraction operator
{
	return Vector2<T>(a.val[0] - k, a.val[1] - k);
}
template <class T> inline Vector2<T> operator-(const T& k, const Vector2<T>& a)				// binary scalar subtraction operator
{
	return Vector2<T>(k - a.val[0], k - a.val[1]);
}
template <class T> inline Vector2<T> operator*(const Vector2<T>& a, const Vector2<T>& b)	// binary multiplication operator (element-wise)
{
	return Vector2<T>(a.val[0] * b.val[0], a.val[1] * b.val[1]);
}
template <class T> inline Vector2<T> operator*(const Vector2<T>& v, const T& k)				// binary scalar multiplication operator
{
	return Vector2<T>(a.val[0] * k, a.val[1] * k);
}
template <class T> inline Vector2<T> operator*(const T& k, const Vector2<T>& v)				// binary scalar multiplication operator
{
	return Vector2<T>(a.val[0] * k, a.val[1] * k);
}
template <class T> inline Vector2<T> operator/(const Vector2<T>& a, const Vector2<T>& b)	// binary division operator (element-wise)
{
	return Vector2<T>(a.val[0] / b.val[0], a.val[1] / b.val[1]);
}
template <class T> inline Vector2<T> operator/(const Vector2<T>& v, const T& k)				// binary scalar division operator
{
	return Vector2<T>(a.val[0] / k, a.val[1] / k);
}
template <class T> inline Vector2<T> operator/(const T& k, const Vector2<T>& v)				// binary scalar division operator
{
	return Vector2<T>(k / a.val[0], k / a.val[1]);
}

typedef Vector2<int>	Vector2i;
typedef Vector2<float>	Vector2f;
typedef Vector2<double>	Vector2d;



template <class T>
class Quaternion
{
// Constructors
public:
	Quaternion()															// Identity quaternion (1, 0, 0, 0)
	{
		val[0] = 1;
		val[1] = val[2] = val[3] = 0;
	}
	Quaternion(T w, T x, T y, T z)											// (w, x, y, z)
	{
		val[0] = w, val[1] = x;	val[2] = y;	val[3] = z;
	}
	Quaternion(T* wxyz)														// (wxyz[0], wxyz[1], wxyz[2], wxyz[3])
	{
		memcpy(val, wxyz, sizeof(T)*4);
	}
	Quaternion(const Quaternion<T>& v)										// copy constructor
	{
		memcpy(this->val, v.val, sizeof(T)*4);
	}
	Quaternion(const T rad, const Vector3<T>& axis)							// from axis angle
	{		
		T m = Norm(axis);

		if(m == 0) // identity
		{
			val[0] = 1;
			val[1] = val[2] = val[3] = 0;
		}		
		else
		{
 			T sm = sin(axis[0]*0.5)/m;
			val[0] = cos(axis[0]*0.5);
			val[1] = axis[1]*sm;
			val[2] = axis[2]*sm;
			val[3] = axis[3]*sm;

			Normalize();				// Do I have to normalize q here?
		}
	}
	Quaternion(const T* R)								// from rotation matrix(3 by 3)
	{													// R is 3x3 matrix, in stacked-column form. i.e. R_(i,j) = R[3*j + i]
		T tr = R[0] + R[4] + R[8];
		if( tr > 0 )
		{
			T s = 0.5 / sqrt(tr + 1.0);
			val[0] = 0.25 / s;
			val[1] = ( R[5] - R[7] ) * s;			// (R21 - R12)*s
			val[2] = ( R[6] - R[2] ) * s;			// (R02 - R20)*s
			val[3] = ( R[1] - R[3] ) * s;			// (R10 - R01)*s
		}
		else
		{
			if( R[0] > R[4] && R[0] > R[8] )	// R00 > R11 && R00 > R22
			{
				T s = 0.5 / sqrt(1.0 + R[0] - R[4] - R[8]);
				val[0] = ( R[5] - R[7] ) * s;		// (R21 - R12)*s
				val[1] = 0.25 / s;
				val[2] = ( R[3] + R[1] ) * s;		// (R01 + R10)*s
				val[3] = ( R[6] + R[2] ) * s;		// (R02 + R20)*s
			}
			else if (R[4] > R[8])
			{
				T s = 0.5 / sqrt(1.0 + R[4] - R[0] - R[8]);
				val[0] = ( R[6] - R[2] ) * s;		// (R02 - R20)*s
				val[1] = ( R[3] + R[1] ) * s;		// (R01 + R10)*s
				val[2] = 0.25 / s;
				val[3] = ( R[7] + R[5] ) * s;		// (R12 + R21)*s
			}
			else
			{
				T s = 0.5 / sqrt(1.0 + R[8] - R[0] - R[4]);
				val[0] = ( R[1] - R[3] ) * s;		// (R10 - R01)*s
				val[1] = ( R[6] + R[2] ) * s;		// (R02 + R20)*s
				val[2] = ( R[7] + R[5] ) * s;		// (R12 + R21)*s
				val[3] = 0.25f / s;
			}
		}
	}
	Quaternion(const T R[3][3])							// from rotation matrix(3 by 3)
	{													// R is 3x3 matrix, in stacked-row form. i.e. R_(i,j) = R[3*i + j]
		T Rtemp[9] = {R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]};
		T tr = Rtemp[0] + Rtemp[4] + Rtemp[8];
		if( tr > 0 )
		{
			T s = 0.5 / sqrt(tr + 1.0);
			val[0] = 0.25 / s;
			val[1] = ( Rtemp[5] - Rtemp[7] ) * s;			// (R21 - R12)*s
			val[2] = ( Rtemp[6] - Rtemp[2] ) * s;			// (R02 - R20)*s
			val[3] = ( Rtemp[1] - Rtemp[3] ) * s;			// (R10 - R01)*s
		}
		else
		{
			if( Rtemp[0] > Rtemp[4] && Rtemp[0] > Rtemp[8] )	// R00 > R11 && R00 > R22
			{
				T s = 0.5 / sqrt(1.0 + Rtemp[0] - Rtemp[4] - Rtemp[8]);
				val[0] = ( Rtemp[5] - Rtemp[7] ) * s;		// (R21 - R12)*s
				val[1] = 0.25 / s;
				val[2] = ( Rtemp[3] + Rtemp[1] ) * s;		// (R01 + R10)*s
				val[3] = ( Rtemp[6] + Rtemp[2] ) * s;		// (R02 + R20)*s
			}
			else if (Rtemp[4] > Rtemp[8])
			{
				T s = 0.5 / sqrt(1.0 + Rtemp[4] - Rtemp[0] - Rtemp[8]);
				val[0] = ( Rtemp[6] - Rtemp[2] ) * s;		// (R02 - R20)*s
				val[1] = ( Rtemp[3] + Rtemp[1] ) * s;		// (R01 + R10)*s
				val[2] = 0.25 / s;
				val[3] = ( Rtemp[7] + Rtemp[5] ) * s;		// (R12 + R21)*s
			}
			else
			{
				T s = 0.5 / sqrt(1.0 + Rtemp[8] - Rtemp[0] - Rtemp[4]);
				val[0] = ( Rtemp[1] - Rtemp[3] ) * s;		// (R10 - R01)*s
				val[1] = ( Rtemp[6] + Rtemp[2] ) * s;		// (R02 + R20)*s
				val[2] = ( Rtemp[7] + Rtemp[5] ) * s;		// (R12 + R21)*s
				val[3] = 0.25f / s;
			}
		}
	}
	Quaternion(const T angles[3], const char* order)			// from Euler angle
	{
		//TODO
	}
	Quaternion(const Vector3<T>& q)											// typecast constructor
	{
		// TODO
	}
	Quaternion(const Vector2<T>& q)											// typecast constructor
	{
		// TODO
	}
	~Quaternion(){}

// Conversion ==> TODO
public:
	Quaternion<T> FromAxisAngle(const T rad, const T axis_x, const T axis_y, const T axis_z)
	{
		T m = sqrt(axis_x*axis_x + axis_y*axis_y + axis_z*axis_z);

		if(m == 0) // identity
		{
			val[0] = 1;
			val[1] = val[2] = val[3] = 0;
		}		
		else
		{
			T sm = sin(rad*0.5)/m;
			val[0] = cos(rad*0.5);
			val[1] = axis_x*sm;
			val[2] = axis_y*sm;
			val[3] = axis_z*sm;

			Normalize();				// Do I have to normalize q here?
		}

		return *this;
	}
	Quaternion<T> FromMatrix(const T* R)
	{
		T tr = R[0] + R[4] + R[8];
		if( tr > 0 )
		{
			T s = 0.5 / sqrt(tr + 1.0);
			val[0] = 0.25 / s;
			val[1] = ( R[5] - R[7] ) * s;			// (R21 - R12)*s
			val[2] = ( R[6] - R[2] ) * s;			// (R02 - R20)*s
			val[3] = ( R[1] - R[3] ) * s;			// (R10 - R01)*s
		}
		else
		{
			if( R[0] > R[4] && R[0] > R[8] )	// R00 > R11 && R00 > R22
			{
				T s = 0.5 / sqrt(1.0 + R[0] - R[4] - R[8]);
				val[0] = ( R[5] - R[7] ) * s;		// (R21 - R12)*s
				val[1] = 0.25 / s;
				val[2] = ( R[3] + R[1] ) * s;		// (R01 + R10)*s
				val[3] = ( R[6] + R[2] ) * s;		// (R02 + R20)*s
			}
			else if (R[4] > R[8])
			{
				T s = 0.5 / sqrt(1.0 + R[4] - R[0] - R[8]);
				val[0] = ( R[6] - R[2] ) * s;		// (R02 - R20)*s
				val[1] = ( R[3] + R[1] ) * s;		// (R01 + R10)*s
				val[2] = 0.25 / s;
				val[3] = ( R[7] + R[5] ) * s;		// (R12 + R21)*s
			}
			else
			{
				T s = 0.5 / sqrt(1.0 + R[8] - R[0] - R[4]);
				val[0] = ( R[1] - R[3] ) * s;		// (R10 - R01)*s
				val[1] = ( R[6] + R[2] ) * s;		// (R02 + R20)*s
				val[2] = ( R[7] + R[5] ) * s;		// (R12 + R21)*s
				val[3] = 0.25f / s;
			}
		}

		return *this;
	}
	Quaternion<T> FromMatrix(const T R[3][3])
	{
		T tr = R[0][0] + R[1][1] + R[2][2];
		if( tr > 0 )
		{
			T s = 0.5 / sqrt(tr + 1.0);
			val[0] = 0.25 / s;
			val[1] = ( R[2][1] - R[1][2] ) * s;			// (R21 - R12)*s
			val[2] = ( R[0][2] - R[2][0] ) * s;			// (R02 - R20)*s
			val[3] = ( R[1][0] - R[0][1] ) * s;			// (R10 - R01)*s
		}
		else
		{
			if( R[0][0] > R[1][1] && R[0][0] > R[2][2] )	// R00 > R11 && R00 > R22
			{
				T s = 0.5 / sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
				val[0] = ( R[2][1] - R[1][2] ) * s;		// (R21 - R12)*s
				val[1] = 0.25 / s;
				val[2] = ( R[0][1] + R[1][0] ) * s;		// (R01 + R10)*s
				val[3] = ( R[0][2] + R[2][0] ) * s;		// (R02 + R20)*s
			}
			else if (R[1][1] > R[2][2])
			{
				T s = 0.5 / sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
				val[0] = ( R[0][2] - R[2][0] ) * s;		// (R02 - R20)*s
				val[1] = ( R[0][1] + R[1][0] ) * s;		// (R01 + R10)*s
				val[2] = 0.25 / s;
				val[3] = ( R[1][2] + R[2][1] ) * s;		// (R12 + R21)*s
			}
			else
			{
				T s = 0.5 / sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
				val[0] = ( R[1][0] - R[0][1] ) * s;		// (R10 - R01)*s
				val[1] = ( R[0][2] + R[2][0] ) * s;		// (R02 + R20)*s
				val[2] = ( R[1][2] + R[2][1] ) * s;		// (R12 + R21)*s
				val[3] = 0.25f / s;
			}
		}

		return *this;
	}
	Quaternion<T> FromEulerAngle(const T a, const T b, const T c, const char* order)
	{
		//TODO
		return *this;
	}	
	Quaternion<T> Log(void) const
	{
		Quaternion<T> q(*this);
		q.Normalize();

		if( q.val[0] == 0 )
		{
			q.val[0] = 0;
			q.val[1] = 0.5*M_PI*q.val[1];
			q.val[2] = 0.5*M_PI*q.val[2];
			q.val[3] = 0.5*M_PI*q.val[3];
		}
		else if( q[0] == 1 )
		{
			q.val[0] = 0;
			q.val[1] = 0;
			q.val[2] = 0;
			q.val[3] = 0;
		}
		else
		{
			T v = sqrt(q.val[1]*q.val[1] + q.val[2]*q.val[2] + q.val[3]*q.val[3]);
			q.val[0] = 2*atan2(v, q.val[0]);
			q.val[1] = q.val[1]/v;
			q.val[2] = q.val[2]/v;
			q.val[3] = q.val[3]/v;
		}

		return q;
	}
	Quaternion<T> Exp(void) const
	{
		T m = val[1]*val[1] + val[2]*val[2] + val[3]*val[3];

		if(m == 0)
			return Quaternion<T>(1.0, 0.0, 0.0, 0.0);
		else
		{
			Quaternion<T> q;
			m = sqrt(m);
			T sm = sin(val[0]*0.5)/m;
			q.val[0] = cos(val[0]*0.5);
			q.val[1] = val[1]*sm;
			q.val[2] = val[2]*sm;
			q.val[3] = val[3]*sm;

			return q.Normalize();
		}
	}

// Methods (Algebra)
public:
	T NormSquared() const												// |this|^2
	{
		return this->val[0]*this->val[0] + this->val[1]*this->val[1] + this->val[2]*this->val[2] + this->val[3]*this->val[3];
	}
	T Norm() const														// |this|
	{
		return sqrt(this->val[0]*this->val[0] + this->val[1]*this->val[1] + this->val[2]*this->val[2] + this->val[3]*this->val[3]);
	}
	Quaternion<T> Normalize(void)											// this /= |this|
	{
		T n = this->Norm();
		this->val[0] /= n; this->val[1] /= n; this->val[2] /= n; this->val[3] /= n;
		return *this;
	}
	Quaternion<T> SetIdentity(void)
	{
		val[0] = (T)1;
		val[1] = (T)0;
		val[2] = (T)0;
		val[3] = (T)0;
	}
	Quaternion<T> Invert(void)												// inverse(this)
	{
		T n = NormSquared();
		val[0] = val[0]/n;
		val[1] = -val[1]/n;
		val[2] = -val[2]/n;
		val[3] = -val[3]/n;

		return *this;
	}
	Quaternion<T> Inv(void) const													// inverse(this)
	{
		T n = NormSquared();
		return Quaternion<T>(val[0]/n, -val[1]/n, -val[2]/n, -val[3]/n);
	}
	Quaternion<T> Add(const Quaternion<T>& q)									// this += q
	{
		this->val[0] += q[0];	this->val[1] += q[1];	this->val[2] += q[2];	this->val[3] += q[3];
		return *this;
	}
	Quaternion<T> Sub(const Quaternion<T>& q)									// this -= q
	{
		this->val[0] -= q[0];	this->val[1] -= q[1];	this->val[2] -= q[2];	this->val[3] -= q[3];
		return *this;
	}
	Quaternion<T> Mul(const Quaternion<T>& q)									// this *= q
	{ 
		T qtemp[4];
		memcpy(qtemp, this->val, sizeof(T)*4);

		val[0] = qtemp[0]*q.val[0] - qtemp[1]*q.val[1] - qtemp[2]*q.val[2] - qtemp[3]*q.val[3];
		val[1] = qtemp[1]*q.val[0] + qtemp[0]*q.val[1] - qtemp[3]*q.val[2] + qtemp[2]*q.val[3];
		val[2] = qtemp[2]*q.val[0] + qtemp[3]*q.val[1] + qtemp[0]*q.val[2] - qtemp[1]*q.val[3];
		val[3] = qtemp[3]*q.val[0] - qtemp[2]*q.val[1] + qtemp[1]*q.val[2] + qtemp[0]*q.val[3];

		return *this;
	}
	Quaternion<T> Mul(const T& k)											// this *= k
	{
		this->val[0] *= k;	this->val[1] *= k;	this->val[2] *= k;	this->val[3] *= k;
		return *this;
	}
	Quaternion<T> Div(const Quaternion<T>& q)								// this /= q
	{		
		return Mul(q.Inv());
	}
	Quaternion<T> Div(const T& k)											// this /= k
	{
		this->val[0] /= k;	this->val[1] /= k;	this->val[2] /= k;	this->val[3] /= k;
		return *this;
	}

// Operators
public:
	Quaternion<T> operator=(const Quaternion<T>& q)							// assign operator
	{
		memcpy(this->val, q.val, sizeof(T)*4);
		return *this;
	}

	bool operator==(const Quaternion<T>& q) const							// compare operator
	{
		return (this->val[0] == q.val[0] && this->val[1] == q.val[1] && this->val[2] == q.val[2] && this->val[3] == q.val[3]);
	}
	bool operator!=(const Quaternion<T>& q) const							// negative compare operator
	{	
		return (this->val[0] != q.val[0] || this->val[1] != q.val[1] || this->val[2] != q.val[2] || this->val[3] != q.val[3]);
	}

	Quaternion<T> operator+=(const Quaternion<T>& q)							// unary addition operator
	{
		this->val[0] += q[0];	this->val[1] += q[1];	this->val[2] += q[2];	this->val[3] += q[3];
		return *this;
	}
	Quaternion<T> operator-=(const Quaternion<T>& q)							// unary subtraction operator
	{
		this->val[0] -= q[0];	this->val[1] -= q[1];	this->val[2] -= q[2];	this->val[3] -= q[3];
		return *this;
	}
	Quaternion<T> operator*=(const Quaternion<T>& q)							// unary multiplication operator (element-wise)
	{
		T qtemp[4];
		memcpy(qtemp, this->val, sizeof(T)*4);

		val[0] = qtemp[0]*q.val[0] - qtemp[1]*q.val[1] - qtemp[2]*q.val[2] - qtemp[3]*q.val[3];
		val[1] = qtemp[1]*q.val[0] + qtemp[0]*q.val[1] - qtemp[3]*q.val[2] + qtemp[2]*q.val[3];
		val[2] = qtemp[2]*q.val[0] + qtemp[3]*q.val[1] + qtemp[0]*q.val[2] - qtemp[1]*q.val[3];
		val[3] = qtemp[3]*q.val[0] - qtemp[2]*q.val[1] + qtemp[1]*q.val[2] + qtemp[0]*q.val[3];

		return *this;
	}
	Quaternion<T> operator*=(const T& k)									// unary scalar multiplication operator
	{
		this->val[0] *= k;	this->val[1] *= k;	this->val[2] *= k;	this->val[3] *= k;
		return *this;
	}
	Quaternion<T> operator/=(const Quaternion<T>& q)						// unary division operator (element-wise)
	{
		return Mul(q.Inv());
	}
	Quaternion<T> operator/=(const T& k)									// unary scalar division operator
	{
		this->val[0] /= k;	this->val[1] /= k;	this->val[2] /= k;	this->val[3] /= k;
		return *this;
	}

	Quaternion<T> operator-() const											// unary negation operator
	{
		return Quaternion<T>(-this->val[0], -this->val[1], -this->val[2], -this->val[3]);
	}


// Methods (Auxiliary)
public:
	template <class T> friend std::istream& operator>>(std::istream& is, Quaternion<T>& q)
	{
		char str[1024];
		char* tok;
		is >> str;
		tok = strtok(str, " ,()<>[]|:");
		q.val[0] = (T)atof(tok);
		tok = strtok(NULL, " ,()<>[]|:");
		q.val[1] = (T)atof(tok);
		tok = strtok(NULL, " ,()<>[]|:");
		q.val[2] = (T)atof(tok);
		tok = strtok(NULL, " ,()<>[]|:");
		q.val[3] = (T)atof(tok);
		return is;
	}
	template <class T> friend std::ostream& operator<<(std::ostream& os, const Quaternion<T>& q)
	{
		os << "(" << q.val[0] << ", " << q.val[1] << ", " << q.val[2] << ", " << q.val[3] <<")";
		return os;
	}

// Accessors
public:
	T& operator[](int i)	{	return val[i];	}
	const T& operator[](int i) const	{	return val[i];	}
	T W() const				{	return val[0];	}
	T X() const				{	return val[1];	}
	T Y() const				{	return val[2];	}
	T Z() const				{	return val[3];	}

	void SetW(T nw)			{	val[0] = nw;	}
	void SetX(T nx)			{	val[1] = nx;	}
	void SetY(T ny)			{	val[2] = ny;	}
	void SetZ(T nz)			{	val[3] = nz;	}

public:
	T val[4];
};

template <class T> inline T NormSquared(const Quaternion<T>& q)								// |q|^2
{
	return q.val[0]*q.val[0] + q.val[1]*q.val[1] + q.val[2]*q.val[2] + q.val[3]*q.val[3];
}
template <class T> inline T Norm(const Quaternion<T>& q)										// |q|
{
	return sqrt(q.val[0]*q.val[0] + q.val[1]*q.val[1] + q.val[2]*q.val[2] + q.val[3]*q.val[3]);
}
template <class T> inline Quaternion<T> Normalize(Quaternion<T>& q)								// v / |v|
{
	T n = Norm(q);
	q.val[0] /= n; q.val[1] /= n; q.val[2] /= n; q.val[3] /= n;
	return q;
}
template <class T> inline Quaternion<T> Inv(const Quaternion<T>& q)								// inverse(this)
{
	T n = NormSquared(q);
	return Quaternion<T>(val[0]/n, -val[1]/n, -val[2]/n, -val[3]/n);
}

template <class T> inline Quaternion<T> Add(const Quaternion<T>& a, const Quaternion<T>& b)			// a + b
{
	return Quaternion<T>(a.val[0] + b.val[0], a.val[1] + b.val[1], a.val[2] + b.val[2], a.val[3] + b.val[3]);
}
template <class T> inline Quaternion<T> Sub(const Quaternion<T>& a, const Quaternion<T>& b)			// a - b
{
	return Quaternion<T>(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2], a.val[3] - b.val[3]);
}
template <class T> inline Quaternion<T> Mul(const Quaternion<T>& a, const Quaternion<T>& b)			// a * b
{
	return Quaternion<T>(a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
						 a[1]*b[0] + a[0]*b[1] - a[3]*b[2] + a[2]*b[3],
						 a[2]*b[0] + a[3]*b[1] + a[0]*b[2] - a[1]*b[3],
						 a[3]*b[0] - a[2]*b[1] + a[1]*b[2] + a[0]*b[3]);
}
template <class T> inline Quaternion<T> Mul(const Quaternion<T>& q, const T& k)					// a * k
{
	return Quaternion<T>(q.val[0] * k, q.val[1] * k, q.val[2] * k, q.val[3] * k);
}
template <class T> inline Quaternion<T> Mul(const T& k, const Quaternion<T>& q)					// k * a
{
	return Quaternion<T>(q.val[0] * k, q.val[1] * k, q.val[2] * k, q.val[3] * k);
}
template <class T> inline Quaternion<T> Div(const Quaternion<T>& a, const Quaternion<T>& b)			// a / b
{
	return Mul(a, Inv(b));	
}
template <class T> inline Quaternion<T> Div(const Quaternion<T>& q, const T& k)						// a / k
{
	return Quaternion<T>(q.val[0] / k, q.val[1] / k, q.val[2] / k, q.val[3] / k);
}
template <class T> inline Quaternion<T> Div(const T& k, const Quaternion<T>& q)					// (k / a_w, k / a_x, k / a_y, k / a_z)
{
	return Quaternion<T>(k / q.val[0], k / q.val[1], k / q.val[2], k / q.val[3]);
}

template <class T> inline Quaternion<T> operator+(const Quaternion<T>& a, const Quaternion<T>& b)	// binary addition operator
{
	return Quaternion<T>(a.val[0] + b.val[0], a.val[1] + b.val[1], a.val[2] + b.val[2], a.val[3] + b.val[3]);
}
template <class T> inline Quaternion<T> operator-(const Quaternion<T>& a, const Quaternion<T>& b)	// binary subtraction operator
{
	return Quaternion<T>(a.val[0] - b.val[0], a.val[1] - b.val[1], a.val[2] - b.val[2], a.val[3] - b.val[3]);
}
template <class T> inline Quaternion<T> operator*(const Quaternion<T>& a, const Quaternion<T>& b)	// binary multiplication operator (element-wise)
{
	return Quaternion<T>(a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
						 a[1]*b[0] + a[0]*b[1] - a[3]*b[2] + a[2]*b[3],
						 a[2]*b[0] + a[3]*b[1] + a[0]*b[2] - a[1]*b[3],
						 a[3]*b[0] - a[2]*b[1] + a[1]*b[2] + a[0]*b[3]);
}
template <class T> inline Quaternion<T> operator*(const Quaternion<T>& q, const T& k)				// binary scalar multiplication operator
{
	return Quaternion<T>(q.val[0] * k, q.val[1] * k, q.val[2] * k, q.val[3] * k);
}
template <class T> inline Quaternion<T> operator*(const T& k, const Quaternion<T>& q)				// binary scalar multiplication operator
{
	return Quaternion<T>(q.val[0] * k, q.val[1] * k, q.val[2] * k, q.val[3] * k);
}
template <class T> inline Quaternion<T> operator/(const Quaternion<T>& a, const Quaternion<T>& b)	// binary division operator (element-wise)
{
	return Mul(a, Inv(b));
}
template <class T> inline Quaternion<T> operator/(const Quaternion<T>& q, const T& k)				// binary scalar division operator
{
	return Quaternion<T>(q.val[0] / k, q.val[1] / k, q.val[2] / k, q.val[3] / k);
}
template <class T> inline Quaternion<T> operator/(const T& k, const Quaternion<T>& q)				// binary scalar division operator
{
	return Quaternion<T>(k / q.val[0], k / q.val[1], k / q.val[2], k / q.val[3]);
}

template <class T> inline Quaternion<T> operator!(const Quaternion<T>& q)							// unary inversion operator
{
	T n = q.NormSquared();
	return Quaternion<T>(q.val[0]/n, -q.val[1]/n, -q.val[2]/n, -q.val[3]/n);
}

template <class T> inline Quaternion<T> QuaternionFromAxisAngle(const T rad, const T axis_x, const T axis_y, const T axis_z)
{
	Quaternion<T> temp;
	T m = sqrt(axis_x*axis_x + axis_y*axis_y + axis_z*axis_z);

	if(m == 0) // identity
	{
		temp.val[0] = 1;
		temp.val[1] = temp.val[2] = temp.val[3] = 0;
	}		
	else
	{
		T sm = sin(rad*0.5)/m;
		temp.val[0] = cos(rad*0.5);
		temp.val[1] = axis_x*sm;
		temp.val[2] = axis_y*sm;
		temp.val[3] = axis_z*sm;

		temp.Normalize();				// Do I have to normalize q here?
	}
	return temp;
}
template <class T> inline Quaternion<T> QuaternionFromMatrix(const T* R)
{
	Quaternion<T> temp;
	T tr = R[0] + R[4] + R[8];
	if( tr > 0 )
	{
		T s = 0.5 / sqrt(tr + 1.0);
		temp.val[0] = 0.25 / s;
		temp.val[1] = ( R[5] - R[7] ) * s;			// (R21 - R12)*s
		temp.val[2] = ( R[6] - R[2] ) * s;			// (R02 - R20)*s
		temp.val[3] = ( R[1] - R[3] ) * s;			// (R10 - R01)*s
	}
	else
	{
		if( R[0] > R[4] && R[0] > R[8] )	// R00 > R11 && R00 > R22
		{
			T s = 0.5 / sqrt(1.0 + R[0] - R[4] - R[8]);
			temp.val[0] = ( R[5] - R[7] ) * s;		// (R21 - R12)*s
			temp.val[1] = 0.25 / s;
			temp.val[2] = ( R[3] + R[1] ) * s;		// (R01 + R10)*s
			temp.val[3] = ( R[6] + R[2] ) * s;		// (R02 + R20)*s
		}
		else if (R[4] > R[8])
		{
			T s = 0.5 / sqrt(1.0 + R[4] - R[0] - R[8]);
			temp.val[0] = ( R[6] - R[2] ) * s;		// (R02 - R20)*s
			temp.val[1] = ( R[3] + R[1] ) * s;		// (R01 + R10)*s
			temp.val[2] = 0.25 / s;
			temp.val[3] = ( R[7] + R[5] ) * s;		// (R12 + R21)*s
		}
		else
		{
			T s = 0.5 / sqrt(1.0 + R[8] - R[0] - R[4]);
			temp.val[0] = ( R[1] - R[3] ) * s;		// (R10 - R01)*s
			temp.val[1] = ( R[6] + R[2] ) * s;		// (R02 + R20)*s
			temp.val[2] = ( R[7] + R[5] ) * s;		// (R12 + R21)*s
			temp.val[3] = 0.25f / s;
		}
	}
	return temp;
}
template <class T> inline Quaternion<T> QuaternionFromMatrix(const T R[3][3])
{
	Quaternion<T> temp;
	T tr = R[0][0] + R[1][1] + R[2][2];
	if( tr > 0 )
	{
		T s = 0.5 / sqrt(tr + 1.0);
		temp.val[0] = 0.25 / s;
		temp.val[1] = ( R[2][1] - R[1][2] ) * s;			// (R21 - R12)*s
		temp.val[2] = ( R[0][2] - R[2][0] ) * s;			// (R02 - R20)*s
		temp.val[3] = ( R[1][0] - R[0][1] ) * s;			// (R10 - R01)*s
	}
	else
	{
		if( R[0][0] > R[1][1] && R[0][0] > R[2][2] )	// R00 > R11 && R00 > R22
		{
			T s = 0.5 / sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
			temp.val[0] = ( R[2][1] - R[1][2] ) * s;		// (R21 - R12)*s
			temp.val[1] = 0.25 / s;
			temp.val[2] = ( R[0][1] + R[1][0] ) * s;		// (R01 + R10)*s
			temp.val[3] = ( R[0][2] + R[2][0] ) * s;		// (R02 + R20)*s
		}
		else if (R[1][1] > R[2][2])
		{
			T s = 0.5 / sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
			temp.val[0] = ( R[0][2] - R[2][0] ) * s;		// (R02 - R20)*s
			temp.val[1] = ( R[0][1] + R[1][0] ) * s;		// (R01 + R10)*s
			temp.val[2] = 0.25 / s;
			temp.val[3] = ( R[1][2] + R[2][1] ) * s;		// (R12 + R21)*s
		}
		else
		{
			T s = 0.5 / sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
			temp.val[0] = ( R[1][0] - R[0][1] ) * s;		// (R10 - R01)*s
			temp.val[1] = ( R[0][2] + R[2][0] ) * s;		// (R02 + R20)*s
			temp.val[2] = ( R[1][2] + R[2][1] ) * s;		// (R12 + R21)*s
			temp.val[3] = 0.25f / s;
		}
	}
	return temp;
}
template <class T> inline Quaternion<T> QuaternionFromEulerAngle(const T a, const T b, const T c, const char* order)
{
	Quaternion<T> temp;
	//TODO
	return temp;
}

template <class T> inline Quaternion<T> Log(const Quaternion<T>& q)
{
	Quaternion<T> qtemp(q);
	qtemp.Normalize();

	if( qtemp.val[0] == 0 )
	{
		qtemp.val[0] = 0;
		qtemp.val[1] = 0.5*M_PI*qtemp.val[1];
		qtemp.val[2] = 0.5*M_PI*qtemp.val[2];
		qtemp.val[3] = 0.5*M_PI*qtemp.val[3];
	}
	else if( qtemp.val[0] == 1 )
	{
		qtemp.val[0] = 0;
		qtemp.val[1] = 0;
		qtemp.val[2] = 0;
		qtemp.val[3] = 0;
	}
	else
	{
		T v = sqrt(qtemp.val[1]*qtemp.val[1] + qtemp.val[2]*qtemp.val[2] + qtemp.val[3]*qtemp.val[3]);
		qtemp.val[0] = 2*atan2(v, qtemp.val[0]);
		qtemp.val[1] = qtemp.val[1]/v;
		qtemp.val[2] = qtemp.val[2]/v;
		qtemp.val[3] = qtemp.val[3]/v;
	}

	return qtemp;
}
template <class T> inline Quaternion<T> Exp(const Quaternion<T>& q)
{
	T m = q.val[1]*q.val[1] + q.val[2]*q.val[2] + q.val[3]*q.val[3];

	if(m == 0)
		return Quaternion<T>(1.0, 0.0, 0.0, 0.0);
	else
	{
		Quaternion<T> qtemp(q);
		m = sqrt(m);
		T sm = sin(q.val[0]*0.5)/m;
		qtemp.val[0] = cos(q.val[0]*0.5);
		qtemp.val[1] = q.val[1]*sm;
		qtemp.val[2] = q.val[2]*sm;
		qtemp.val[3] = q.val[3]*sm;

		return qtemp.Normalize();
	}
}

typedef Quaternion<double>	Quaterniond;
typedef Quaternion<float>	Quaternionf;

#endif // HCCL_VECTORQUATERNION_H_