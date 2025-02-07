/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <math.h>
#include <float.h>
#include <assert.h>
#include <stdlib.h>

const float k_pi = 3.14159265358979323846264f;

struct Vec2
{
	Vec2() {}
	Vec2(float x, float y) : x(x), y(y) {}

	void Set(float x_, float y_) { x = x_; y = y_; }

	Vec2 operator -() { return Vec2(-x, -y); }

	void operator += (const Vec2& v)
	{
		x += v.x; y += v.y;
	}

	void operator -= (const Vec2& v)
	{
		x -= v.x; y -= v.y;
	}

	void operator *= (float a)
	{
		x *= a; y *= a;
	}

	float Length() const
	{
		return sqrtf(x * x + y * y);
	}

	float x, y;
};

struct Vec3
{
	Vec3() {}
	Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

	void Set(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

	Vec3 operator -() { return Vec3(-x, -y, -z); }

	void operator += (const Vec3& v)
	{
		x += v.x; y += v.y; z += v.z;
	}

	void operator -= (const Vec3& v)
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	void operator *= (float a)
	{
		x *= a; y *= a; z *= a;
	}

	float Length() const
	{
		return sqrtf(x * x + y * y + z * z);
	}

	Vec3 Normalized() const
	{
		Vec3 v = *this;
		float len = Length();
		if (len > 0.0f)
		{
			v *= 1.0f / len;
		}
		return v;
	}

	float x, y, z;
};

struct Mat22
{
	Mat22() {}
	Mat22(float angle)
	{
		float c = cosf(angle), s = sinf(angle);
		col1.x = c; col2.x = -s;
		col1.y = s; col2.y = c;
	}

	Mat22(const Vec2& col1, const Vec2& col2) : col1(col1), col2(col2) {}

	Mat22 Transpose() const
	{
		return Mat22(Vec2(col1.x, col2.x), Vec2(col1.y, col2.y));
	}

	Mat22 Invert() const
	{
		float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
		Mat22 B;
		float det = a * d - b * c;
		assert(det != 0.0f);
		det = 1.0f / det;
		B.col1.x =  det * d;	B.col2.x = -det * b;
		B.col1.y = -det * c;	B.col2.y =  det * a;
		return B;
	}

	Vec2 col1, col2;
};

struct Mat33
{
	Mat33() {}
	Mat33(const Vec3& col1, const Vec3& col2, const Vec3& col3) : col1(col1), col2(col2), col3(col3) {}

	void Identity()
	{
		col1 = Vec3(1.0f, 0.0f, 0.0f);
		col2 = Vec3(0.0f, 1.0f, 0.0f);
		col3 = Vec3(0.0f, 0.0f, 1.0f);
	}

	void Zero()
	{
		col1 = Vec3(0.0f, 0.0f, 0.0f);
		col2 = Vec3(0.0f, 0.0f, 0.0f);
		col3 = Vec3(0.0f, 0.0f, 0.0f);
	}

	Mat33 Transpose() const
	{
		return Mat33(Vec3(col1.x, col2.x, col3.x), Vec3(col1.y, col2.y, col3.y), Vec3(col1.z, col2.z, col3.z));
	}

	Mat33 Invert() const
	{
		float a11 = col1.x, a12 = col2.x, a13 = col3.x;
		float a21 = col1.y, a22 = col2.y, a23 = col3.y;
		float a31 = col1.z, a32 = col2.z, a33 = col3.z;
		float det = a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
		assert(det != 0.0f);
		det = 1.0f / det;
		Mat33 B;
		B.col1.x = det * (a22 * a33 - a23 * a32);
		B.col2.x = det * (a13 * a32 - a12 * a33);
		B.col3.x = det * (a12 * a23 - a13 * a22);
		B.col1.y = det * (a23 * a31 - a21 * a33);
		B.col2.y = det * (a11 * a33 - a13 * a31);
		B.col3.y = det * (a13 * a21 - a11 * a23);
		B.col1.z = det * (a21 * a32 - a22 * a31);
		B.col2.z = det * (a12 * a31 - a11 * a32);
		B.col3.z = det * (a11 * a22 - a12 * a21);
		return B;
	}

	Vec3 col1, col2, col3;
};

struct Quat
{
	Quat() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
	Quat(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
	Quat(const Vec3& angles)
	{
		float halfPitch = angles.x * 0.5f;
		float halfYaw = angles.y * 0.5f;
		float halfRoll = angles.z * 0.5f;
		float sinPitch = sinf(halfPitch);
		float cosPitch = cosf(halfPitch);
		float sinYaw = sinf(halfYaw);
		float cosYaw = cosf(halfYaw);
		float sinRoll = sinf(halfRoll);
		float cosRoll = cosf(halfRoll);
		x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
		y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
		z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
		w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	}
	void Set(float x_, float y_, float z_, float w_) { x = x_; y = y_; z = z_; w = w_; }

	void Identity()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		w = 1.0f;
	}

	Mat33 ToMatrix() const
	{
		Mat33 M;
		float xx = x * x;
		float yy = y * y;
		float zz = z * z;
		float xy = x * y;
		float xz = x * z;
		float yz = y * z;
		float wx = w * x;
		float wy = w * y;
		float wz = w * z;
		M.col1.x = 1.0f - 2.0f * (yy + zz);
		M.col1.y = 2.0f * (xy + wz);
		M.col1.z = 2.0f * (xz - wy);
		M.col2.x = 2.0f * (xy - wz);
		M.col2.y = 1.0f - 2.0f * (xx + zz);
		M.col2.z = 2.0f * (yz + wx);
		M.col3.x = 2.0f * (xz + wy);
		M.col3.y = 2.0f * (yz - wx);
		M.col3.z = 1.0f - 2.0f * (xx + yy);
		return M;
	}

	Quat operator*(const Quat& q) const
	{
		return Quat(w * q.x + x * q.w + y * q.z - z * q.y,
			w * q.y + y * q.w + z * q.x - x * q.z,
			w * q.z + z * q.w + x * q.y - y * q.x,
			w * q.w - x * q.x - y * q.y - z * q.z);
	}

	const Quat& operator*=(const Quat& q)
	{
		return *this = *this * q;
	}

	float x, y, z, w;
};

inline float Dot(const Vec2& a, const Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

inline float Dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float Cross(const Vec2& a, const Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

inline Vec2 Cross(const Vec2& a, float s)
{
	return Vec2(s * a.y, -s * a.x);
}

inline Vec2 Cross(float s, const Vec2& a)
{
	return Vec2(-s * a.y, s * a.x);
}

inline Vec2 operator * (const Mat22& A, const Vec2& v)
{
	return Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
}

inline Vec2 operator + (const Vec2& a, const Vec2& b)
{
	return Vec2(a.x + b.x, a.y + b.y);
}

inline Vec2 operator - (const Vec2& a, const Vec2& b)
{
	return Vec2(a.x - b.x, a.y - b.y);
}

inline Vec2 operator * (float s, const Vec2& v)
{
	return Vec2(s * v.x, s * v.y);
}

inline Vec3 operator * (const Mat33& A, const Vec3& v)
{
	return Vec3(A.col1.x * v.x + A.col2.x * v.y + A.col3.x * v.z,
		        A.col1.y * v.x + A.col2.y * v.y + A.col3.y * v.z,
		        A.col1.z * v.x + A.col2.z * v.y + A.col3.z * v.z);
}

inline Vec3 operator + (const Vec3& a, const Vec3& b)
{
	return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vec3 operator - (const Vec3& a, const Vec3& b)
{
	return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vec3 operator * (float s, const Vec3& v)
{
	return Vec3(s * v.x, s * v.y, s * v.z);
}

inline Vec3 operator * (const Vec3& a, const Vec3& b)
{
	return Vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline Vec3 Cross(const Vec3& a, const Vec3& b)
{
	return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline Mat22 operator + (const Mat22& A, const Mat22& B)
{
	return Mat22(A.col1 + B.col1, A.col2 + B.col2);
}

inline Mat22 operator * (const Mat22& A, const Mat22& B)
{
	return Mat22(A * B.col1, A * B.col2);
}

inline Mat33 operator + (const Mat33& A, const Mat33& B)
{
	return Mat33(A.col1 + B.col1, A.col2 + B.col2, A.col3 + B.col3);
}

inline float Abs(float a)
{
	return a > 0.0f ? a : -a;
}

inline Vec2 Abs(const Vec2& a)
{
	return Vec2(fabsf(a.x), fabsf(a.y));
}

inline Mat22 Abs(const Mat22& A)
{
	return Mat22(Abs(A.col1), Abs(A.col2));
}

inline float Sign(float x)
{
	return x < 0.0f ? -1.0f : 1.0f;
}

inline float Min(float a, float b)
{
	return a < b ? a : b;
}

inline float Max(float a, float b)
{
	return a > b ? a : b;
}

inline float Clamp(float a, float low, float high)
{
	return Max(low, Min(a, high));
}

template<typename T> inline void Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

// Random number in range [-1,1]
inline float Random()
{
	float r = (float)rand();
	r /= RAND_MAX;
	r = 2.0f * r - 1.0f;
	return r;
}

inline float Random(float lo, float hi)
{
	float r = (float)rand();
	r /= RAND_MAX;
	r = (hi - lo) * r + lo;
	return r;
}

#endif

