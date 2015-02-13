/*--------------------------------------------------------------
 __    __                                         __    __
/\ \__/\ \                                     __/\ \__/\ \__
\ \ ,_\ \ \___      __  _____     __     _ __ /\_\ \ ,_\ \ ,_\
 \ \ \/\ \  _ `\  /'__`\\ '__`\ /'__`\  /\`'__\/\ \ \ \/\ \ \/
  \ \ \_\ \ \ \ \/\  __/ \ \L\ \\ \L\.\_\ \ \/ \ \ \ \ \_\ \ \_
   \ \__\\ \_\ \_\ \____\ \ ,__/ \__/.\_\\ \_\  \ \_\ \__\\ \__\
    \/__/ \/_/\/_/\/____/\ \ \/ \/__/\/_/ \/_/   \/_/\/__/ \/__/
                          \ \_\
                           \/_/
----------------------------------------------------------------*/
/*
 * kinematics.hpp
 * Copyright (C) 2014 [theparitt]
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by theparit peerasathien <theparitt@gmail.com>, December 2014
 */



#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <string>
#include <iostream>
#include <sstream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

#include "matrix.hpp"




//ใช้ใน quaternion เพิ่อดูว่า trace มีค่าน้อยมากๆๆหรือไม่
const double M_EPSILON = 0.000001;



template <class T> std::string to_string( const T &t )
{
		std::ostringstream oss;
		oss << t;
		return std::string (oss.str());
}




class vec3
{

protected:

	static const int X = 0;
	static const int Y = 1;
	static const int Z = 2;


	double data[3];

public:
	vec3()
	{

	}


	vec3(double x, double y, double z)
	{
		this->data[X] = x;
		this->data[Y] = y;
		this->data[Z] = z;
	}



	vec3(const vec3& B)
	{
		this->data[X] = B.data[X];
		this->data[Y] = B.data[Y];
		this->data[Z] = B.data[Z];
	}



	virtual ~vec3()
	{

	}



	double& at(int i)
	{
		assert( i < 3 );
		return this->data[i];
	}


	double at(int i) const
	{
		return this->data[i];
	}


	double& operator[](int i)
	{
		assert( i<3 );
		return this->data[i];
	}


	double operator[](int i) const
	{
		assert( i<3 );
		return this->data[i];
	}



	friend bool operator==(const vec3& A, const vec3& B)
	{
		if( abs(A[0] - B[0]) > M_EPSILON )
			return false;

		if( abs(A[1] - B[1]) > M_EPSILON )
			return false;

		if( abs(A[2] - B[2]) > M_EPSILON )
			return false;

		return true;
	}


	virtual vec3& operator=(const vec3& A)
	{
		if (&A == this)
		{
			return *this;
		}

		this->data[0] = A.data[0];
		this->data[1] = A.data[1];
		this->data[2] = A.data[2];

		return *this;
	}



	//A dot B - Project vector A to vector B and see the size of A on B
	double dot(vec3& B) const
	{
		return (data[0]*B.at(0)) + (data[1]*B.at(1)) + (data[2]*B.at(2));
	}


	double dot(const vec3& B) const
	{
		return (data[0]*B.at(0)) + (data[1]*B.at(1)) + (data[2]*B.at(2));
	}


	double friend dot(const vec3& A, const vec3& B)
	{
		return (A[0]*B[0]) + (A[1]*B[1]) + (A[2]*B(2));
	}


	virtual void debug(const char* message = "") const
	{
		printf("%s", message); fflush( stdout );
		printf("( %.5f, %.5f, %.5f )\n", this->data[0], this->data[1], this->data[2]);
		fflush( stdout );
	}


	vec3& debug(const char* message = "")
	{
		printf("%s", message); fflush( stdout );
		printf("( %.5f, %.5f, %.5f )\n", this->data[0], this->data[1], this->data[2]);
		fflush( stdout );

		return *this;
	}


	double& operator()(int axis)
	{
		assert( (axis >= 0) && (axis <= 2) );
		return this->data[axis];
	};


	double operator()(int axis) const
	{
		assert( (axis >= 0) && (axis <= 2) );
		return this->data[axis];
	};


	void operator()(double x, double y, double z)
	{
		this->data[X] = x;
		this->data[Y] = y;
		this->data[Z] = z;
	};


	void operator()(const vec3& A)
	{
		this->data[0] = A.data[0];
		this->data[1] = A.data[1];
		this->data[2] = A.data[2];
	}


	vec3 operator* (vec3& A) const
	{
		return vec3( A(0)*this->data[0], A(1)*this->data[1], A(2)*this->data[2] );
	}


	vec3 operator+ (const vec3& A) const
	{
		return vec3( A.data[0] + data[0],
				     A.data[1] + data[1],
				     A.data[2] + data[2] );
	}


	void operator+= (vec3& A)
	{
		this->data[0] += A(0);
		this->data[1] += A(1);
		this->data[2] += A(2);
	}


	void operator+= (const vec3& A)
	{
		this->data[0] += A.data[0];
		this->data[1] += A.data[1];
		this->data[2] += A.data[2];
	}


	vec3 operator- (const vec3& A) const
	{
		return vec3( this->data[0]-A(0), this->data[1]-A(1), this->data[2]-A(2) );
	}


	void operator-= (vec3& A)
	{
		this->data[0] -= A(0);
		this->data[1] -= A(1);
		this->data[2] -= A(2);
	}


	void operator-= (const vec3& A)
	{
		this->data[0] -= A.data[0];
		this->data[1] -= A.data[1];
		this->data[2] -= A.data[2];
	}


	void operator/= (double a)
	{
		this->data[0] /= a;
		this->data[1] /= a;
		this->data[2] /= a;
	}



	bool equal(const vec3& A) const
	{
		if( abs( this->data[0] - A(0)) > SMALL_VALUE  )
		{
			return false;
		}

		if( abs( this->data[1] - A(1)) > SMALL_VALUE  )
		{
			return false;
		}

		if( abs( this->data[2] - A(2)) > SMALL_VALUE  )
		{
			return false;
		}

		return true;
	}


	vec3 operator+ (double a) const
	{
		return vec3( this->data[0]+a, this->data[1]+a, this->data[2]+a);
	}


	vec3 operator- (double a) const
	{
		return vec3( this->data[0]-a, this->data[1]-a, this->data[2]-a);
	}


	friend vec3 operator*(const vec3& A, double c)
	{
		return vec3( c*A[0], c*A[1], c*A[2] );
	}


	friend vec3 operator*(double c, const vec3& A)
	{
		return vec3( c*A[0], c*A[1], c*A[2] );
	}


	void operator*= (double a)
	{
		data[0] *= a;
		data[1] *= a;
		data[2] *= a;
	}


	vec3 operator/ (double a) const
	{
		assert( a != 0.0);
		return vec3( this->data[0]/a, this->data[1]/a, this->data[2]/a);
	}


	vec3 cross(const vec3& B) const
	{
		return vec3( this->at(Y)* B(Z) - this->at(Z)* B(Y),
					 this->at(Z)* B(X) - this->at(X)* B(Z),
				     this->at(X)* B(Y) - this->at(Y)* B(X)
		);
	}


	friend vec3 cross(const vec3& A, const vec3& B)
	{

		return vec3( A(Y)* B(Z) - A(Z)* B(Y),
					 A(Z)* B(X) - A(X)* B(Z),
					 A(X)* B(Y) - A(Y)* B(X)
			);
	}


	friend vec3 operator+(double a, vec3& B)
	{
		return vec3(a+B(0), a+B(1), a+B(2));
	}

	friend vec3 operator-(double a, vec3& B)
	{
		return vec3(a-B(0), a-B(1), a-B(2));
	}


	friend vec3 operator-(const vec3& A)
	{
		return vec3( -A[0], -A[1], -A[2] );
	}


	friend vec3 operator*(double a, vec3& B)
	{
		return vec3(a*B(0), a*B(1), a*B(2));
	}


	friend vec3 operator/(double a, vec3& B)
	{
		return vec3(a/B(0), a/B(1), a/B(2));
	}


	double mag() const
	{
		return sqrt( this->data[0]*this->data[0] +
				     this->data[1]*this->data[1] +
			      	 this->data[2]*this->data[2]);
	}


	double mag2() const
	{
		return this->data[0]*this->data[0] +
				 this->data[1]*this->data[1] +
				 this->data[2]*this->data[2];
	}



	void normalize()
	{
		double sz = mag();
		assert( sz != 0.0 );
		this->data[0] /= sz;
		this->data[1] /= sz;
		this->data[2] /= sz;
	}


	vec3 get_normalize() const
	{
		vec3 result( *this );

		result.normalize();
		return result;
	}


	friend double distance_square(const vec3& A, const vec3& B)
	{

		return  (A[0] - B[0])*(A[0] - B[0]) + (A[1] - B[1])*(A[1] - B[1]) + (A[2] - B[2])*(A[2] - B[2]);
	}


	friend double mag(const vec3& A, const vec3& B)
	{
		return sqrt( distance_square(A,B) );
	}

};



double scalar_triple(const vec3& a, const vec3& b, const vec3& c)
{
	return dot( a, cross(b,c) );
}


vec3 vector_triple(const vec3& a, const vec3& b, const vec3& c)
{
	return cross( a, cross(b,c) );
}


const double M_PI_SQUARE = M_PI * M_PI;

//convert from degree to rad
inline double DEG(double deg)
{
	return ( (deg * M_PI) / 180.0);
}


//convert from rad to deg
inline double RAD(double rad)
{
	return (rad * 180.0) / M_PI;
}



class quaternion;


class matrix_rot: public matrix_core
{

private:


protected:


public:

	matrix_rot()
		:matrix_core(3,3)
	{
	}


	matrix_rot(const matrix_rot& A)
		:matrix_core(A)
	{
	}


	matrix_rot(const matrix_core& A)
		:matrix_core(A)
	{
	}


	matrix_rot(double roll, double pitch, double yaw )
		:matrix_core(3,3)
	{
		set_roll_pitch_yaw( roll, pitch, yaw );
	}


	virtual ~matrix_rot()
	{

	}


	void set_rotx(double rad)
	{
		at(0,0) = 1.0;
		at(0,1) = 0.0;
		at(0,2) = 0.0;

		at(1,0) = 0.0;
		at(1,1) = cos(rad);
		at(1,2) = -sin(rad);

		at(2,0) = 0.0;
		at(2,1) = sin(rad);
		at(2,2) = cos(rad);
	}

	void set_roty(double rad)
	{
		at(0,0) = cos(rad);
		at(0,1) = 0.0;
		at(0,2) = sin(rad);

		at(1,0) = 0.0;
		at(1,1) = 1.0;
		at(1,2) = 0.0;

		at(2,0) = -sin(rad);
		at(2,1) = 0;
		at(2,2) = cos(rad);
	}

	void set_rotz(double rad)
	{
		at(0,0) = cos(rad);
		at(0,1) = -sin(rad);
		at(0,2) = 0.0;

		at(1,0) = sin(rad);
		at(1,1) = cos(rad);
		at(1,2) = 0.0;

		at(2,0) = 0.0;
		at(2,1) = 0.0;
		at(2,2) = 1.0;
	}

	vec3 operator* (const vec3& A) const
	{
		double a, b, c;
		a =     at(0,0) * A(0) +
				at(0,1) * A(1) +
				at(0,2) * A(2);

		b =     at(1,0) * A(0) +
				at(1,1) * A(1) +
				at(1,2) * A(2);

		c =     at(2,0) * A(0) +
				at(2,1) * A(1) +
				at(2,2) * A(2);

		return vec3(a,b,c);
	}


	matrix_rot operator*(const matrix_rot& A ) const
	{
		matrix_rot result;

		result.at(0,0) = at(0,0)* A(0,0) + at(0,1)* A(1,0) + at(0,2)* A(2,0);
		result.at(0,1) = at(0,0)* A(0,1) + at(0,1)* A(1,1) + at(0,2)* A(2,1);
		result.at(0,2) = at(0,0)* A(0,2) + at(0,1)* A(1,2) + at(0,2)* A(2,2);

		result.at(1,0) = at(1,0)* A(0,0) + at(1,1)* A(1,0) + at(1,2)* A(2,0);
		result.at(1,1) = at(1,0)* A(0,1) + at(1,1)* A(1,1) + at(1,2)* A(2,1);
		result.at(1,2) = at(1,0)* A(0,2) + at(1,1)* A(1,2) + at(1,2)* A(2,2);

		result.at(2,0) = at(2,0)* A(0,0) + at(2,1)* A(1,0) + at(2,2)* A(2,0);
		result.at(2,1) = at(2,0)* A(0,1) + at(2,1)* A(1,1) + at(2,2)* A(2,1);
		result.at(2,2) = at(2,0)* A(0,2) + at(2,1)* A(1,2) + at(2,2)* A(2,2);

		return result;
	}


	matrix_rot operator*(const matrix_core& A ) const
	{
		matrix_rot result;

		result.at(0,0) = at(0,0)* A(0,0) + at(0,1)* A(1,0) + at(0,2)* A(2,0);
		result.at(0,1) = at(0,0)* A(0,1) + at(0,1)* A(1,1) + at(0,2)* A(2,1);
		result.at(0,2) = at(0,0)* A(0,2) + at(0,1)* A(1,2) + at(0,2)* A(2,2);

		result.at(1,0) = at(1,0)* A(0,0) + at(1,1)* A(1,0) + at(1,2)* A(2,0);
		result.at(1,1) = at(1,0)* A(0,1) + at(1,1)* A(1,1) + at(1,2)* A(2,1);
		result.at(1,2) = at(1,0)* A(0,2) + at(1,1)* A(1,2) + at(1,2)* A(2,2);

		result.at(2,0) = at(2,0)* A(0,0) + at(2,1)* A(1,0) + at(2,2)* A(2,0);
		result.at(2,1) = at(2,0)* A(0,1) + at(2,1)* A(1,1) + at(2,2)* A(2,1);
		result.at(2,2) = at(2,0)* A(0,2) + at(2,1)* A(1,2) + at(2,2)* A(2,2);

		return result;
	}

	/*
	 * rotate p vector around x-axis and translate by d
	 */
	static vec3 Rxp_d(double R_rad, vec3 p, vec3 d)
	{
		double a, b, c;

		a = p(0)                                        + d(0);
		b =        (cos(R_rad)*p(1) - sin(R_rad)*p(2))  + d(1);
		c =        (sin(R_rad)*p(1) + cos(R_rad)*p(2))  + d(2);

		return vec3(a, b, c);
	}

	/*
	 * rotate p vector around y-axis and translate by d
	 */
	static vec3 Ryp_d(double R_rad, vec3 p, vec3 d)
	{
		double a, b, c;

		a = ( cos(R_rad)*p(0)       + sin(R_rad)*p(2))   + d(0);
		b =                    p(1)                      + d(1);
		c = (-sin(R_rad)*p(0)       + cos(R_rad)*p(2))   + d(2);

		return vec3(a, b, c);
	}

	/*
	 * rotate p vector around z-axis and translate by d
	 */
	static vec3 Rzp_d(double R_rad, vec3 p, vec3 d)
	{
		double a, b, c;
		a = ( cos(R_rad)*p(0) - sin(R_rad)*p(1))       + d(0);
		b = ( sin(R_rad)*p(0) + cos(R_rad)*p(1))       + d(1);
		c =                                       p(2) + d(2);
		return vec3(a, b, c);
	}



	void set_angx(double rad_s)
	{
		at(0,0) = 1.0;
		at(0,1) = 0.0;
		at(0,2) = 0.0;

		at(1,0) =  0.0;
		at(1,1) = -sin(rad_s);
		at(1,2) = -cos(rad_s);

		at(2,0) =  0.0;
		at(2,1) =  cos(rad_s);
		at(2,2) = -sin(rad_s);
	}


	void set_angy(double rad_s)
	{
		at(0,0) = -sin(rad_s);
		at(0,1) =  0.0;
		at(0,2) =  cos(rad_s);

		at(1,0) = 0.0;
		at(1,1) = 1.0;
		at(1,2) = 0.0;

		at(2,0) = -cos(rad_s);
		at(2,1) =  0.0;
		at(2,2) = -sin(rad_s);
	}


	void set_angz(double rad_s)
	{
		at(0,0) = -sin(rad_s);
		at(0,1) = -cos(rad_s);
		at(0,2) =  0.0;

		at(1,0) =  cos(rad_s);
		at(1,1) = -sin(rad_s);
		at(1,2) =  0.0;

		at(2,0) = 0.0;
		at(2,1) = 0.0;
		at(2,2) = 1.0;
	}


	/**
	 * @param u_hat  axis of rotation
	 * @param rad    radian of rotation around axis u_hat
	 * @return       matrix or rotation
	 *
	 * ex: u_hat = [1.0/sqrt(3.0),1.0/sqrt(3.0),1.0/sqrt(3.0)]
	 *     rad   = DEG(30)
	 *
	 * return:
	 *     0.910684 -0.244017  0.333333
 	 *     0.333333  0.910684 -0.244017
	 *    -0.244017  0.333333  0.910684
	 */

	matrix_rot& set_axis_angle(const vec3& u_hat, double rad)
	{
		double vers = 1.0 - cos(rad);
		double cos_r = cos(rad);
		double sin_r = sin(rad);

		at(0,0) = u_hat(0) * u_hat(0) * vers  +             cos_r;
		at(0,1) = u_hat(0) * u_hat(1) * vers  -  u_hat(2) * sin_r;
		at(0,2) = u_hat(0) * u_hat(2) * vers  +  u_hat(1) * sin_r;

		at(1,0) = u_hat(0) * u_hat(1) * vers  +  u_hat(2) * sin_r;
		at(1,1) = u_hat(1) * u_hat(1) * vers  +             cos_r;
		at(1,2) = u_hat(1) * u_hat(2) * vers  -  u_hat(0) * sin_r;

		at(2,0) = u_hat(0) * u_hat(2) * vers  -  u_hat(1) * sin_r;
		at(2,1) = u_hat(1) * u_hat(2) * vers  +  u_hat(0) * sin_r;
		at(2,2) = u_hat(2) * u_hat(2) * vers  +             cos_r;

		return *this;
	}



	/**
	 * Using equation from "Theory of Applied Robotics, page 97"
	 * @param u_hat - axis of rotation
	 * @param rad   - radian of rotation around u_hat
	 *
	 * ex: rotation Rz -> Rx -> Rz = (30, 45, 60) deg
	 * get Rzxz  =
	 * 	0.126826 -0.926777  0.353553
	 *  0.780330 -0.126826 -0.612372
	 *  0.612372  0.353553  0.707107
	 *
	 *  get axis&angle
	 *  axis: (0.48823, -0.13082, 0.86286)  angle: 98.421058
	 */

	void get_axis_angle(vec3& u_hat, double& rad) const
	{
		rad = acos( 1.0/2.0 * ( (tr(*this)) - 1.0) );
		//---------------//
		//  no rotation  //
		//---------------//
		if( abs(rad) < 0.00001 )
		{
			u_hat(1.0, 0.0, 0.0);
			rad = 0;
			return;
		}

		matrix_core mat_tp = this->tp();
		matrix_core tmp  = (*this) - mat_tp;
		double dom = 2*sin( rad );


		u_hat[0] = tmp(2,1) / dom;
		u_hat[1] = tmp(0,2) / dom;
		u_hat[2] = tmp(1,0) / dom;
	}


	void set_sym_skew(double w0, double w1, double w2)
	{
		at(0,0) =  0.0;
		at(0,1) = -w2;
		at(0,2) =  w1;

		at(1,0) =  w2;
		at(1,1) =  0.0;
		at(1,2) = -w0;

		at(2,0) = -w1;
		at(2,1) =  w0;
		at(2,2) =  0.0;
	}




	/** rotation Rx -> Ry -> Rz:  Rz*Ry*Rx
	 * @param rad_x
	 * @param rad_y
	 * @param rad_z
	 * ex:  rad_x =  0.38759
	 *      rad_y = -0.36136
	 *      rad_z =  0.71372
	 * rot: 0.707111 -0.707102 -0.000002
	 *      0.612371 0.612381 -0.499992
	 *      0.353547 0.353549 0.866030
	 */

	void set_roll_pitch_yaw(double rad_x, double rad_y, double rad_z)
	{
		double sx = sin(rad_x);
		double sy = sin(rad_y);
		double sz = sin(rad_z);
		double cx = cos(rad_x);
		double cy = cos(rad_y);
		double cz = cos(rad_z);

		at(0,0) =  cy*cz;
		at(0,1) = -cx*sz + sx*sy*cz;
		at(0,2) =  sx*sz + cx*sy*cz;

		at(1,0) =  cy*sz;
		at(1,1) =  cx*cz + sx*sy*sz;
		at(1,2) = -cz*sx + cx*sy*sz;

		at(2,0) = -sy;
		at(2,1) =  cy*sx;
		at(2,2) =  cx*cy;
	}


	/**
	 *
	 * @return vec3 of [roll pitch yaw]
	 */
	vec3 get_roll_pitch_yaw() const
	{
		double roll  =  atan2( at(2,1), at(2,2) );
		double pitch = -asin ( at(2,0) );
		double yaw   =  atan2( at(1,0), at(0,0) );

		return vec3( roll, pitch, yaw );
	}

	/**
	 * Theory of Applied Robotics. p121
	 * @param rad_z0
	 * @param rad_x
	 * @param rad_z1
	 * R = Rz0 --> Rx --> Rz1
	 * rot.set_euler_zxz( DEG(30), DEG(45), DEG(60) );
	 * rot = 0.126826 -0.926777  0.353553
	 *       0.780330 -0.126826 -0.612372
	 *       0.612372  0.353553  0.707107
	 */
	void set_euler_zxz(double rad_z0, double rad_x, double rad_z1)
	{
		double c1 = cos(rad_z0);
		double s1 = sin(rad_z0);
		double c2  = cos(rad_x);
		double s2  = sin(rad_x);
		double c3 = cos(rad_z1);
		double s3 = sin(rad_z1);

		at(0,0) =  c1*c3 - c2*s1*s3;
		at(0,1) = -c1*s3 - c2*c3*s1;
		at(0,2) =  s2*s1;


		at(1,0) =  c3*s1 + c2*c1*s3;
		at(1,1) = -s1*s3 + c2*c1*c3;
		at(1,2) = -c1*s2;

		at(2,0) =  s2*s3;
		at(2,1) =  s2*c3;
		at(2,2) =  c2;
	}


	/** Theory of Applied Robotics. page 121
	*    R = Rz0 --> Rx --> Rz1
	*    note: euler has singularity at Y = -90/+90 deg
	*/
	vec3 get_euler_zxz() const
	{
		double z0 = -atan( at(0,2) / at(1,2) );
		double z1 =  atan( at(2,0) / at(2,1) );
		double  x =  atan( (at(0,2)*sin(z0) - at(1,2)*cos(z0)) / at(2,2) ); //stable method

		return vec3(z0, x, z1);
	}


	/* B = M A */
	/* know vector_A and vector_B find rotation matrix M */
	friend void solve_matrix_rotation(vec3& normalB, matrix_rot& M, const vec3& normalA)
	{
		const vec3 crossAB = cross( normalA, normalB);
		double mag = crossAB.mag();
		double rad = asin( mag );
		printf("mag %f -- rad %f\n", mag, rad ); fflush( stdout );
		const vec3 axis = crossAB.get_normalize();

		M.set_axis_angle( axis, rad );
	}


//	virtual matrix_rot inv() const
//	{
//		return static_cast<matrix_rot>( matrix_core::inv() );
//	}

};




class matrix_homo: public matrix_rot
{
protected:

	vec3 vec;

public:
	matrix_homo()
		:matrix_rot()
	{

	}

	matrix_homo(const matrix_rot& A)
		:matrix_rot(A)
	{

	}

	matrix_homo(matrix_rot& A, vec3& vec)
		:matrix_rot(A)
	{
		this->vec = vec;
	}


	matrix_homo(const matrix_rot& A, const vec3& vec)
		:matrix_rot(A)
	{
		this->vec = vec;
	}


	matrix_homo(const matrix_homo& A)
		:matrix_rot(A)
	{
		this->vec = A.vec;
	}


	matrix_homo& operator=(const matrix_homo& rhs)
	{
		if( this != &rhs )
		{
			matrix_container::operator=( rhs );
			this->vec = rhs.vec;
		}

		return *this;
	}


	virtual ~matrix_homo()
	{
	}


	void set_rot(matrix_rot& A)
	{
		at(0,0) = A(0,0);
		at(0,1) = A(0,1);
		at(0,2) = A(0,2);

		at(1,0) = A(1,0);
		at(1,1) = A(1,1);
		at(1,2) = A(1,2);

		at(2,0) = A(2,0);
		at(2,1) = A(2,1);
		at(2,2) = A(2,2);
	}

	void set_sphere(double z_rad, double y_rad, double radius)
	{
		double cos_y = cos(y_rad);
		double cos_z = cos(z_rad);
		double sin_y = sin(y_rad);
		double sin_z = sin(z_rad);

		at(0,0) =  cos_y  *  cos_z;
		at(0,0) = -sin_z          ;
		at(0,0) =  cos_z  *  sin_y;

		at(0,0) =  cos_y  *  sin_z;
		at(0,0) =  cos_z;
		at(0,0) =  sin_y  *  sin_z;

		at(0,0) = -sin_y;
		at(0,0) =  0.0;
		at(0,0) =  cos_y;

		this->vec(0) = radius * cos_z * sin_y;
		this->vec(1) = radius * sin_y * sin_z;
		this->vec(2) = radius * cos_y;
	}

	matrix_rot get_rot()
	{
		matrix_rot result;

		result(0,0) = at(0,0);
		result(0,1) = at(0,1);
		result(0,2) = at(0,2);

		result(1,0) = at(1,0);
		result(1,1) = at(1,1);
		result(1,2) = at(1,2);

		result(2,0) = at(2,0);
		result(2,1) = at(2,1);
		result(2,2) = at(2,2);

		return result;
	};


	vec3& get_pos()
	{
		return vec;
	};


	void set_pos(double x, double y, double z)
	{
		vec(x,y,z);
	}

	void set_pos(vec3& vec)
	{
		this->vec = vec;
	}

	void set_pos(const vec3& vec)
	{
		this->vec = vec;
	}


	virtual void debug(const char* message="")
	{
		printf("%s", message);
		for(size_t i=0; i < this->nrow  ; i++)
		{
			for(size_t j=0; j< this->ncol  ; j++)
			{
				std::cout << to_string( at(i,j)) << ((j!=( this->ncol -1)) ? ' ' : '|');
			}
			std::cout << to_string(vec(i)) << std::endl;
		}
	}

	virtual matrix_homo operator*(matrix_homo& A)
	{
		matrix_homo result;

		for(size_t i=0; i<A.nrow ; i++)
		{
			for(size_t k=0; k<A.ncol ; k++)
			{
				double sum = 0.0;
				for(size_t j=0; j<A.nrow ; j++)
					sum += ( at(i,j) * A(j,k) );
				result(i,k) = sum;
			}

			result.vec(i) = at(i,0) * A.vec(0) +
								at(i,1) * A.vec(1) +
								at(i,2) * A.vec(2) +
								this->vec(i);
		}

		return result;
	}


	virtual vec3 operator*(vec3& A)
	{
		vec3 result;

		for(int i=0; i<3; i++)
		{
			result(i) = at(i,0) * A(0) +
					    at(i,1) * A(1) +
				     	at(i,2) * A(2) +
				    	this->vec(i);
		}

		return result;
	}


	matrix_homo inverse()
	{
		matrix_core At = this->tp();
		matrix_homo result( *static_cast<matrix_rot*>(&At) );

		double a, b, c;
		a = result.at(0,0) * this->vec(0) +
				result.at(0,1) * this->vec(1) +
				result.at(0,2) * this->vec(2);

		b = result.at(1,0) * this->vec(0) +
				result.at(1,1) * this->vec(1) +
				result.at(1,2) * this->vec(2);

		c = result.at(2,0) * this->vec(0) +
				result.at(2,1) * this->vec(1) +
				result.at(2,2) * this->vec(2);

		result.vec(-a,-b,-c);

		return result;
	}

	//set center screw
	void set_screw(double h, double rad, vec3& u_hat)
	{
		this->set_axis_angle(u_hat, rad);
		this->vec( h * u_hat(0), h * u_hat(1), h * u_hat(2));

	}

	//set general screw
	//h - height per rotation of screw
	void set_screw(double h, double rad, vec3& u_hat, vec3& s_hat)
	{
		this->set_axis_angle(u_hat, rad);

		double vers = 1- cos(rad);
		double sin_r = sin(rad);
		double a, b, c;

		a = h*u_hat(0) - u_hat(0)*( s_hat(0)*u_hat(0) + s_hat(1)*u_hat(1) + s_hat(2)*u_hat(2)) * vers +
				(s_hat(1)*u_hat(2) - s_hat(2)*u_hat(1))*sin_r + s_hat(0)*vers;
		b = h*u_hat(1) - u_hat(1)*( s_hat(0)*u_hat(0) + s_hat(1)*u_hat(1) + s_hat(2)*u_hat(2)) * vers +
				(s_hat(2)*u_hat(0) - s_hat(0)*u_hat(2))*sin_r + s_hat(1)*vers;
		c = h*u_hat(2) - u_hat(2)*( s_hat(0)*u_hat(0) + s_hat(1)*u_hat(1) + s_hat(2)*u_hat(2)) * vers +
				(s_hat(0)*u_hat(1) - s_hat(1)*u_hat(0))*sin_r + s_hat(2)*vers;

		this->vec(a,b,c);
	}


};



class quaternion: public vec3
{

protected:

	void create(const quaternion& A)
	{
		this->e       = A.e;
		this->data[0] = A[0];
		this->data[1] = A[1];
		this->data[2] = A[2];
	}


	void create(const matrix_rot& A)
	{

		double trace = A(0,0) + A(1,1) + A(2,2);

		if( trace > M_EPSILON )
		{
			double s          = 0.5 / sqrt( 1.0 + trace );
			this->e           = 0.25 / s;
			this->data[0] = ( A(2,1) - A(1,2) ) * s;
			this->data[1] = ( A(0,2) - A(2,0) ) * s;
			this->data[2] = ( A(1,0) - A(0,1) ) * s;
		}
		else
		{
			if( (A(0,0) > A(1,1)) && (A(0,0) > A(2,2)) )  //A(0,0) à¸¡à¸²à¸à¸—à¸µà¹ˆà¸ªà¸¸à¸”
			{
				double s          = 2.0 * sqrt( 1.0 +  A(0,0) - A(1,1) - A(2,2) );
				this->e           = ( A(2,1) - A(1,2) ) / s;
				this->data[0] = 0.25 * s;
				this->data[1] = ( A(0,1) + A(1,0) ) / s;
				this->data[2] = ( A(0,2) + A(2,0) ) / s;
			}
			else if( A(1,1) > A(2,2) )  //A(1,1) à¸¡à¸²à¸à¸—à¸µà¹ˆà¸ªà¸¸à¸”
			{
				double s          = 2.0 * sqrt( 1.0 +  A(1,1) - A(0,0) - A(2,2) );
				this->e           = ( A(0,2) - A(2,0) ) / s;
				this->data[0] = ( A(0,1) + A(1,0) ) / s;
				this->data[1] = 0.25 * s;
				this->data[2] = ( A(1,2) + A(2,1) ) / s;
			}
			else //A(2,2) à¸¡à¸²à¸à¸—à¸µà¹ˆà¸ªà¸¸à¸”
			{
				double s          = 2.0 * sqrt( 1.0 +  A(2,2) - A(0,0) - A(1,1) );
				this->e           = ( A(1,0) - A(0,1) ) / s;
				this->data[0] = ( A(0,2) + A(2,0) ) / s;
				this->data[1] = ( A(1,2) + A(2,1) ) / s;
				this->data[2] = 0.25 * s;
			}

		}


	}



public:

	double e;

	quaternion()
		:vec3(0.0, 0.0, 0.0), e(1.0)
	{

	}

	quaternion(double rad, double q_i, double q_j, double q_k)
		:vec3()
	{
		double angle_rad_2 = rad/2.0;
		data[0] = q_i * sin(angle_rad_2);
		data[1] = q_j * sin(angle_rad_2);
		data[2] = q_k * sin(angle_rad_2);

		e = cos(angle_rad_2);
	}


	quaternion(double angle_rad, const vec3& axis)
		:vec3()
	{
		double angle_rad_2 = angle_rad/2.0;
		data[0] = axis[0] * sin(angle_rad_2);
		data[1] = axis[1] * sin(angle_rad_2);
		data[2] = axis[2] * sin(angle_rad_2);

		e = cos(angle_rad_2);
	}


	quaternion(double roll, double pitch, double yaw )
		:vec3()
	{
		set_roll_pitch_yaw( roll, pitch, yaw );
	}


	quaternion(matrix_rot& A)
		:vec3()
	{
		create(A);
	}


	quaternion(const matrix_rot& A)
		:vec3()
	{
		create(A);
	}


	void set_rad_axis(double rad, double u_i, double u_j, double u_k)
	{
		this->e           = cos(rad/2);
		this->data[0] = u_i*sin(rad/2);
		this->data[1] = u_j*sin(rad/2);
		this->data[2] = u_k*sin(rad/2);
	}


	void operator()(quaternion& A)
	{
		create(A);
	}


	double& operator()(int axis)
	{
		assert( (axis >= 0) && (axis <= 2) );
		return this->data[axis];
	};


	virtual void debug(const char* s="")
	{
		printf( "%s", s );
		printf("( %.3f , %.3f %.3f %.3f )\n",
				this->e, this->data[0], this->data[1], this->data[2] );

		matrix_rot rot;
		rot = this->to_matrix_rot();
		rot.debug();
	}


	quaternion& operator=(quaternion& A)
	{
		create(A);

		return *this;
	}

	//@ref "Math Primer for Computer Graphics and Game Development", p208-209
	void set_rotx(double rad)
	{
		double rad_2 = rad/2.0;
		this->e       = cos(rad_2);
		this->data[0] = sin(rad_2);
		this->data[1] = 0.0;
		this->data[2] = 0.0;
	}

	//@ref "Math Primer for Computer Graphics and Game Development", p208-209
	void set_roty(double rad)
	{
		double rad_2 = rad/2.0;
		this->e       = cos(rad_2);
		this->data[0] = 0.0;
		this->data[1] = sin(rad_2);
		this->data[2] = 0.0;
	}

	//@ref "Math Primer for Computer Graphics and Game Development", p208-209
	void set_rotz(double rad)
	{
		double rad_2 = rad/2.0;
		this->e       = cos(rad_2);
		this->data[0] = 0.0;
		this->data[1] = 0.0;
		this->data[2] = sin(rad_2);
	}

	virtual quaternion& operator=(const quaternion& A)
	{
		if (&A == this)
		{
			return *this;
		}

		create(A);
		return *this;
	}


	void operator()(matrix_rot& A)
	{
		create(A);
	};


	void operator()(const matrix_rot& A)
	{
		create(A);
	};


	void operator()(double e, double q_i, double q_j, double q_k)
	{
		this->e       = e;
		this->data[0] = q_i;
		this->data[1] = q_j;
		this->data[2] = q_k;
	}


	quaternion operator+(quaternion& A)
	{
		return quaternion( this->e       + A.e,
						   this->data[0] + A(0),
				           this->data[1] + A(1),
				           this->data[2] + A(2)
				         );
	}


	quaternion operator-(quaternion& A)
	{
		return quaternion( this->e       - A.e,
						   this->data[0] - A(0),
				           this->data[1] - A(1),
				           this->data[2] - A(2)
				         );
	}

	//@ref "Math Primer for Computer Graphics and Game Development", p163
	friend quaternion operator-(const quaternion& A)
	{
		return quaternion( -A.e,
						   -A.at(0),
				           -A.at(1),
				           -A.at(2)
				         );
	}


	// same as cross product
	//@ref "Math Primer for Computer Graphics and Game Development", p210
	quaternion operator*(quaternion& A)
	{
		// error when combine two rotation
		assert( false );
		double q, q_i, q_j, q_k;

		q =     (this->e       * A.e)  -
				(this->data[0] * A(0)) -
				(this->data[1] * A(1)) -
				(this->data[2] * A(2));

		q_i =   (this->e       * A(0)) +
				(this->data[0] * A.e)  +
				(this->data[1] * A(2)) -
				(this->data[2] * A(1));

		q_j =   (this->e       * A(1)) -
				(this->data[0] * A(2)) +
				(this->data[1] * A.e)  +
				(this->data[2] * A(0));

		q_k =   (this->e       * A(2)) +
				(this->data[0] * A(1)) -
				(this->data[1] * A(0)) +
				(this->data[2] * A.e );

		return  quaternion(q, q_i, q_j, q_k);
	}


	//@ref "Math Primer for Computer Graphics and Game Development", p165
	friend quaternion cross(const quaternion& A, const quaternion& B)
	{
		double dotAB = A.dot( static_cast<vec3>( B ) );
		vec3 crossBA = B.cross( static_cast<vec3>(A) );

		vec3 C = A.e*B + B.e*A + crossBA;

		return quaternion( A.e*B.e - dotAB, C );
	}



	friend double dot(const quaternion& A, const quaternion& B)
	{
		return A.e*B.e + A.at(0)*B.at(0) + A.at(1)*B.at(1) + A.at(2)*B.at(2);
	}


	//@ref "Math Primer for Computer Graphics and Game Programming", p163
	double size() const
	{
		return sqrt(    this->e          * this->e +
						this->data[0] * this->data[0] +
						this->data[1] * this->data[1] +
						this->data[2] * this->data[2] );
	}


	quaternion conj() const
	{
		return quaternion( this->e,
							-this->data[0] ,
							-this->data[1] ,
							-this->data[2] );
	}


	// inv() is conj() divide by its magnitude
	quaternion inv() const
	{
		double siz = this->size();
		double siz2 = siz* siz;

		quaternion qinv( e/siz2, -data[0]/siz2, -data[1]/siz2, -data[2]/siz2 );

		return qinv;
	}


	matrix_rot to_matrix_rot() const
	{
		matrix_rot m;

		double w = e;
		double x = data[0];
		double y = data[1];
		double z = data[2];

		double wx = w*x;
		double wy = w*y;
		double wz = w*z;

		double xy = x*y;
		double xz = x*z;
		double yz = y*z;

		//double w2 = w*w;
		double x2 = x*x;
		double y2 = y*y;
		double z2 = z*z;


		m(0,0) = 1 - 2*( y2 + z2 );
		m(0,1) = 2*( xy + wz );
		m(0,2) = 2*( xz - wy );


		m(1,0) = 2*( xy - wz );
		m(1,1) = 1 - 2*( x2 + z2 );
		m(1,2) = 2*( yz + wx );


		m(2,0) = 2*( xz + wy );
		m(2,1) = 2*( yz - wx );
		m(2,2) = 1 - 2*(x2 + y2);

		return m;

//		m(0,0) = 1 - 2*( y2 + z2 );
//		m(0,1) = 2*( xy - wz );
//		m(0,2) = 2*( wy + xz );
//
//		m(1,0) = 2*( wz + xy );
//		m(1,1) = 1 - 2*( x2 + z2 );
//		m(1,2) = 2*( yz - wx );
//
//		m(2,0) = 2*( xz - wy );
//		m(2,1) = 2*( wx + yz );
//		m(2,2) = 1 - 2*(x2 + y2);
//
//		return m;
	}


	vec3 rotate_vec(vec3& R) const
	{
		matrix_rot rot = this->to_matrix_rot();
		return  rot * R;
	}


	vec3 RotRd(vec3 R, vec3 d)
	{
		return rotate_vec( R ) + d;
	}


	double get_rad()
	{
		/* sometime e is out of [-1,1] */
		if( this->e > 1.0 )
		{
			this->e = 1.0;
		}
		else if(this->e < -1.0)
		{
			this->e = -1.0;
		}

		return 2*acos(this->e);
	}


	double get_u_size()
	{
		return sqrt( this->data[0]*this->data[0] +
					 this->data[1]*this->data[1] +
					 this->data[2]*this->data[2]);

	}


	virtual void nomalize()
	{
		double sz = this->size();

		if( sz > 0.0 )
		{
			this->e /= sz;
			this->data[0] /= sz;
			this->data[1] /= sz;
			this->data[2] /= sz;
		}
		else
		{
			assert( false );
			set_identity();
		}
	}


	quaternion get_normalize() const
	{
		quaternion norm( *this );
		double sz = norm.size();
		if( sz > 0.0 )
		{
			norm.e /= sz;
			norm.data[0] /= sz;
			norm.data[1] /= sz;
			norm.data[2] /= sz;
		}
		else
		{
			assert( false );
			norm.set_identity();
		}

		return norm;
	}


	void set_identity()
	{
		this->e = 1.0;
		this->data[0] = 0.0;
		this->data[1] = 0.0;
		this->data[2] = 0.0;
	}

	//@ref "Math Primer for Computer Graphics and Game Development", p162
	void set_axis_angle(double angle_rad, const vec3& axis)
	{
		double angle_rad_2 = angle_rad/2.0;
		data[0] = axis[0] * sin(angle_rad_2);
		data[1] = axis[1] * sin(angle_rad_2);
		data[2] = axis[2] * sin(angle_rad_2);

		e = cos(angle_rad_2);
	}


	//@ref "Math Primer for Computer Graphics and Game Development", p212
	void get_axis_angle(double& angle_rad, vec3& axis )
	{
		angle_rad = acos(2.0) * 2.0;

		double a = 1.0 -  e*e;
		if( a <= 0)
		{
			axis(1.0, 0.0, 0.0 );
			return;
		}

		double b = 1.0/sqrt( a );
		axis *= b;
	}


	// Rx --> Ry --> Rz
	void set_euler_xyz(const vec3& euler)
	{
		double sx = sin( euler[X]/2 );
		double sy = sin( euler[Y]/2 );
		double sz = sin( euler[Z]/2 );

		double cx = cos( euler[X]/2 );
		double cy = cos( euler[Y]/2 );
		double cz = cos( euler[Z]/2 );

		e       =  cx*cy*cz + sx*sy*sz;
		data[X] =  sx*cy*cz - cx*sy*sz;
		data[Y] =  cx*sy*cz + sx*cy*sz;
		data[Z] =  cx*cy*sz - sx*sy*cz;
	}


	// Rx --> Ry --> Rz
	void set_euler_zyx(const vec3& euler)
	{
		double sx = sin( euler[X]/2 );
		double sy = sin( euler[Y]/2 );
		double sz = sin( euler[Z]/2 );

		double cx = cos( euler[X]/2 );
		double cy = cos( euler[Y]/2 );
		double cz = cos( euler[Z]/2 );

		e       =  cx*cy*cz - sx*sy*sz;
		data[X] =  sx*cy*cz + cx*sy*sz;
		data[Y] =  cx*sy*cz - sx*cy*sz;
		data[Z] =  cx*cy*sz + sx*sy*cz;
	}


	void set_roll_pitch_yaw(double roll, double pitch, double yaw)
	{
		double roll_2 = roll/2;
		double sin_roll_2 = sin( roll_2 );
		double cos_roll_2 = cos( roll_2 );

		double pitch_2 = pitch/2;
		double sin_pitch_2 = sin( pitch_2 );
		double cos_pitch_2 = cos( pitch_2 );

		double yaw_2 = yaw/2;
		double sin_yaw_2 = sin( yaw_2 );
		double cos_yaw_2 = cos( yaw_2 );

		data[X] = cos_yaw_2*cos_pitch_2*cos_roll_2 + sin_yaw_2*sin_pitch_2*sin_roll_2;
		data[Y] = cos_yaw_2*cos_pitch_2*sin_roll_2 - sin_yaw_2*sin_pitch_2*cos_roll_2;
		data[Z] = cos_yaw_2*sin_pitch_2*cos_roll_2 + sin_yaw_2*cos_pitch_2*sin_roll_2;
		e       = sin_yaw_2*cos_pitch_2*cos_roll_2 - cos_yaw_2*sin_pitch_2*sin_roll_2;
	}


};

#endif /* KINEMATICS_HPP_ */
