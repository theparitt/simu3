/*
 * collision.hpp
 *
 *  Created on: Jan 4, 2015
 *      Author: theparitt
 */

#ifndef COLLISION_HPP_
#define COLLISION_HPP_


#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <fstream>
#include <memory>
#include <map>
#include <algorithm>
#include "primitive.hpp"
#include "mylog.hpp"

/*

		  __           ___                                                   ___                       ___           ___
	   	/\__\         /\  \                                                 /\__\                     /\  \         /\  \
	   /:/  /        /::\  \                                   ___         /:/ _/_       ___         /::\  \        \:\  \
	  /:/  /        /:/\:\  \                                 /\__\       /:/ /\  \     /\__\       /:/\:\  \        \:\  \
	 /:/  /  ___   /:/  \:\  \   ___     ___   ___     ___   /:/__/      /:/ /::\  \   /:/__/      /:/  \:\  \   _____\:\  \
	/:/__/  /\__\ /:/__/ \:\__\ /\  \   /\__\ /\  \   /\__\ /::\  \     /:/_/:/\:\__\ /::\  \     /:/__/ \:\__\ /::::::::\__\
	\:\  \ /:/  / \:\  \ /:/  / \:\  \ /:/  / \:\  \ /:/  / \/\:\  \__  \:\/:/ /:/  / \/\:\  \__  \:\  \ /:/  / \:\~~\~~\/__/
	 \:\  /:/  /   \:\  /:/  /   \:\  /:/  /   \:\  /:/  /     \:\/\__\  \::/ /:/  /     \:\/\__\  \:\  /:/  /   \:\  \
	  \:\/:/  /     \:\/:/  /     \:\/:/  /     \:\/:/  /       \::/  /   \/_/:/  /       \::/  /   \:\/:/  /     \:\  \
	   \::/  /       \::/  /       \::/  /       \::/  /        /:/  /      /:/  /        /:/  /     \::/  /       \:\__\
		\/__/         \/__/         \/__/         \/__/         \/__/       \/__/         \/__/       \/__/         \/__/


*/


/*
 * collision is a help class for detection collision between primitive_base object.
 * it distinguishes each primitive for a particular collision algorithm using
 * primitive ID ( myid ) that only have among "primitive_base" inheritances
*/



size_t collision_id = -1;
primitive_base* collision_prim = NULL;
primitive_base* collision_Bmain = NULL;

class collision
{

	/* a b c it point on triangle ABC in couter-clockwise direction */
	friend vec3 bary_to_point(const vec3& bary, const vec3& a, const vec3& b, const vec3& c)
	{
		const int u = 0;
		const int v = 1;
		const int w = 2;

		return ( 1 - bary[v] - bary[w] )*a +
				bary[v]*b +
				bary[w]*c;
	}


	/* a b c it point on triangle ABC in couter-clockwise direction */
	friend vec3 point_to_bary(const vec3& point, const vec3& a, const vec3& b, const vec3& c )
	{
		vec3 v0 = b - a;
		vec3 v1 = c - a;
		vec3 v2 = point - a;

		float d00 = dot( v0, v0 );
		float d01 = dot( v0, v1 );
		float d11 = dot( v1, v1 );
		float d20 = dot( v2, v0 );
		float d21 = dot( v2, v1 );

		float denom = d00*d11 - d01*d01;

		float v = (d11*d20 - d01*d21) / denom;
		float w = (d00*d21 - d01*d20) / denom;
		float u = 1.0f - v - w;

		return vec3( u, v, w );
	}


	/*
					  _       _                    _
					 (_)     | |                  | |
		  _ __   ___  _ _ __ | |_   ______   _ __ | | __ _ _ __   ___
		 | '_ \ / _ \| | '_ \| __| |______| | '_ \| |/ _` | '_ \ / _ \
		 | |_) | (_) | | | | | |_           | |_) | | (_| | | | |  __/
		 | .__/ \___/|_|_| |_|\__|          | .__/|_|\__,_|_| |_|\___|
		 | |                                | |
		 |_|                                |_|

	 */

	friend vec3 point_plane_closest_point(vec3& pointA,const vec3& normal, double offset)
	{
		double t = (normal.dot( pointA ) - offset) / (normal.dot( normal) );
		vec3 result = pointA;
		result -= (t*normal);

		return result;
	}


	// if return > 0 on plane surface
	// if return < 0 under plane surface
	/* p is a point */
	/* normal and offset are belong to plane */
	friend double point_plane_distance(const vec3& p, const vec3& normal, double offset )
	{
		return (dot( normal, p) - offset) / dot(normal, normal);
	}




	//book 141
	friend vec3 point_triangle_closest_point(const vec3& p, const triangle_t& tri) //
	{
		vec3 closest_p;// point on triangle tri that closest to point p

		const vec3& a = tri.A();
		const vec3& b = tri.B();
		const vec3& c = tri.C();

		vec3 ab = b - a;
		vec3 ac = c - a;
		vec3 ap = p - a;
		float d1 = dot( ab, ap );
		float d2 = dot( ac, ap );

		if( d1 <= 0.0f && d2 <= 0.0f )
			return a;

		vec3 bp = p - b;
		float d3 = dot(ab, bp);
		float d4 = dot(ac, bp);
		if( d3 >= 0.0f && d4 <= d3 )
			return b;

		float vc = d1*d4 - d3*d2;
		if( vc <= 0.0f && d1 >=0.0f && d3 <=0.0f)
		{
			float v = d1 / (d1-d3);
			return a + v*ab;
		}

		vec3 cp = p - c;
		float d5 = dot(ab, cp);
		float d6 = dot(ac, cp);
		if( d6>=0.0f && d5<=d6 )
			return c;

		float vb = d5*d2 - d1*d6;
		if( vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f )
		{
			float w = d2 / (d2 - d6);
			return a + w*ac;
		}

		float va = d3*d6 - d5*d4;
		if( va <= 0.0f && (d4-d3) >= 0.0f && ( d5 - d6) >= 0.0f )
		{
			float w = (d4 - d3) / ((d4-d3) + (d5-d6) );
			return b + w * (c - b);
		}


		float denom = 1.0f / ( va + vb + vc );
		float v = vb * denom;
		float w = vc * denom;

		return a + ab*v + ac*w;
	}


	friend void point_segment_closet_point(vec3& pointC, vec3& segmentA, vec3& segmentB, double& t, vec3& pointP)
	{
		vec3 lineAB = segmentB - segmentA;

		//project C on lineAB
		t  =  (pointC - segmentA).dot( lineAB );

		if( t < 0.0 )
		{
			t = 0.0;
			pointP = segmentA;
		}
		else
		{
			double denom = lineAB.dot( lineAB );
			if( t >= denom )
			{
				t = 1.0;
				pointP = segmentB;
			}
			else
			{
				t /= denom;
				pointP = segmentA + t*lineAB;
			}
		}

		pointP = segmentA + (t * lineAB);
	}


	friend bool point_triangle_inside(const vec3& p, const triangle_t& tri)
	{
		vec3 a = tri.A() - p;
		vec3 b = tri.B() - p;
		vec3 c = tri.C() - p;

		float ab = dot(a,b);
		float ac = dot(a,c);
		float bc = dot(b,c);
		float cc = dot(c,c);

		if( bc*ac - cc*ab < 0.0f)
			return false;

		float bb = dot(b,b);

		if( ab*bc - ac*bb < 0.0f)
			return false;

		return true;
	}


	friend double point_segment_square_distance(vec3& pointC, vec3& lineA, vec3& lineB)
	{
		vec3 lineAB = lineB - lineA;
		vec3 lineAC = pointC - lineA;
		vec3 lineBC = pointC - lineB;
		double e = lineAC.dot( lineAB );
		if( e <= 0)
			return lineAC.dot( lineAC );

		double f = lineAB.dot( lineAB );
		if( e >= f )
			return lineBC.dot( lineBC );

		return lineAC.dot( lineAC ) - e*e/f;
	}





//
//	friend vec3 sphere_triangle_closest_point(const sphere& s, const plane_base& p) //
//	{
//		vec3 closest_p = point_triangle_closest_point( s.get_center(), p );
//
//		return closest_p;
//	}
//



//
//
//	friend bool sphere_triangle_collision(const sphere& s, const plane_base& p) //
//	{
//		const vec3 &sphere_center = s.get_center();
//
//		vec3 closest_p = point_triangle_closest_point( sphere_center, p );
//		if( sqrt( distance_square( closest_p, sphere_center ) ) < s.radius )
//		{
//			return true;
//		}
//
//		return false;
//	}
//
//
//	friend bool sphere_triangle_collision(const sphere& s, const plane_base& p, vec3& closest_p)
//	{
//		const vec3 &sphere_center = s.get_center();
//
//		closest_p = point_triangle_closest_point( sphere_center, p );
//		if( sqrt( distance_square( closest_p, sphere_center ) ) < s.radius )
//		{
//			return true;
//		}
//
//		return false;
//	}
//
//
//
//	friend bool sphere_plane_inside(sphere* s, plane* p) //
//	{
//		double dist = s->get_center().dot( p->normal ) - p->offset;
//		return dist < -s->radius;
//	}



	/*----------------------------------------------------------------------------------
	 *      _       _                          _   _
	 *     (_)     | |                        | | (_)
	 *      _ _ __ | |_ ___ _ __ ___  ___  ___| |_ _ _ __   __ _
	 *     | | '_ \| __/ _ \ '__/ __|/ _ \/ __| __| | '_ \ / _` |
	 *     | | | | | ||  __/ |  \__ \  __/ (__| |_| | | | | (_| |
	 *     |_|_| |_|\__\___|_|  |___/\___|\___|\__|_|_| |_|\__, |               _
	 *     | (_)                                            __/ |              | |
	 *     | |_ _ __   ___                  ___  ___  __ _ |___/___   ___ _ __ | |_
	 *     | | | '_ \ / _ \                / __|/ _ \/ _` | '_ ` _ \ / _ \ '_ \| __|
	 *     | | | | | |  __/                \__ \  __/ (_| | | | | | |  __/ | | | |_
	 *     |_|_|_| |_|\___| _ _   _ ___    |___/\___|\__, |_| |_| |_|\___|_| |_|\__|
	 *             | '__/ _` | | | / __|              __/ |
	 *             | | | (_| | |_| \__ \             |___/
	 *             |_|  \__,_|\__, |___/
	 *                         __/ |
	 *                        |___/
	 ---------------------------------------------------------------------------------*/



	/*------------------------------------------------------------------------------*/
	/*                       line/ray/regment - plane collision                     */
	/*------------------------------------------------------------------------------*/
	/* line -    direction  no bounday p q is a part of line  -INF <= t <= INF      */
	/* ray  -    direction  one side boudary                     0 <= t <= INF      */
	/* segment - direction  but bound with p q                   0 <= t <= 1        */
	/*------------------------------------------------------------------------------*/


	/*-------------------------------*/
	/*  line/ray/segment  collision  */
	/*-------------------------------*/
	friend bool segment_plane_collision(const vec3& a, const vec3& b, const vec3& p_normal, double p_offset, double& t, vec3& colli_p)
	{
		vec3 ab = b - a;
		t = (p_offset - dot(p_normal, a) ) / dot( p_normal, ab );

		if( t >= 0.0f && t < 1.0f)
		{
			colli_p = a + t*ab;

			return true;
		}

		return false;
	}


	/* note: "Real-time Collision Detection". page 178*/
	friend bool segment_sphere_collision(const vec3& p, const vec3& q, const vec3& s_pos, const double s_radius, double t, vec3& colli_p )
	{
		vec3 pq = ( q - p);
		double mag = pq.mag();

		vec3 d = pq/mag;
		vec3 m = p - s_pos;
		double b = dot( m, d );
		double c = dot( m, m ) - (s_radius*s_radius);

		if( c > 0.0 && b > 0.0 )
			return false;

		 /* discriminat corresponds to ray missing sphere */
		double discr = b*b - c;

		if( discr < 0.0 )
			return false;

		t = -b - sqrt( discr );

		if( t > mag )
			return false;

		/* if t is negative, ray start inside the sphere */
		if( t < 0.0 )
			t = 0.0;

		colli_p = p + t*d;

		return true;
	}




	/* segment/ray collision */
	/* directional collision */
	/* segment p ---> q and triangle abc in couter-clockwise direction */
	/* ref: real-time collision detection, page 191 */
	friend bool segment_triangle_collision_barycentric(const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, double& t, double& u, double& v, double& w)
	{
		vec3 qp = p - q;
		vec3 ab = b - a; /* ab - line b <--- a */
		vec3 ac = c - a; /* ac - line c <--- a */

		const vec3& n = cross( ab, ac ); /* calculate triangle normal ( normal with magnitude ) */

		float d = dot(qp, n); /* d is the distance when project qp on the normal vector n */
		if( d <= 0.0f )
			return false;

		vec3 ap = p - a;
		t = dot(ap,n);

		if( t < 0.0f)
			return false;
		if( t > d ) /* ignore this for ray testing */
			return false;

		vec3 e = cross( qp, ap );

		v = dot( ac, e );
		if( v <0.0f || v>d)
			return false;
		w = -dot( ab, e);
		if( w <0.0f || v + w > d)
			return false;

		float ood = 1.0f / d;
		t *= ood;
		v *= ood;
		w *= ood;
		u = 1.0f - v - w;

		return true;
	}


	/* segment/ray collision */
	/* directional collision */
	/* segment p ---> q and triangle abc in couter-clockwise direction */
	/* ref: real-time collision detection, page 191 (modified) */
	/* if t >0 the direction of the collision is p---- hit---> q , if t <0  q---hit----> p */
	friend bool segment_triangle_collision_barycentric_nodirec(const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, double& t, double& u, double& v, double& w)
	{
		vec3 qp = p - q;
		vec3 ab = b - a; /* ab - line b <--- a */
		vec3 ac = c - a; /* ac - line c <--- a */

		const vec3& n = cross( ab, ac ); /* calculate triangle normal ( normal with magnitude ) */

		float d = dot(qp, n); /* d is the distance when project qp on the normal vector n */
		//printf("d: %f\n", d);
		if( d == 0.0 )
			return false;

		vec3 ap = p - a;
		t = dot(ap,n);
		//printf("t: %f\n", t);

		if( d >= 0.0)
		{
			if( t<0.0 || t>d ) /* ignore this for ray testing */
				return false;

			vec3 e = cross( qp, ap );

			v = dot( ac, e );
			if( v <0.0f || v>d)
				return false;
			w = -dot( ab, e);
			if( w <0.0f || v + w > d)
				return false;

			float ood = 1.0f / d;
			t *= ood;
			v *= ood;
			w *= ood;
			u = 1.0f - v - w;

			return true;
		}

		/*-----------*/
		/*   d < 0   */
		/*-----------*/
		if( t>0.0 || t<d )
			return false;

		vec3 e = cross( qp, ap );

		v = dot( ac, e );
		//printf("v: %f\n", v);
		if( v > 0.0f || v<d)
			return false;

		w = -dot( ab, e);
		//printf("v: %f\n", v);
		if( w > 0.0f || v + w < d)
			return false;

		float ood = 1.0f / d;
		t *= -ood;
		v *= ood;
		w *= ood;
		u = 1.0f - v - w;

		return true;
	}


	/* segment p-->q and quad abcd couter-clockwise */
	friend bool segment_quad_collision( const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, const vec3& d, vec3& colli_p )
	{
		double t, u, v, w;
		if ( segment_triangle_collision_barycentric( p, q, a, b, c, t, u, v, w ) )
		{
			vec3 bary( u, v, w );
			colli_p = bary_to_point( bary, a, b, c );
			return true;
		}

		if( segment_triangle_collision_barycentric( p, q, a, c, d, t, u, v, w ))
		{
			vec3 bary( u, v, w );
			colli_p = bary_to_point( bary, a, c, d );
			return true;
		}

		return false;
	}


	/* segment p-->q and quad abcd couter-clockwise */
	friend bool segment_quad_collision_nodirec( const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, const vec3& d, vec3& colli_p )
	{
		double t, u, v, w;
		if ( segment_triangle_collision_barycentric_nodirec( p, q, a, b, c, t, u, v, w ) )
		{
			vec3 bary( u, v, w );
			colli_p = bary_to_point( bary, a, b, c );

			return true;
		}

		if( segment_triangle_collision_barycentric_nodirec( p, q, a, c, d, t, u, v, w ))
		{
			vec3 bary( u, v, w );
			colli_p = bary_to_point( bary, a, c, d );

			return true;
		}

		return false;
	}


	/* segment sa->sb
		p = position of the center of top cap
		q = position of the center of bottom cap
		r = radius of both cap
		ref: "Real-time COllision Detection", page 197
	*/
	bool segment_cylinder_collision(const vec3& sa, const vec3& sb, const vec3& p, const vec3& q, double r, double& t)
	{
		vec3 d = q - p;
		vec3 m = sa - p;
		vec3 n = sb - sa;

		double md = dot( m, d );
		double nd = dot( n, d );
		double dd = dot( d, d );

		if( md < 0.0 && md + nd < 0.0 )
			return false;

		if( md > dd && md + nd > dd )
			return false;

		double nn = dot( n, n );
		double mn = dot( m, n );
		double a = dd*nn - nd*nd;
		double k = dot(m,m) - r*r;
		double c = dd*k - md*md;

		if( abs(a) < EPSILON )
		{
			if( c > 0.0 )
				return false;

			if( md < 0.0 )
				t = -mn/nn;
			else if( md > dd )
				t = (nd-mn)/nn;
			else
				t = 0.0;

			return true;
		}

		double b = dd*mn - nd*md;
		double discr = b*b - a*c;
		if( discr < 0.0 )
			return false;

		t = (-b-sqrt(discr)) / a;
		if( t < 0.0 || t > 1.0 )
			return false;

		if( md + t*nd < 0.0 )
		{
			if( nd <= 0.0 )
				return false;
			t = -md/nd;

			return k + 2*t*(mn + t*nn) <= 0.0;
		}
		else if( md + t*nd > dd)
		{
			if( nd >= 0.0 )
				return false;
			t = (dd - md)/nd;

			return k + dd - 2*md + t*(2*(mn - nd) + t*nn ) <= 0.0;
		}

		return true;
	}


	/*-------------------------------*/
	/*          ray collision        */
	/*-------------------------------*/
	friend bool ray_plane_collision(const vec3& a, const vec3& b, const vec3& p_normal, double p_offset, double& t, vec3& colli_p)
	{
		vec3 ab = b - a;
		t = (p_offset - dot(p_normal, a) ) / dot( p_normal, ab );

		if( t >= 0.0f )
		{
			colli_p = a + t*ab;

			return true;
		}

		return false;
	}


	/* ray collision is similar to segment_t collisition, except it does not check "t<=1" */
	/* directional collision */
	/* ray (p) ---> (q)  triangle abc couter-clockwise direction */
	/* ref: real-time collision detection, page 191 */
	friend bool ray_triangle_collision_barycentric(const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, double& t, double& u, double& v, double& w)
	{
		vec3 ab = b - a; /* ab - line b <--- a */
		vec3 ac = c - a; /* ac - line c <--- a */
		vec3 qp = p - q;

		const vec3& n = cross( ab, ac ); /* calculate triangle normal ( normal with magnitude ) */

		float d = dot(qp, n);
		if( d <= 0.0f )
			return false;

		vec3 ap = p - a;
		t = dot(ap,n);

		if( t < 0.0f )
			return false;

		vec3 e = cross( qp, ap );

		v = dot( ac, e );
		if( v <0.0f || v>d)
			return false;
		w = -dot( ab, e);
		if( w <0.0f || v + w > d)
			return false;

		float ood = 1.0f / d;
		t *= ood;
		v *= ood;
		w *= ood;
		u = 1.0f - v - w;

		return true;
	}


	/* ray p-->q and quad abcd couter-clockwise */
	friend bool ray_quad_collision( const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, const vec3& d, vec3& colli_p )
	{
		double t, u, v, w;
		if ( ray_triangle_collision_barycentric( p, q, a, b, c, t, u, v, w ) )
		{
			vec3 bary( u, v, w );
			colli_p = bary_to_point( bary, a, b, c );
			return true;
		}

		if( ray_triangle_collision_barycentric( p, q, a, c, d, t, u, v, w ))
		{
			vec3 bary( u, v, w );
			colli_p = bary_to_point( bary, a, c, d );
			return true;
		}

		return false;
	}


	/* note: Real-time Collision Detection. page 179*/
	friend bool ray_sphere_collision(const vec3& p, const vec3& q, const vec3& s_pos, const double s_radius, double t, vec3& colli_p )
	{
		vec3 d = (q - p).get_normalize();
		vec3 m = p - s_pos;
		double b = dot( m, d );
		double c = dot( m, m ) - (s_radius*s_radius);

		if( c > 0.0 && b > 0.0 )
			return false;

		 /* discriminat corresponds to ray missing sphere */
		double discr = b*b - c;

		if( discr < 0.0 )
			return false;

		t = -b - sqrt( discr );

		/* if t is negative, ray start inside the sphere */
		if( t < 0.0 )
			t = 0.0;

		colli_p = p + t*d;

		return true;
	}


	/* note: Real-time Collision Detection. page 179*/
	/* just test there is collision or not */
	friend bool ray_sphere_collision(const vec3& p, const vec3& q, const vec3& s_pos, const double s_radius )
	{
		vec3 d = (q - p).get_normalize();
		vec3 m = p - s_pos;
		double c = dot( m, m ) - (s_radius*s_radius);

		/* if there is at least one root, there must be an collision */
		if( c <= 0.0 )
			return true;

		double b = dot( m, d );
		if( b > 0.0 )
			return false;

		 /* discriminat corresponds to ray missing sphere */
		double disc = b*b - c;

		if( disc < 0.0 )
			return false;

		return true;
	}


	/* line is infinite length in both direction with direcion p--->q */
	/* ref: real-time collision detection, page 189 */
	friend bool line_quad_collision(const vec3& p, const vec3& q, const vec3& a, const vec3& b, const vec3& c, const vec3& d, vec3& colli_p)
	{
		vec3 pq = q - p;
		vec3 pa = a - p;
		vec3 pb = b - p;
		vec3 pc = c - p;

		vec3 m = cross( pc, pq );
		double v = dot( pa, m );
		if( v >= 0.0 )
		{
			/* test against triangle ABC */
			double u = -dot( pb, m );
			if( u < 0.0 )
				return false;

			double w = scalar_triple( pq, pb, pa );
			if( w < 0.0 )
				return false;

			double denom = 1.0/( u + v + w );
			u *= denom;
			v *= denom;
			w *= denom;

			colli_p = u*a + v*b + w*c;
		}
		else
		{
			/* test against triangle DAC */
			vec3 pd = d - p;
			double u = dot( pd, m );
			if( u < 0.0 )
				return false;

			double w = scalar_triple( pq, pa, pd );
			if( w < 0.0 )
				return false;

			v = -v;
			double denom = 1.0/( u + v + w );
			u *= denom;
			v *= denom;
			w *= denom;

			colli_p = u*a + v*d + w*c;
		}

		return true;
	}



//	//line - infinite line (direction)
//	friend bool line_triangle_collision_barycentric(vec3& p, vec3& q, triangle_base& tri, double& u, double& v, double& w)
//	{
//		vec3 &a = tri.A();
//		vec3 &b = tri.B();
//		vec3 &c = tri.C();
//
//		vec3 pq = q - p;
//		vec3 pa = a - p;
//		vec3 pb = b - p;
//		vec3 pc = c - p;
//
//		u = moo::scalar_triple( pq, pc, pb);
//		if( u < 0.0f )
//			return false;
//
//		v = moo::scalar_triple( pq, pa, pc );
//		if( v < 0.0f )
//			return false;
//
//		w = moo::scalar_triple( pq, pb, pa);
//		if( w < 0.0f )
//			return false;
//
//		float denom = 1.0f / ( u + v + w );
//		u *= denom;
//		v *= denom;
//		w *= denom;
//
//		return true;
//	}


//	friend bool line_triangle_collision(vec3& p, vec3& q, triangle_base& tri)
//	{
//		double u, v, w;
//		return line_triangle_collision_barycentric(p, q,  tri, u, v ,w);
//	}



//	//------------------------------------------//
//	//          dynamic collision               //
//	//------------------------------------------//
//	friend bool sphere_plane_dynamic_collision(const sphere& s, const vec3& v, const plane& p, float& t, vec3& q)
//	{
//		float dist = dot( p.normal, s.get_center() ) - p.offset;
//
//		//colloide now
//		if( abs( dist ) <= s.radius ) //t=0 there is collision
//		{
//			t = 0.0f;
//			q = s.get_center();
//			return true;
//		}
//		else // other t can be a collsiion
//		{
//			float denom = dot( p.normal, v );
//			if( denom*dist >= 0.0f )
//			{
//				//no intersection sphere move away from plane p
//				return false;
//			}
//			else
//			{
//				//sphere moving toward to plane p
//				float r = dist > 0.0f ? s.radius : -s.radius;
//				t = ( r - dist ) / denom;
//				q = s.get_center() + t*v - r*p.normal;
//				return true;
//			}
//
//		}// else
//
//	}//
//
//
//	// moving from sphere - pointA to pointB
//	bool sphere_plane_dynamic_collision( const vec3& pointA, const vec3& pointB, float r, const plane_base& p)
//	{
//		float adist = dot( pointA, p.normal ) - p.offset;
//		float bdist = dot( pointB, p.normal ) - p.offset;
//
//		if( adist * bdist < 0.0f )
//			return true;
//
//		if( abs(adist) <= r || abs(bdist) <= r )
//			return true;
//
//		return false;
//	}


	//	//-------------------------------------//
	//	//            mesh                     //
	//	//-------------------------------------//
	//	// point point to test inside mesh
	//	// point_out - point that known to be outside mesh
	//
	//	friend bool point_mesh_inside(const vec3& point, std::vector<plane_base*>& planev, const vec3& point_out)
	//	{
	//		bool inside = false;
	//		size_t hit_count = 0;
	//
	//
	//		for(size_t i=0; i< planev.size(); i++)
	//		{
	//			plane_base* p = planev[i];
	//
	//			bool coll = segment_nodirec_triangle_collision( point,  point_out, *p );
	//			if( coll )
	//			{
	//				hit_count++;
	//			}
	//		}
	//
	//		if( hit_count % 2 == 1 )
	//		{
	//			return true;
	//		}
	//
	//		return false;
	//
	//	}
	//
	//
	//	friend bool segment_nodirec_mesh_collision(const vec3& pointA, const vec3& pointB, std::vector<plane_base*>& planev, double& t, vec3& p_bary, plane_base* &tri_coll )
	//	{
	//		int segment_hit = 0;
	//		for(size_t i=0; i<planev.size(); i++)
	//		{
	//			plane_base* p = planev[i];
	//			double u, v, w;
	//			bool coll = segment_nodirec_triangle_collision( pointA, pointB, *p, p_bary  );
	//			if( coll )
	//			{
	//				p->set_color( 0.0, 0.0, 1.0 );
	//
	//
	//				segment_hit++;
	//				tri_coll = p;
	//			}
	//		}
	//
	//		if( segment_hit >= 1) //TODO.. this send only last triangle that hit
	//		{
	//			//printf("hit: %d\n", segment_hit ); fflush( stdout );
	//			return true;
	//		}
	//
	//		return false;
	//	}
	//
	//
	//	friend bool segment_mesh_collision(const vec3& pointA, const vec3& pointB, std::vector<plane_base*>& planev, double& t, vec3& p_bary, plane_base* &tri_coll )
	//	{
	//		int segment_hit = 0;
	//		for(size_t i=0; i<planev.size(); i++)
	//		{
	//			plane_base* p = planev[i];
	//			double u, v, w;
	//
	//			bool coll = segment_triangle_collision( pointA, pointB, *p, p_bary  );
	//			if( coll )
	//			{
	//				p->set_color( 0.0, 0.0, 1.0 );
	//
	//				segment_hit++;
	//				tri_coll = p;
	//			}
	//		}
	//
	//
	//		if( segment_hit >= 1) //TODO.. this send only last triangle that hit
	//		{
	//			printf("HIT TOTOL %d\n", segment_hit ); fflush( stdout );
	//			return true;
	//		}
	//
	//		return false;
	//	}
	//
	//

	//	friend bool line_mesh_collision(const vec3& pointA, const vec3& pointB, std::vector<plane>& planev )
	//	{
	//		int segment_hit = 0;
	//		for(size_t i=0; i<planev.size(); i++)
	//		{
	//			plane& p = planev[i];
	//			double u, v, w;
	//			bool coll = segment_nodirec_triangle_collision( pointA, pointB, p );
	//			if( coll )
	//			{
	//				segment_hit++; //TODO: can avoid this by making dynamic collsiion to avoid
	//			}
	//		}
	//
	//		if( segment_hit >= 2) //
	//		{
	//			return true;
	//		}
	//
	//		return false;
	//	}
	//
	//
	//
	//
	//	friend bool sphere_triangle_collision_staticdynamic(const sphere& s, const plane_base& p, const vec3& sphere_pos_prev, vec3& shadow_pos, float& depth)
	//	{
	//		vec3 closet_p;
	//		if( sphere_triangle_collision(s, p, closet_p) )
	//		{
	//			depth = s.radius - point_plane_distance( s.get_center(), p );
	//			shadow_pos = s.get_center() + p.normal * depth; //previous
	//			return true;
	//		}
	//
	//
	//
	//		vec3 p_bary;
	//		// directional segment
	//		bool coll = segment_triangle_collision( sphere_pos_prev, s.get_center(), p, p_bary  );
	//		if( coll )
	//		{
	//			vec3 direc = (s.get_center() - sphere_pos_prev).get_normalize();
	//			shadow_pos = p_bary - direc * s.radius;
	//
	//			vec3 collision_p = bary_to_point( p_bary, p );
	//			return true;
	//
	//		}
	//
	//
	//
	//		return false;
	//	}
	//
	//
	//
	//	friend bool sphere_triangle_collision_staticdynamic_bary(const sphere& s, const plane_base& p, const vec3& sphere_pos_prev,
	//			vec3& shadow_pos, float& depth, vec3& p_bary)
	//	{
	//		vec3 closet_p;
	//		if( sphere_triangle_collision(s, p, closet_p) )
	//		{
	//			depth = s.radius - point_plane_distance( s.get_center(), p );
	//			shadow_pos = s.get_center() + p.normal * depth; //previous
	//
	//			p_bary = point_to_bary(closet_p, p);//closet_p
	//			return true;
	//		}
	//
	//
	//
	//		// directional segment
	//		bool coll = segment_triangle_collision( sphere_pos_prev, s.get_center(), p, p_bary  );
	//		if( coll )
	//		{
	//			vec3 direc = (s.get_center() - sphere_pos_prev).get_normalize();
	//			shadow_pos = p_bary - direc * s.radius;
	//
	//			vec3 collision_p = bary_to_point( p_bary, p );
	//			return true;
	//
	//		}
	//
	//
	//
	//		return false;
	//	}


	friend double depth_segment_plane_collision(const vec3& seg_a, const vec3& seg_b,
												const vec3& normal, double offset, double t )
	{
		if( t > 0 )
			return point_plane_distance( seg_b, normal, offset );

		if( t < 0 )
			return point_plane_distance( seg_a, normal, offset );

		return 0.0;
	}


	friend double depth_segment_plane_collision(const vec3& seg_a, const vec3& seg_b, const vec3& normal, double offset )
	{
		double t = dot( (seg_b - seg_a), normal );
		//printf("======== t ========== %f\n", t );
		if( t < 0 )
			return point_plane_distance( seg_b, normal, offset );

		if( t > 0 )
			return point_plane_distance( seg_a, normal, offset );

		return 0.0;
	}

};



/* primA and primB are very primitive_object that cannot seperate anymore */
bool collision_primitive_dispatcher(primitive_base* Asimple, primitive_base* Bsimple, primitive_base* &Amain, primitive_base* &Bmain )
{

	assert( (Asimple != NULL) && (Bsimple != NULL) );



	const primitive_base* primA = ( Asimple->prim_id <  Bsimple->prim_id ) ? Asimple : Bsimple;
	const primitive_base* primB = ( Asimple->prim_id >= Bsimple->prim_id ) ? Asimple : Bsimple;

	bool hit = false;
	bool swap = ( primA == Asimple )? false: true ;

//	printf("prim A %d\n", primA->prim_id );
//	printf("prim B %d\n", primB->prim_id );
//	mylog::d( primA->name, "primA name: ");
//	mylog::d( primB->name, "primB name: ");

	/*-------------------*/
	/*      segment      */
	/*-------------------*/
	if( primA->prim_id == SEGMENT_ID )
	{
		segment_t* seg = (segment_t*)primA;

		/*---------------------------*/
		/*      segment/triangle     */
		/*---------------------------*/
		if( primB->prim_id == TRIANGLE_ID )
		{
			triangle_base* tri = (triangle_base*)primB;
			double t;       /* t position of segment 'seg' */
			double u, v, w; /* barycentric point of triangle 'tri' */

			/* test collision with different between direction/ non-directional segment */
			if( seg->direc ) /* directional segment collision */
				hit = segment_triangle_collision_barycentric( seg->A(), seg->B(), tri->A(), tri->B(), tri->C(), t, u, v, w );
			else /* non-directional segment collision */
				hit = segment_triangle_collision_barycentric_nodirec( seg->A(), seg->B(), tri->A(), tri->B(), tri->C(), t, u, v, w );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("segment-triangle HIT", 10, 60);

				vec3 colli_p = bary_to_point( vec3(u,v,w), tri->A(), tri->B(), tri->C() );
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );


				double depth = depth_segment_plane_collision( seg->A(), seg->B(), tri->normal, tri->offset, t );

				/*-------------------------------------------*/
				/* calculated information to store in object */
				/*-------------------------------------------*/
				seg->colli_p = colli_p;
				seg->colli_normal = -(seg->B() - seg->A()).get_normalize();//tri->normal;
				seg->is_colli = true;
				seg->colli_depth = abs(depth);


				tri->colli_p = colli_p;
				tri->colli_normal = -tri->normal;
				tri->is_colli = true;
				tri->colli_depth = abs( depth );

				/*-----------------*/
				/* calculate force */
				/*-----------------*/
				vec3 force = depth*tri->normal;
				force.debug("immediate force:  ");
				printf("depth %f\n", depth); fflush( stdout );

				Amain->add_force(  force[0], force[1], force[2] );
				Bmain->add_force(  force[0], force[1], force[2] );
			}

		} /* segment - triangle */
		/*---------------------------*/
		/*      segment/quad         */
		/*---------------------------*/
		else if( primB->prim_id == QUAD_ID )
		{
			rectangle_base* quad = (rectangle_base*)primB;
			vec3 colli_p;

			if( seg->direc ) /* find collision segment quad with 'directional segment' */
				hit = segment_quad_collision( seg->A(), seg->B(), quad->A(), quad->B(), quad->C(), quad->D(), colli_p );
			else  /* find collision segment quad with 'non-directional segment' */
				hit = segment_quad_collision_nodirec(  seg->A(), seg->B(), quad->A(), quad->B(), quad->C(), quad->D(), colli_p );

			if( hit )
			{
				//colli_p.debug("colli p");
				//stoke_colori( 242, 130, 129 );
				//draw_string("segment-regtangle HIT", 10, 60);
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );


				double depth = depth_segment_plane_collision( seg->A(), seg->B(), quad->normal, quad->offset );
				//printf("depth %f\n", depth );
				/*-------------------------------------------*/
				/* calculated information to store in object */
				/*-------------------------------------------*/
				seg->colli_p = colli_p;
				seg->colli_normal = (-seg->B() + seg->A()).get_normalize();//quad->normal;
				seg->is_colli = true;
				seg->colli_depth = abs(depth);

				//std::string msg = "collide with " + to_string( quad->myid );
				//draw_string( msg, 0.0, 10.0 );


				quad->colli_p = colli_p;
				quad->colli_normal = -quad->normal;
				quad->is_colli = true;
				quad->colli_depth = abs( depth );


				/* this is somehting just add */
				Amain->is_colli |= true;
				Bmain->is_colli |= true;


				collision_id = quad->myid; /* using in cutting */
				collision_prim = quad;

				collision_Bmain = !swap? Amain: Bmain;
				//quad->set_color( 1.0, 0.0, 0.5 );


				/*-----------------*/
				/* calculate force */
				/*-----------------*/
				vec3 force = depth*quad->normal;
				//force.debug("immediate force:  ");
				//printf("depth %f\n", depth); fflush( stdout );

				Amain->add_force(  force[0], force[1], force[2] );
				Bmain->add_force( -force[0], -force[1], -force[2] );

				//todo just add it - just add it
				const double factor = 400.0;
				quad->add_force( factor*force[0], factor*force[1], factor*force[2] );


				/* sneha collision */
				wave_collide_x = quad->myid % 100;
				wave_collide_z = quad->myid / 100;
			}

		}/* segment - quad */
		/*------------------------*/
		/*    segment/sphere      */
		/*------------------------*/
		else if( primB->prim_id == SPHERE_ID )
		{
			sphere_t* ball = (sphere_t*)primB;
			double t = 0.0;
			vec3 colli_p;
			hit = segment_sphere_collision( seg->A(), seg->B(), ball->A(), ball->radius, t, colli_p );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("seg-sphere HIT", 10, 60);
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );
			}

		} /* segment - sphere */
		/*------------------------*/
		/*      segment/plane     */
		/*------------------------*/
		else if( primB->prim_id == PLANE_ID )
		{
			plane_t* plane = (plane_t*)primB;
			double t = 0.0;
			vec3 colli_p;
			hit = segment_plane_collision( seg->A(), seg->B(), plane->normal, plane->offset,  t, colli_p );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("segment-plane HIT HIT", 10, 60);
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );


				double depth = depth_segment_plane_collision( seg->A(), seg->B(), plane->normal, plane->offset );
				//printf("depth %f\n", depth );

				/*-------------------------------------------*/
				/* calculated information to store in object */
				/*------------------8-------------------------*/
				seg->colli_p = colli_p;
				seg->colli_normal = plane->normal;
				seg->is_colli = true;
				seg->colli_depth = abs(depth);


				plane->colli_p = colli_p;
				//plane->colli_normal = -plane->normal;
				plane->is_colli = true;
				plane->colli_depth = abs( depth );

				/*-----------------*/
				/* calculate force */
				/*-----------------*/
				vec3 force = depth*plane->normal;
				force.debug("immediate force:  ");
				printf("depth %f\n", depth); fflush( stdout );

				Amain->add_force(  force[0], force[1], force[2] );
				Bmain->add_force( -force[0], -force[1], -force[2] );
			}

		} /* segment - plane */

	} /* segment */

	/*--------------*/
	/*     ray      */
	/*--------------*/
	else if( primA->prim_id == RAY_ID )
	{
		ray_t* ray = (ray_t*)primA;
		/*------------------------*/
		/*      ray/triangle      */
		/*------------------------*/
		if( primB->prim_id == TRIANGLE_ID )
		{
			triangle_t* tri = (triangle_t*)primB;
			double t;       /* t position of segment 'ray' */
			double u, v, w; /* barycentric point of triangle 'tri' */
			hit = ray_triangle_collision_barycentric( ray->A(), ray->B(), tri->A(), tri->B(), tri->C(), t, u, v, w );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("ray-traingle HIT HIT", 10, 60);
				vec3 colli_p = bary_to_point( vec3(u,v,w), tri->A(), tri->B(), tri->C() );
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );
			}

		} /* ray - triangle */
		/*------------------------*/
		/*      ray/quad          */
		/*------------------------*/
		else if( primB->prim_id == QUAD_ID )
		{
			rectangle_t* quad = (rectangle_t*)primB;
			vec3 colli_p;
			hit = ray_quad_collision( ray->A(), ray->B(), quad->A(), quad->B(), quad->C(), quad->D(), colli_p );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("ray-quad HIT HIT", 10, 60);
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );
			}

		} /* ray - quad */
		/*------------------------*/
		/*      ray/sphere        */
		/*------------------------*/
		else if( primB->prim_id == SPHERE_ID )
		{
			sphere_t* ball = (sphere_t*)primB;
			double t = 0.0;
			vec3 colli_p;
			hit = ray_sphere_collision( ray->A(), ray->B(), ball->A(), ball->radius, t, colli_p );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("ray-sphere HIT HIT", 10, 60);
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );
			}

		} /* ray - sphere */
		/*------------------------*/
		/*      ray/plane         */
		/*------------------------*/
		else if( primB->prim_id == PLANE_ID )
		{
			plane_t* plane = (plane_t*)primB;
			double t = 0.0;
			vec3 colli_p;
			hit = ray_plane_collision( ray->A(), ray->B(), plane->normal,plane->offset,  t, colli_p );

			if( hit )
			{
				stoke_colori( 242, 130, 129 );
				draw_string("ray-plane HIT HIT", 10, 60);
				stoke_color( 1.0, 0.0, 0.0 );
				draw_dot( colli_p, 10 );
			}
		} /* ray - plane */

	} /* ray */


	return hit;
}



void collision_main( primitive_base* objA, primitive_base* objB)
{
	if( !objA->collidable || !objB->collidable )
		return;

	bool collide = false;

	std::vector<primitive_base*> A_activev;
	std::vector<primitive_base*> A_passivev;

	std::vector<primitive_base*> B_activev;
	std::vector<primitive_base*> B_passivev;

	objA->get_active_collision( A_activev );
	objA->get_passive_collision( A_passivev );

	objB->get_active_collision( B_activev );
	objB->get_passive_collision( B_passivev );


	/*----------------*/
	/*  test A over B */
	/*----------------*/

	auto end_activeA = A_activev.end();
	auto end_passiveB = B_passivev.end();
	for( auto activeA=A_activev.begin();activeA < end_activeA; ++activeA )
	{
		primitive_base* primA = (*activeA);
		if( primA == NULL )
			continue;

		for( auto passiveB=B_passivev.begin(); passiveB<end_passiveB; ++passiveB)
		{
			primitive_base* primB = (*passiveB);
			if( primB == NULL )
				continue;

			collide |= collision_primitive_dispatcher( primA, primB, objA, objB );
		}
	}



	/*----------------*/
	/*  test B over A */
	/*----------------*/
	auto end_activeB = B_activev.end();
	auto end_passiveA = A_passivev.end();

	for( auto activeB=B_activev.begin();activeB < end_activeB; ++activeB )
	{
		primitive_base* primB = (*activeB);
		if( primB == NULL )
			continue;

		for( auto passiveA=A_passivev.begin(); passiveA<end_passiveA; ++passiveA)
		{
			primitive_base* primA = (*passiveA);
			if( primA == NULL )
				continue;

			collide |= collision_primitive_dispatcher( primB, primA, objA, objB );
		}
	}


	objA->is_colli |= collide;
	objB->is_colli |= collide;


//	std::cout << "[A]" << objA->name << ": "<< objA->is_colli << std::endl;
//	objA->get_force().debug("force");
//	objA->get_pos().debug("pos");
//
//
//	std::cout << "[B]" << objB->name << ": "<< objB->is_colli << std::endl;
//	objB->get_force().debug("force");
//	objB->get_pos().debug("pos");

}



const size_t OBJECT_MAX = 20;

class world_t
{

	static world_t* instance_;

	world_t()
	{
		objectv.resize( OBJECT_MAX );
		for(size_t i=0; i< OBJECT_MAX; i++)
		{
			objectv[i] = NULL;
		}
	}

public:

	std::vector< primitive_base* > objectv;

	static world_t* instance()
	{
		if( world_t::instance_ == NULL )
			world_t::instance_ = new world_t();

		return world_t::instance_;
	}


	int add_object(primitive_base* obj)
	{
		if( obj == NULL )
		{
			fprintf( stderr, "warning: add NULL obj to world" );
			return -1;
		}

		size_t i=0;
		for(; i< OBJECT_MAX; i++)
		{
			if( objectv[i] != NULL )
				continue;

			objectv[i] = obj;
			break;
		}

		return i;
	}


	primitive_base* get_object(size_t i)
	{
		assert( i < objectv.size() );

		return objectv[i];
	}


	void delete_object(size_t i)
	{
		assert( i < objectv.size() );
		assert( objectv[i] != NULL );

		delete objectv[i];
		objectv[i] = NULL;
	}


	//todo improve later
	void delete_inefficeint_way(primitive_base* obj)
	{
		for(size_t i=0; i<objectv.size(); i++)
		{
			if( objectv[i] == obj )
			{
				delete obj;
				obj = NULL;
				objectv[i] = NULL;
				return;
			}
		}

	}


	size_t object_count()
	{
		size_t count = 0;
		for(size_t i=0; i<OBJECT_MAX; i++)
		{
			if( objectv[i] != NULL )
				count++;
		}

		return count++;
	}


	/*------------------------
	 *         force
	 ------------------------*/
	void clear_all_force()
	{
		for(size_t i=0; i<OBJECT_MAX; i++)
		{
			object_base* obj = objectv[i];
			if( obj == NULL )
				continue;

			obj->clear_all_force( );
		}
	}


	void add_enviroment_force(const vec3& envi_force)
	{
		for(size_t i=0; i<OBJECT_MAX; i++)
		{
			object_base* obj = objectv[i];
			if( obj == NULL )
				continue;

			obj->add_force( envi_force[0], envi_force[1], envi_force[2] );
		}
	}


	void simulate(double dt)
	{
		for(size_t i=0; i<OBJECT_MAX; i++)
		{
			object_base* obj = objectv[i];
			if( obj == NULL )
				continue;

			obj->step_simulation( dt );
		}

	}


	void solve_collision_force()
	{
		for(size_t i=0; i<OBJECT_MAX-1; i++)
		{
			primitive_base* objA = objectv[i];
			if( objA == NULL )
				continue;

			if( !objA->collidable )
				continue;

			for(size_t j=i+1; j<OBJECT_MAX; j++)
			{
				primitive_base* objB = objectv[j];
				if( objB == NULL )
					continue;

				if( !objB->collidable )
					continue;

				collision_main( objA, objB );
			} /* for j */
		}/* for i */

	}


	/* must call after simulate(dt); */
	void render_scene()
	{
		for(size_t i=0; i<OBJECT_MAX; i++)
		{
			primitive_base* obj = objectv[i];
			if( obj == NULL )
				continue;

			obj->render_scene();
		}

	}

};



world_t* world_t::instance_ = NULL;





#endif /* COLLISION_HPP_ */
