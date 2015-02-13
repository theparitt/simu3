/*--------------------------------------------------------------------------------

   ▄▄▄█████▓ ██░ ██ ▓█████  ██▓███   ▄▄▄       ██▀███   ██▓▄▄▄█████▓▄▄▄█████▓
   ▓  ██▒ ▓▒▓██░ ██▒▓█   ▀ ▓██░  ██▒▒████▄    ▓██ ▒ ██▒▓██▒▓  ██▒ ▓▒▓  ██▒ ▓▒
   ▒ ▓██░ ▒░▒██▀▀██░▒███   ▓██░ ██▓▒▒██  ▀█▄  ▓██ ░▄█ ▒▒██▒▒ ▓██░ ▒░▒ ▓██░ ▒░
   ░ ▓██▓ ░ ░▓█ ░██ ▒▓█  ▄ ▒██▄█▓▒ ▒░██▄▄▄▄██ ▒██▀▀█▄  ░██░░ ▓██▓ ░ ░ ▓██▓ ░
     ▒██▒ ░ ░▓█▒░██▓░▒████▒▒██▒ ░  ░ ▓█   ▓██▒░██▓ ▒██▒░██░  ▒██▒ ░   ▒██▒ ░
     ▒ ░░    ▒ ░░▒░▒░░ ▒░ ░▒▓▒░ ░  ░ ▒▒   ▓▒█░░ ▒▓ ░▒▓░░▓    ▒ ░░     ▒ ░░
       ░     ▒ ░▒░ ░ ░ ░  ░░▒ ░       ▒   ▒▒ ░  ░▒ ░ ▒░ ▒ ░    ░        ░
     ░       ░  ░░ ░   ░   ░░         ░   ▒     ░░   ░  ▒ ░  ░        ░
             ░  ░  ░   ░  ░               ░  ░   ░      ░

----------------------------------------------------------------------------------*/
/*
 * primitve.hpp
 * Copyright (C) 2014 [theparitt]
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by theparit peerasathien <theparitt@gmail.com>, December 2014
 */


#ifndef PRIMITIVE_HPP_
#define PRIMITIVE_HPP_

#include <iostream>
#include <fstream>
#include <memory>
#include <map>
#include <algorithm>
#include <iterator>
#include <cmath>

#ifdef _WIN32
	#include <GL\freeglut.h>
	#include <GL\gl.h>
	#include <GL\glu.h>
	#include <GL\glut.h>

#elif __APPLE__
	#include <gl.h>
	#include <glu.h>
	#include <glut.h>
#endif


#include "SOIL.h"


#include "rk4.hpp"
#include "model.hpp"
#include "kinematics.hpp"
#include "mylog.hpp"


const double GRAVITY_Y = -9.80;


double sneha_depth = 0.0;


#ifndef isnan
	#define isnan(x) ((x)!=(x))
#endif


void random_color(vec3& color)
{
	color[0] = (rand()%10)/10.0f;
	color[1] = (rand()%10)/10.0f;
	color[2] = (rand()%10)/10.0f;
};


const int X_POS   = 0;
const int Y_POS   = 1;
const int Z_POS   = 2;
const int X_VEL   = 3;
const int Y_VEL   = 4;
const int Z_VEL   = 5;
const int X_FORCE = 6;
const int Y_FORCE = 7;
const int Z_FORCE = 8;
const int MASS    = 9;
const int X_ROT   = 10;
const int Y_ROT   = 11;
const int Z_ROT   = 12;
const int X_ANG   = 13;
const int Y_ANG   = 14;
const int Z_ANG   = 15;
const int X_TOQ   = 16;
const int Y_TOQ   = 17;
const int Z_TOQ   = 18;
const int INERTIA = 19;



class object_base: private model_t
{

	virtual void operator ()(float,const state_t& X, state_t& dX)
	{
		// if mass = 0, then the position is fixed
		if( X[MASS] == 0.0 )
		{
			for(size_t i=0; i<state_siz; i++)
			{
				dX[i] = 0.0;
			}
			return;
		}

        //---------------------------//
		//    implicit euler style   //
		//---------------------------//
		dX[X_POS] =  X[X_VEL];
		dX[Y_POS] =  X[Y_VEL];
		dX[Z_POS] =  X[Z_VEL];

		dX[X_VEL] =  X[X_FORCE] / X[MASS];
		dX[Y_VEL] =  X[Y_FORCE] / X[MASS];
		dX[Z_VEL] =  X[Z_FORCE] / X[MASS];

		//--------------//
		//    angular   //
		//--------------//
//		dX[X_ROT] =  X[X_ANG];
//		dX[Y_ROT] =  X[Y_ANG];
//		dX[Z_ROT] =  X[Z_ANG];
//
//		assert( X[INERTIA] != 0.0 );
//		dX[X_ANG] = X[X_TOQ] / X[INERTIA];
//		dX[Y_ANG] = X[Y_TOQ] / X[INERTIA];
//		dX[Z_ANG] = X[Z_TOQ] / X[INERTIA];
	}


	//local coordinate

public:

	std::string name;

	vec3 lpos;
	vec3 lrot;
	object_base* ref;
	vec3 color;

	int myid; //myid is different from primitive_id

	GLint *texture;
	std::vector<vec3> UVv; /* array of UV coordinate */
	material_t* mat;

	bool lframe_stage; /* true when lframe has change and other properties are up-to-date, false otherwise */
	bool collidable;

	object_base(object_base* ref=NULL)
		:model_t(20), name(""), lpos(0.0,0.0,0.0), lrot(0.0,0.0,0.0), ref( ref ), myid(-1), texture( NULL ), mat( NULL ), collidable( true )
	{
		init_state();
		color( 0.90, 0.90, 0.90 );

		state_curr[MASS] = 1.0;
		state_next[MASS] = 1.0;
		state_curr[INERTIA] = 1.0;
		state_next[INERTIA] = 1.0;

		lframe_stage = true;
	}


	virtual ~object_base()
	{

	}


	float state_of(int state_name) const
	{
		return state_curr[state_name];
	}


	float& state_of(int state_name )
	{
		lframe_stage = false;
		return state_curr[state_name];
	}


	virtual vec3 get_inertia(vec3 inerta)
	{
		return vec3(1.0,1.0,1.0);
	}


	void set_color(float r, float g, float b)
	{
		color[0] = r;
		color[1] = g;
		color[2] = b;
	}


	virtual void init_state()
	{
		assert( this != NULL );
		for(size_t i=0; i<state_siz; i++)
		{
			state_curr[i] = 0.0;
			state_next[i] = 0.0;
		}
	}


	void set_mass(double mass)
	{
		assert( this != NULL );
		state_curr[MASS] = mass;
	}


	/* initial forace to the frame */
	void set_force(float x, float y, float z)
	{
		assert( this != NULL );
		state_curr[X_FORCE] = x;
		state_curr[Y_FORCE] = y;
		state_curr[Z_FORCE] = z;
	}


	virtual void clear_all_force()
	{
		state_curr[X_FORCE] = 0.0;
		state_curr[Y_FORCE] = 0.0;
		state_curr[Z_FORCE] = 0.0;
	}


	/* add more force to the object */
	virtual void add_force(double x, double y, double z)
	{
		assert( this != NULL );
		state_curr[X_FORCE] += x;
		state_curr[Y_FORCE] += y;
		state_curr[Z_FORCE] += z;
	}


	/* set state position */
	void set_pos(float x, float y, float z)
	{
		assert( ref == NULL ); //just waring. this time only "global" object can use this function
		state_curr[X_POS] = x;
		state_curr[Y_POS] = y;
		state_curr[Z_POS] = z;

		lframe_stage = false;
	}


	/* set state position */
	void set_pos(const vec3& pos)
	{
		assert( ref == NULL );
		state_curr[X_POS] = pos[0];
		state_curr[Y_POS] = pos[1];
		state_curr[Z_POS] = pos[2];

		lframe_stage = false;
	}


	/* frame-to-frame relation */
	void set_local_pos(float x, float y, float z)
	{
		assert( ref != NULL);
		lpos( x, y, z);

		lframe_stage = false;
	}


	/* frame to frame relation */
	void set_local_pos(const vec3& pos)
	{
		assert( ref != NULL);
		lpos = pos;

		lframe_stage = false;
	}


	void set_local_rot(float rad_x, float rad_y, float rad_z)
	{
		assert( ref != NULL); //todo just this time, lock this fucntion for local object
		lrot( rad_x, rad_y, rad_z );

		lframe_stage = false;
	}


	void set_local_rot(const vec3& rot)
	{
		assert( ref != NULL); //TODO just this time, lock this fucntion for local object
		lrot = rot;

		lframe_stage = false;
	}


	/* get frame position */
	vec3 get_pos() const
	{
		return vec3( state_curr[X_POS], state_curr[Y_POS], state_curr[Z_POS] );
	}


	/* get frame position */
	void get_pos(double& x, double& y, double& z) const
	{
		x = state_curr[X_POS];
		y = state_curr[Y_POS];
		z = state_curr[Z_POS];
	}


	/* get force that acts to the frame */
	vec3 get_force() const
	{
		return vec3( state_curr[X_FORCE], state_curr[Y_FORCE], state_curr[Z_FORCE] );
	}


	/* get frame velocity */
	vec3 get_vel() const
	{
		return vec3( state_curr[X_VEL], state_curr[Y_VEL], state_curr[Z_VEL] );
	}


	/* set frame velocity */
	void set_vel(float vx, float vy, float vz)
	{
		state_curr[X_VEL] = vx;
		state_curr[Y_VEL] = vy;
		state_curr[Z_VEL] = vz;
	}


	/* set frame rotation roll-pitch-yaw */
	void set_rot(double x_rad, double y_rad, double z_rad)
	{
		assert( ref == NULL );
		state_curr[X_ROT] = x_rad;
		state_curr[Y_ROT] = y_rad;
		state_curr[Z_ROT] = z_rad;

		lframe_stage = false;
	}


	/* get frame oriention [roll pitch yaw] */
	vec3 get_rot() const
	{
		return vec3(state_curr[X_ROT], state_curr[Y_ROT], state_curr[Z_ROT] );
	}


	/* get frame orientation in quaternion format */
	quaternion get_rot_quaternion() const
	{
		/* createa quaternion with roll_pitch_yaw */
		return quaternion( state_curr[X_ROT], state_curr[Y_ROT], state_curr[Z_ROT] );
	}


	matrix_rot get_matrix_rot() const
	{
		matrix_rot result;
		result.set_roll_pitch_yaw( state_curr[X_ROT], state_curr[Y_ROT], state_curr[Z_ROT] );

		return result;
	}


	void get_axis_angle(vec3& axis, double& angle)
	{
		matrix_rot rot;
		rot.set_roll_pitch_yaw( state_curr[X_ROT], state_curr[Y_ROT], state_curr[Z_ROT] );
		rot.get_axis_angle( axis, angle );
	}


	// @concept:  call this function to get state_curr[*]
	// to get state_curr there are two tools that to help
	// [1] solver->solve is for calculate using rk4, then output will be place at state_next[*]
	// [2] update_state() will copy all data from state_next and put back to state_curr
	//
	// before calling this function, all force must be set to state_curr[*_FORCE]
	// if there is a force, must call solver->solve to get state_next[] then update
	// state_curr . by the end of this function state_curr[*] will be filled.
	// state[*_POS] state[*_VEL]
	virtual void step_simulation(float dt )
	{
		/* if mass is zero, no need to simulate */
		if( state_curr[MASS] == 0.0 )
			return;

		solver->solve( 0, dt, state_curr, state_next );
		update_state(); /* copy date from state_curr := state_next */

		post_update();
	}



	/* function that must call after calling step_simulation */
	virtual void post_update()
	{
		/* each object has different implementation of this function */
		/* override this function for each class, if need            */
	}


	/* draw with global "position" */
	virtual void render_scene()
	{
		glPushMatrix();
		draw();
		glPopMatrix();
	}


	/* update its frame to find its frame when reference to global_frame */
	void update_global_frame()
	{
		/* global_frame object */
		if( ref == NULL )
		{
			return;
		}

		/* local_frame object */
		const vec3 ref_grot = ref->get_rot(); /*get reference global_rotation frame */
		matrix_rot ref_gmrot( ref_grot[0], ref_grot[1], ref_grot[2] );
		/* find its local_frame rotation refereing to  its reference frame */
		matrix_rot lmrot( lrot[0], lrot[1], lrot[2] );

		/* find its frame according to the world */
		matrix_rot gmrot = ref_gmrot*lmrot;
		vec3 gpos = ref->get_pos() + gmrot*lpos;
		vec3 grot = gmrot.get_roll_pitch_yaw();

		/* update position & orientation to global frame */
		state_curr[X_POS] = gpos[0];
		state_curr[Y_POS] = gpos[1];
		state_curr[Z_POS] = gpos[2];
		state_curr[X_ROT] = grot[0];
		state_curr[Y_ROT] = grot[1];
		state_curr[Z_ROT] = grot[2];
	}


	/* when its frame move unrefencely with its refence frame, it need to update back */
	void update_local_frame()
	{
		if( ref == NULL )
		{
			return;
		}

		vec3 ref_rot = ref->get_rot();
		matrix_rot ref_mrot;
		ref_mrot.set_roll_pitch_yaw( ref_rot[0], ref_rot[1], ref_rot[2] );
		matrix_rot ref_minv = ref_mrot.inv();

		matrix_rot gmrot;
		gmrot.set_roll_pitch_yaw( state_curr[X_ROT], state_curr[Y_ROT], state_curr[Z_ROT]  );
		matrix_rot gmrot_inv( gmrot.inv() );

		matrix_rot lmrot = gmrot*ref_minv;
		lrot = lmrot.get_roll_pitch_yaw();

		vec3 gpos( state_curr[X_POS], state_curr[Y_POS], state_curr[Z_POS] );
		lpos = gmrot_inv*(gpos - ref->get_pos());
	}


	void check_ref_null()
	{
		if( ref == NULL )
		{
			printf("ref is NULL");
		}
		else
		{
			printf("ref is REF with other object");
		}

		fflush( stdout );
	}


	void set_UV(size_t i, float u, float v)
	{
		assert( i < UVv.size() );
		UVv[i](0) = u;
		UVv[i](1) = v;
		UVv[i](2) = 0.0;
	}


	/* load texture from file */
	virtual GLint* load_texture(const std::string filename)
	{
		texture = new GLint(
			SOIL_load_OGL_texture // load an image file directly as a new OpenGL texture
			(
				filename.c_str(),
				SOIL_LOAD_AUTO,
				SOIL_CREATE_NEW_ID,
				SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
			)
		);

		return texture;
	}




	/* set texture from pointer to texture_id */
	void set_texture(GLint* texture)
	{
		this->texture = texture;
	}


	void set_material(material_t* mat)
	{
		this->mat = mat;
	}


	void apply_material()
	{
		mat->apply_material();
	}


	void debug_pos(char* message) const
	{
		vec3 pos = get_pos();
		pos.debug( message );
	}

protected:
	//--------------------------------------------------//
	// like draw the object at (0,0,0)                //
	// dont have to worry about rotation or translation //
	//--------------------------------------------------//
	virtual void draw() = 0;

};



/*----------------------------------------------------------------------------
	            _           _ _   _                   _     _           _
	           (_)         (_) | (_)                 | |   (_)         | |
	 _ __  _ __ _ _ __ ___  _| |_ ___   _____    ___ | |__  _  ___  ___| |_
	| '_ \| '__| | '_ ` _ \| | __| \ \ / / _ \  / _ \| '_ \| |/ _ \/ __| __|
	| |_) | |  | | | | | | | | |_| |\ V /  __/ | (_) | |_) | |  __/ (__| |_
	| .__/|_|  |_|_| |_| |_|_|\__|_| \_/ \___|  \___/|_.__/| |\___|\___|\__|
	| |                                                   _/ |
	|_|                                                  |__/

-----------------------------------------------------------------------------*/




/*--------------*/
/* primitive id */
/*--------------*/
const int DOT_ID           = 0;
const int LINE_ID          = 1;
const int RAY_ID           = 2;
const int SEGMENT_ID       = 3; /* directional segment */
const int SEGMENTN_ID      = 4; /* non-direction segment */
const int PLANE_ID         = 5;
const int POLYGON_ID       = 6;
const int TRIANGLE_ID      = 7;
const int QUAD_ID          = 8;
const int CYLINDER_ID      = 9;
const int SPHERE_ID        = 10;

const int DOT_BODY_ID      = 11;
const int SPRING_BODY_ID   = 12;
const int TRIANGLE_BODY_ID = 13;
const int MULTI_BODY_ID    = 14;


const vec3 VEC3_ZERO(0.0,0.0,0.0);
const vec3 VEC3_X(1.0,0.0,0.0);
const vec3 VEC3_Y(0.0,1.0,0.0);
const vec3 VEC3_Z(0.0,0.0,1.0);




/**
* @class primitive_base
*
* @brief similar to object_base, except it can redender
*        and coliisionable, renderable
*
*
*/
class primitive_base: public object_base
{

protected:

	vec3* ref_normal;

public:
	/* primitive id to determine the type of itself */
	int prim_id;
	std::vector<object_base*> subv; /* hold subobject which reference to this object */

	/* 'pos' is the 'this' s position, it is like a orignin of the frame */
	/* 'ref' is the reference frame of this object, NULL is reference to the global frame */
	primitive_base(int prim_id, const vec3& frame_pos, object_base* ref)
		:object_base(ref), prim_id(prim_id), is_colli( false ), colli_depth( 0.0 )
	{
		/* 'this' is an object reference to global frame */
		if( ref==NULL )
		{
			set_pos( frame_pos ); /* set position as a global position */
			ref_normal = NULL;
		}
		else
		{	/* 'this' is an object reference to local frame, ref to 'ref' */
			set_local_pos( frame_pos ); /* set position as a local position reference to 'ref'object */
			ref_normal = new vec3( ref->get_pos() - frame_pos );
			subv.push_back( ref ); /*store sub object */
		}

	}


	virtual ~primitive_base()
	{
		if( ref_normal != NULL )
		{
			delete ref_normal;
			ref_normal = NULL;
		}

	}


	/* class primitive_t rotate object, but class dot_body dont rotate object */
	virtual void render_scene()
	{
		/*-----------------------------------*/
		/*  draw local frame for debugging   */
		/*-----------------------------------*/
		//draw_axis( get_pos(), get_rot(), 10.0 );
		glPushMatrix();
		draw(); /* draw must draw with global frame */
		glPopMatrix();


		/*------------------------------*/
		/*   draw collision point debug */
		/*------------------------------*/
		//stoke_color( 0.1, 0.8, 0.8 );
		//draw_dot( colli_p, 20 );

	}


	/* get colliable object_vector */
	/* objv will be add more objects */
	virtual bool get_active_collision(std::vector<primitive_base*>& objv)
	{
		objv.insert( objv.end(), this );
		return true; /* if done completely, all object in objv is final primitive */
	}


	/* get colliable object_vector */
	/* objv will be add more objects */
	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		objv.insert( objv.end(), this );
		return true; /* if done completely, all object in objv is final primitive */
	}


	virtual void step_simulation(float dt )
	{
		/* update is to chanage the 'this' object position to global position
		   if this object is already global_frame object, it does nothing */
		update_global_frame();
		/* simulate only global_object, local_object doesn't need to be simulated
		   becuase it changes its states according to its reference object */
		if( ref == NULL )
			object_base::step_simulation( dt );

		post_update();
	}


	/* update other composite object inside */
	virtual void post_update()
	{
		/* if there is subbody, it need to override this function */
	}


	virtual void step_simulation_lframe(float dt )
	{
		update_global_frame();
		object_base::step_simulation( dt );
	}


	/* collision point */
	vec3 colli_p;
	vec3 colli_normal;
	double colli_depth;
	bool is_colli;

	virtual vec3 get_collision_point() const
	{
		return colli_p;
	}


	virtual vec3 get_collision_normal() const
	{
		return colli_normal;
	}


};



class dot_t: public primitive_base
{

protected:

	/* draw with global frame */
	virtual void draw()
	{
		vec3 gpos = A();
		glColor3f( color[0], color[1], color[2] );
		glPointSize( 5.0 );
		glBegin(GL_POINTS); // render with points
			glVertex3f( gpos[0], gpos[1], gpos[2] ); //display a point
		glEnd();
	}

public:

	vec3 pointA; /* store the reference position to its frame */
	vec3 myrot;  /* self-rotaion relatively between rotating surrounding itself and the frame */

	/* create with pointA in the origin of its frame, the orientation of the point is also fix with the frame */
	dot_t(const vec3& pos, object_base* ref=NULL )
		:primitive_base( DOT_ID, pos, ref ), pointA(0.0,0.0,0.0), myrot(0.0,0.0,0.0)
	{

	}


	/* create with frame_pos and pointA are different positions */
	dot_t(const vec3& frame_pos, const vec3& pointA, object_base* ref=NULL )
		:primitive_base( DOT_ID, frame_pos, ref ), pointA( pointA ), myrot(0.0,0.0,0.0)
	{

	}


	float operator[](int i) const
	{
		return state_of(i);
	}


	float& operator[](int i)
	{
		return state_of(i);
	}


	/* get global position */
	vec3 A()
	{
		return get_pos();
	}


};



/*------------------------------------------------------------------------------*/
/*                       line/ray/regment - plane collision                     */
/*------------------------------------------------------------------------------*/
/* line -    direction  no bounday. p q is a part of line  -INF <= t <= INF     */
/* ray  -    direction  one side boudary                     0 <= t <= INF      */
/* segment - direction  but bound with p q                   0 <= t <= 1        */
/*------------------------------------------------------------------------------*/

class segment_t: public primitive_base
{

protected:

	/* draw is global position */
	/* must call after step_simulation(double dt). it needs to call update_global_frame() first */
	void draw()
	{
		stoke( 3.0, color[0], color[1], color[2] );
		/* draw from global position */
		draw_line( A(), B() );

		/*---------------------------*/
		/* draw letter for debugging */
		/*---------------------------*/
		vec3 posA = A();
		vec3 posB = B();

		draw_string("a", posA[0], posA[1], posA[2] );
		draw_string("b", posB[0], posB[1], posB[2] );
	}

	vec3 pointA;
	vec3 pointB;
	vec3 myrot; /* normally this is no need, it need when different orientation from its frame */

public:

	bool direc; /* directional or non-directional segment */

	/* create segment with pointA at origin of its local frame */
	segment_t(const vec3& pointA, const vec3& pointB, object_base* ref=NULL)
		:primitive_base( SEGMENT_ID, pointA, ref ), myrot(0.0,0.0,0.0), direc( true )
	{
		/* pointA is at the origin of its local frame */
		this->pointA(0.0,0.0,0.0);
		/* store pointB as local position relative to pointA */
		this->pointB = pointB - pointA;
	}


	/* create segment with its local frame position */
	segment_t(const vec3& frame_pos, const vec3& pointA, const vec3& pointB, object_base* ref=NULL)
		:primitive_base( SEGMENT_ID, frame_pos, ref ), pointA(pointA), pointB(pointB), myrot(0.0,0.0,0.0), direc( true )
	{
	}


	/* get global position of A */
	/* must call "update_global_frame()" before calling this function */
	vec3 A()
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		vec3 resultA = get_pos() + rot*pointA;
		return get_pos() + rot*pointA;
	}


	/* get global position of B */
	/* must call "update_global_frame()" before calling this function */
	vec3 B()
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		vec3 resultB = get_pos() + rot*pointB;
		return get_pos() + rot*pointB;
	}


	void set_nodirec()
	{
		direc = false;
	}


	void set_direc()
	{
		direc = true;
	}


	double length()
	{
		return mag( A(), B() );
	}


	/* get colliable object_vector */
	/* objv will be add more objects */
	virtual bool get_active_collision(std::vector<primitive_base*>& objv)
	{
		objv.insert( objv.end(), this );
		return true; /* if done completely, all object in objv is final primitive */
	}


	/* get colliable object_vector */
	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		/* no passive object */
		return true; /* if done completely, all object in objv is final primitive */
	}

};



class ray_t: public primitive_base
{

protected:

	/* draw is global position */
	/* must call after step_simulation(double dt). it needs to call update_global_frame() first */
	void draw()
	{
		stoke( 1.0, color[0], color[1], color[2] );
		/* draw from global position */
		vec3 posA = A();
		vec3 posB = B();
		draw_line( posA, posB );

		/*---------------------------*/
		/* draw letter for debugging */
		/*---------------------------*/
		draw_string("a", posA[0], posA[1], posA[2] );
		draw_string("...", posB[0], posB[1], posB[2] );
	}

	vec3 pointA;
	vec3 normal;
	vec3 myrot; /* normally this is no need, it need when different orientation from its frame */

public:

	/* create ray with pointA at origin of its local frame */
	ray_t(const vec3& pointA, const vec3& pointB, object_base* ref=NULL)
		:primitive_base( RAY_ID, pointA, ref ), myrot(0.0,0.0,0.0)
	{
		/* pointA is at the origin of its local frame */
		this->pointA(0.0,0.0,0.0);
		/* store pointB as local position relative to pointA */
		this->normal = (pointB - pointA).get_normalize();
	}


	/* create segment with its local frame position */
	ray_t(const vec3& frame_pos, const vec3& pointA, const vec3& pointB, object_base* ref=NULL)
		:primitive_base( RAY_ID, frame_pos, ref ), pointA(pointA), myrot(0.0,0.0,0.0)
	{
		this->normal = (pointB - pointA).get_normalize();
	}


	/* get global position of A */
	/* must call "update_global_frame()" before calling this function */
	vec3 A()
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointA;
	}


	/* get global position of B */
	/* must call "update_global_frame()" before calling this function */
	vec3 B()
	{
		const double SIZ = 20.0;
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*(pointA + SIZ*normal);
	}

};




class plane_base: public primitive_base
{

protected:

public:

	vec3 normal;
	double offset;

	plane_base( const double offset, const vec3& normal, object_base* ref=NULL )
		:primitive_base( PLANE_ID, vec3(0.0,0.0,0.0), ref ), normal( normal ), offset(offset)
	{

	}


	/* update normal & offset */
	virtual void update_normal() = 0;
};



/* infinite plane */
class plane_t: public primitive_base
{

protected:

	virtual void draw()
	{
		const double dist = 200.0; /* assume it is a large number */
		static const vec3 a( -dist,  dist, 0.0 );
		static const vec3 b( -dist, -dist, 0.0 );
		static const vec3 c(  dist, -dist, 0.0 );
		static const vec3 d(  dist,  dist, 0.0 );

		matrix_rot rot(  state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		vec3 pointA = get_pos() + rot*(a + offset);
		vec3 pointB = get_pos() + rot*(b + offset);
		vec3 pointC = get_pos() + rot*(c + offset);
		vec3 pointD = get_pos() + rot*(d + offset);

		if( texture == NULL )
		{
			glColor3f( color[0], color[1], color[2] );
			glBegin(GL_QUADS);
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointA[0], pointA[1], pointA[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointB[0], pointB[1], pointB[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointC[0], pointC[1], pointC[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointD[0], pointD[1], pointD[2] );
			glEnd();

			/*-----------------------*/
			/* draw line surrounding */
			/*-----------------------*/
			glLineWidth(3.0);
			//glColor3f(0.0f, 0.0f, 0.0f); // black
			glColor3ub( 61, 59, 56 );      // dark-brown
			glBegin(GL_LINE_LOOP );
				glVertex3f( pointA[0], pointA[1], pointA[2] );
				glVertex3f( pointB[0], pointB[1], pointB[2] );
				glVertex3f( pointC[0], pointC[1], pointC[2] );
				glVertex3f( pointD[0], pointD[1], pointD[2] );
			glEnd();
		}
		else
		{
			glColor3f( color[0], color[1], color[2] );
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, *texture );

			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
			glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );

			/* minify texture with factor 'r' with REPEAT texture */
			double r = 10.0;

			glBegin(GL_QUADS);
				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( r*UVv[0](0), r*UVv[0](1), r*0.0f);
				glVertex3f( pointA[0], pointA[1], pointA[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( r*UVv[1](0), r*UVv[1](1), r*0.0f );
				glVertex3f( pointB[0], pointB[1], pointB[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( r*UVv[2](0), r*UVv[2](1), r*0.0f );
				glVertex3f( pointC[0], pointC[1], pointC[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( r*UVv[3](0), r*UVv[3](1), r*0.0f );
				glVertex3f( pointD[0], pointD[1], pointD[2] );
			glEnd();

			glDisable(GL_TEXTURE_2D);
		}

		/*----------------------*/
		/*  draw normal vector  */
		/*----------------------*/
		vec3 mid = ( pointA + pointB + pointC + pointD ) / 4 ;
		double siz = ( pointB-pointA).mag();
		vec3 mide = mid + normal*siz;

		glLineWidth(3.0);
			glColor3f(1.0, 0.5, 0.5);
			glBegin(GL_LINES);
			glVertex3f( mid[0], mid[1], mid[2]);
			glVertex3f( mide[0], mide[1], mide[2] );
		glEnd();
	}


public:

	vec3 normal;
	double offset;

	/* plane normally create with offset, where its frame is reference to global frame */
	plane_t(const double offset, const vec3& normal, object_base* ref=NULL )
		:primitive_base( PLANE_ID, vec3(0.0,0.0,0.0), ref ), normal( normal ), offset(offset)
	{
		UVv.resize( 4 );
		/* convert from normal to row_pitch yaw */
		double pitch = atan2( normal[1]*normal[1] + normal[0]*normal[0], normal[2] );
		double yaw   = atan2( normal[1] , normal[0] );
		set_rot( 0.0, pitch, yaw );
	}


	/* call after update_global_frame */
	vec3 get_projection_point(const vec3& pointA)
	{
		double dist = dot(normal, pointA) + offset;
		return pointA - dist * normal;
	}

	/* after solve_motion and update_state, 'this' will call this function */
	virtual void post_update()
	{
		/* plane need to update normal vector after simulate */
		update_normal();
	}


	/* update normal & offset */
	virtual void update_normal()
	{
		matrix_rot rot(  state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );

		normal[0] =  rot(0,2);
		normal[1] =  rot(1,2);
		normal[2] =  rot(2,2);

		lframe_stage = true;
	}

};



class polygon_base: public primitive_base
{

public:

	vec3 normal;
	double offset;

	polygon_base(const double offset, const vec3& normal, object_base* ref=NULL )
		:primitive_base( POLYGON_ID, vec3(0.0,0.0,0.0), ref ), normal( normal ), offset( offset )
	{

	}


	/* fixme calculate normal wrong */
	/* update normal & offset */
	virtual void update_normal()
	{
		vec3 pointA( A() );
		vec3 pointB( B() );
		vec3 pointC( C() );

		normal = cross( pointB-pointA, pointC-pointA ).get_normalize();
		offset = dot( normal, pointA );

		lframe_stage = true;
	}


	/* the reason that I need to have 3 points on plane
	 * is because, I need to update normal_vector
	 * it seems a bit misconcept for plane but it is easy to program */
	virtual vec3 A() const = 0;
	virtual vec3 B() const = 0;
	virtual vec3 C() const = 0;


	virtual void post_update()
	{
		/* plane need to update normal vector after simulate */
		update_normal();
	}


	/* get colliable object_vector */
	/* objv will be add more objects */
	virtual bool get_active_collision(std::vector<primitive_base*>& objv)
	{
		/* no active object */
		return true; /* if done completely, all object in objv is final primitive */
	}


	/* get colliable object_vector */
	/* objv will be add more objects */
	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		objv.insert( objv.end(), this );
		return true; /* if done completely, all object in objv is final primitive */
	}

};



/* triangle_base is a cloas that know how to draw a triangle
 * however, the way to get coordinate A() B() C() is still
 * do not know in this step
 */
class triangle_base: public polygon_base
{

protected:

	/* draw with absolute position (global), need to override render_scene() */
	virtual void draw()
	{
		const vec3& pointA = A();
		const vec3& pointB = B();
		const vec3& pointC = C();

		if( texture == NULL )
		{
			glColor3f( color[0], color[1], color[2] );
			glBegin(GL_TRIANGLES);
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointA[0], pointA[1], pointA[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointB[0], pointB[1], pointB[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointC[0], pointC[1], pointC[2] );
			glEnd();

			/*-----------------------*/
			/* draw line surrounding */
			/*-----------------------*/
			glLineWidth(3.0);
			glColor3ub( 61, 59, 56 );
			glBegin(GL_LINE_LOOP );
				glVertex3f( pointA[0], pointA[1], pointA[2] );
				glVertex3f( pointB[0], pointB[1], pointB[2] );
				glVertex3f( pointC[0], pointC[1], pointC[2] );
			glEnd();
		}
		else
		{
			glColor3f( color[0], color[1], color[2] );
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, *texture );
			glBegin(GL_TRIANGLES);

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[0](0), UVv[0](1), 0.0f);
				glVertex3f( pointA[0], pointA[1], pointA[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[1](0), UVv[1](1), 0.0f );
				glVertex3f( pointB[0], pointB[1], pointB[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[2](0), UVv[2](1), 0.0f );
				glVertex3f( pointC[0], pointC[1], pointC[2] );

			glEnd();

			glDisable(GL_TEXTURE_2D);
		}

		/*----------------------*/
		/*  draw normal vector  */
		/*----------------------*/
//		vec3 mid = ( A() + B() + C() ) / 3 ;
//		double siz = 10;
//		vec3 mide = mid + normal*siz;
//
//		glLineWidth(3.0);
//			glColor3f(1.0, 0.5, 0.5);
//			glBegin(GL_LINES);
//			glVertex3f( mid[0], mid[1], mid[2]);
//			glVertex3f( mide[0], mide[1], mide[2] );
//		glEnd();

	}


public:

	triangle_base(double offset, const vec3& normal, object_base* ref=NULL)
		:polygon_base( offset, normal, ref )
	{
		prim_id = TRIANGLE_ID;
		UVv.resize( 3 );
	}

};



class triangle_t: public triangle_base
{

protected:


public:

	vec3 pointv[3];


	triangle_t(const vec3& frame_pos, const vec3& pointA, const vec3& pointB, const vec3& pointC, object_base* ref=NULL)
		:triangle_base(0, vec3(0.0,1.0,0.0), ref )
	{
		if( ref == NULL )
		{
			set_pos( frame_pos );
		}
		else
		{
			set_local_pos( frame_pos );
		}

		/* store point as local position */
		this->pointv[0] =  pointA;
		this->pointv[1] =  pointB;
		this->pointv[2] =  pointC;

		/* find normal and offset */
		update_normal();
		offset = dot( normal, pointA );
	}


	/*  pointA and local_frame_origin reside in different positions */
	triangle_t(const vec3& frame_pos, const vec3& pointA, const vec3& pointB, const vec3& pointC, const vec3& normal, object_base* ref=NULL)
		:triangle_base(0, normal, ref )
	{
		if( ref == NULL )
		{	/* set global position of 'this' */
			set_pos( frame_pos );
		}
		else
		{
			set_local_pos( frame_pos );
		}

		/* store point as local position */
		/* because we set this global to pointA, then A point as a local will be ZERO_VECTOR */
		this->pointv[0] =  pointA;
		this->pointv[1] =  pointB;
		this->pointv[2] =  pointC;

		offset = normal.dot( pointA );
	}


	/* create with pointA resides at origin of its local frame */
	static triangle_t* create(const vec3& pointA, const vec3& pointB, const vec3& pointC, object_base* ref=NULL)
	{
		triangle_t* tri = new triangle_t( pointA, vec3(0,0,0), pointB-pointA, pointC-pointA, ref );
		return tri;
	}


	/* create with pointA resides at origin of its local frame */
	static triangle_t* create(const vec3& pointA, const vec3& pointB, const vec3& pointC, const vec3& normal, object_base* ref=NULL)
	{
		triangle_t* tri = new triangle_t( pointA, vec3(0,0,0), pointB-pointA, pointC-pointA, normal, ref );
		return tri;
	}


	virtual ~triangle_t()
	{
	}


	/* global position of A */
	/* must call "update_global_frame()" before calling this function */
	virtual vec3 A() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[0];
	}


	/* global position of B */
	/* must call "update_global_frame()" before calling this function */
	virtual vec3 B() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[1];
	}


	/* global position of C */
	/* must call "update_global_frame()" before calling this function */
	virtual vec3 C() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[2];
	}


};



class rectangle_base: public polygon_base
{
protected:

	void draw()
	{
		const vec3& pointA = A();
		const vec3& pointB = B();
		const vec3& pointC = C();
		const vec3& pointD = D();

		if( texture == NULL )
		{
			glColor3f( color[0], color[1], color[2] );
			glBegin(GL_QUADS);
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointA[0], pointA[1], pointA[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointB[0], pointB[1], pointB[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointC[0], pointC[1], pointC[2] );
				glNormal3f( normal[0], normal[1], normal[2] );
				glVertex3f( pointD[0], pointD[1], pointD[2] );
			glEnd();

			/*-----------------------*/
			/* draw line surrounding */
			/*-----------------------*/
			glLineWidth(3.0);
			//glColor3f(0.0f, 0.0f, 0.0f); // black
			glColor3ub( 61, 59, 56 );      // dark-brown
			glBegin(GL_LINE_LOOP );
				glVertex3f( pointA[0], pointA[1], pointA[2] );
				glVertex3f( pointB[0], pointB[1], pointB[2] );
				glVertex3f( pointC[0], pointC[1], pointC[2] );
				glVertex3f( pointD[0], pointD[1], pointD[2] );
			glEnd();
		}
		else
		{
			glColor3f( color[0], color[1], color[2] );
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, *texture );
			glBegin(GL_QUADS);

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[0](0), UVv[0](1), 0.0f);
				glVertex3f( pointA[0], pointA[1], pointA[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[1](0), UVv[1](1), 0.0f );
				glVertex3f( pointB[0], pointB[1], pointB[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[2](0), UVv[2](1), 0.0f );
				glVertex3f( pointC[0], pointC[1], pointC[2] );

				glNormal3f( normal[0], normal[1], normal[2] );
				glTexCoord3f( UVv[3](0), UVv[3](1), 0.0f );
				glVertex3f( pointD[0], pointD[1], pointD[2] );
			glEnd();

			glDisable(GL_TEXTURE_2D);
		}

		/*----------------------*/
		/*  draw normal vector  */
		/*----------------------*/
//		vec3 mid = ( A() + B() + C() + D() ) / 4 ;
//		double siz = (B()-A()).mag();
//		vec3 mide = mid + normal*siz;
//
//		glLineWidth(3.0);
//			glColor3f(1.0, 0.5, 0.5);
//			glBegin(GL_LINES);
//			glVertex3f( mid[0], mid[1], mid[2]);
//			glVertex3f( mide[0], mide[1], mide[2] );
//		glEnd();


//		draw_string( "A", pointA[0], pointA[1], pointA[2] );
//		draw_string( "B", pointB[0], pointB[1], pointB[2] );
//		draw_string( "C", pointC[0], pointC[1], pointC[2] );
//		draw_string( "D", pointD[0], pointD[1], pointD[2] );

	}

public:

	rectangle_base(double offset, const vec3& normal, object_base* ref=NULL)
		:polygon_base(offset, normal, ref )
	{
		prim_id = QUAD_ID;
		UVv.resize( 4 );
	}


	virtual ~rectangle_base()
	{
	}


	virtual vec3 D() const = 0;

};



class rectangle_t: public rectangle_base
{

public:

	/* store local position */
	vec3 pointv[4];

	rectangle_t(const vec3& pointA, const vec3& pointB, const vec3& pointC, const vec3& pointD, object_base* ref=NULL)
		:rectangle_base( 0.0, vec3(0.0,1.0,0.0), ref )
	{
		if( ref == NULL)
		{
			set_pos( pointA );
		}
		else
		{
			set_local_pos( pointA );
		}

		vec3 A = pointB - pointA;
		vec3 B = pointC - pointA;
		vec3 C = cross( A, B );
		normal = C.get_normalize();
		offset = dot( normal, pointA );

		pointv[0] = vec3( 0.0, 0.0, 0.0 );
		pointv[1] = pointB - pointA;
		pointv[2] = pointC - pointA;
		pointv[3] = pointD - pointA;
	}


	rectangle_t(const vec3& frame_pos ,const vec3& pointA, const vec3& pointB, const vec3& pointC, const vec3& pointD, object_base* ref=NULL)
		:rectangle_base( 0.0, vec3(0.0,1.0,0.0), ref )
	{
		if( ref == NULL)
		{
			set_pos( frame_pos );
		}
		else
		{
			set_local_pos( frame_pos );
		}

		vec3 A = pointB - pointA;
		vec3 B = pointC - pointA;
		vec3 C = cross( A, B );
		normal = C.get_normalize();
		offset = dot( normal, pointA );

		pointv[0] = pointA;
		pointv[1] = pointB;
		pointv[2] = pointC;
		pointv[3] = pointD ;
	}


	rectangle_t(double width, double height, object_base* ref=NULL)
		:rectangle_base( 0.0, vec3(1.0,0.0,0.0), ref )
	{

		vec3 pointA(0    ,0      , 0);
		vec3 pointB(width,      0, 0);
		vec3 pointC(width, height, 0);
		vec3 pointD(    0, height, 0);

		if( ref == NULL)
		{
			set_pos( pointA );
		}
		else
		{
			set_local_pos( pointA );
		}

		vec3 A_line = pointB - pointA;
		vec3 B_line = pointC - pointA;
		vec3 C_line = cross( A_line, B_line );
		normal = C_line.get_normalize();
		offset = dot( normal, pointA );

		pointv[0] = vec3( 0.0, 0.0, 0.0 );
		pointv[1] = pointB - pointA;
		pointv[2] = pointC - pointA;
		pointv[3] = pointD - pointA;
	}


	rectangle_t(const vec3& frame_pos, double width, double height, object_base* ref=NULL)
		:rectangle_base( 0.0, vec3(1.0,0.0,0.0), ref )
	{

		vec3 pointA(0    ,0      , 0);
		vec3 pointB(width,      0, 0);
		vec3 pointC(width, height, 0);
		vec3 pointD(    0, height, 0);

		if( ref == NULL)
		{
			set_pos( frame_pos );
		}
		else
		{
			set_local_pos( frame_pos );
		}

		vec3 A_line = pointB - pointA;
		vec3 B_line = pointC - pointA;
		vec3 C_line = cross( A_line, B_line );
		normal = C_line.get_normalize();
		offset = dot( normal, pointA );

		pointv[0] = pointA;
		pointv[1] = pointB;
		pointv[2] = pointC;
		pointv[3] = pointD;
	}


	/* get global position of A */
	virtual vec3 A() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[0];
	}


	/* get global position of B */
	virtual vec3 B() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[1];
	}


	/* get global position of C */
	virtual vec3 C() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[2];
	}


	/* get global position of D */
	virtual vec3 D() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointv[3];
	}

};



class sphere_t: public primitive_base
{

public:

	double radius;
	GLint  nslice;
	GLint  nstack;
	vec3 pointA;

	/* when sphere_t reside at origin of its local frame */
	sphere_t(double radius, const vec3& pos, object_base* ref=NULL)
		:primitive_base( SPHERE_ID, pos, ref ), radius( radius ), pointA( 0.0, 0.0, 0.0 )
	{
		/* set default color to white */
		color( 0.95, 0.95, 0.95 );
		nslice = 20;
		nstack = 20;
	}


	sphere_t(const vec3& frame_pos, double radius, const vec3& pos, object_base* ref=NULL)
		:primitive_base( SPHERE_ID, frame_pos, ref ), radius( radius ), pointA( pos )
	{
		/* set default color to white */
		color( 0.95, 0.95, 0.95 );
		nslice = 20;
		nstack = 20;
	}


	vec3 A() const
	{
		matrix_rot rot( state_of(X_ROT), state_of(Y_ROT), state_of(Z_ROT) );
		return get_pos() + rot*pointA;
	}


protected:

	virtual void draw()
	{
		/*--------------------------*/
		/*       draw sphere        */
		/*--------------------------*/
		glColor3f( color[0], color[1], color[2] );
		if( texture == NULL)
		{
			glutSolidSphere(radius, nslice, nstack);
			/*---------------------------*/
			/* draw circle for debugging */
			/*---------------------------*/
//			glColor3f( 0.0f, 0.0f, 0.0f );
//			glPointSize( 2.0 );
//			glBegin(GL_LINE_LOOP);
//			for(int i=0;i<nslice;++i)
//			{
//				glVertex3f( radius*cos(2*M_PI*i/nslice) , radius*sin(2*M_PI*i/nslice), 0 );
//			}
//			glEnd();
		}
		else
		{	/* simple sphere textureing */
			GLUquadricObj* quad;

			glEnable( GL_TEXTURE_2D );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
			glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
			glBindTexture( GL_TEXTURE_2D, *texture );

			quad = gluNewQuadric();
			/* if true, texture is generate for quadric, false otherwise */
			gluQuadricTexture( quad, GLU_TRUE );
			gluSphere( quad, radius, nslice, nstack);
			gluDeleteQuadric( quad );

			glDisable( GL_TEXTURE_2D );
		}

		/*-------------------------*/
		/* draw line for debugging */
		/*-------------------------*/
		//draw_axis( radius * 2.0 );
	}




	virtual void render_scene()
	{
		glPushMatrix();

		//draw_axis( get_pos(), get_rot(), 10.0 );
		const vec3 gpos = A();
		glTranslatef( gpos[0], gpos[1], gpos[2] );
		//glTranslatef( state_of(X_POS), state_of(Y_POS), state_of(Z_POS) );
		/* need to rotate, because I assume draw is local position drawing */
		glRotated( RAD( state_of(Z_ROT)), 0, 0, 1 );
		glRotated( RAD( state_of(Y_ROT)), 0, 1, 0 );
		glRotated( RAD( state_of(X_ROT)), 1, 0, 0 );

		draw();
		glPopMatrix();
	}


public:

	void set_nslice_nstack(GLint nslice, GLint nstack)
	{
		this->nslice = nslice;
		this->nstack = nstack;
	}
};



class cylinder_t: public primitive_base
{


public:

	GLint* texture_top;  /* top-cylinder texture */
	GLint* texture_base; /* base-cylinder texture */
	bool   top_cap;
	bool   base_cap;
	double radius_base;
	double radius_top;

	double height;


	cylinder_t(double radius_base, double radius_top, double height, const vec3& pos, object_base* ref=NULL)
		:primitive_base( CYLINDER_ID, pos, ref), radius_base( radius_base ), radius_top(radius_top), height( height )
	{
		texture_top  = NULL;
		texture_base = NULL;
		top_cap = true;
		base_cap = true;
	}


protected:


	virtual void draw()
	{
		if( texture == NULL )
		{
			glColor3f( color[0], color[1], color[2] );
			GLUquadricObj *quadratic;
			quadratic = gluNewQuadric();
			/* gluCylinder( quad, base, top, height, slices, stacks) */
			gluCylinder(quadratic, radius_base, radius_top, height, 32, 32);
		}
		else
		{ 	/* draw with texture */
			glColor3f( color[0], color[1], color[2] );
			glEnable(GL_TEXTURE_2D);
			glBindTexture(GL_TEXTURE_2D, *texture );

			/*--------------------------*/
			/* draw top cap of cylinder */
			/*--------------------------*/
//			if( top_cap )
//			{
//				glBegin( GL_TRIANGLE_FAN );
//					glTexCoord2f( 0.5, 0.5 );
//					glVertex3f(0, height, 0);  /* center */
//					for(float i = 2 * M_PI; i >= 0.0f; i -= 0.2f)
//					{
//						glTexCoord2f( 0.5f * cos(i) + 0.5f, 0.5f * sin(i) + 0.5f );
//						glVertex3f(radius_top * cos(i), height, radius_top * sin(i) );
//					}
//					/* close the loop back to 0 degrees */
//					glTexCoord2f( 0.5, 0.5 );
//					glVertex3f(radius_top, height, 0);
//				glEnd();
//			}

			/*---------------------------*/
			/* draw base cap of cylinder */
			/*---------------------------*/
//			if( base_cap )
//			{
//				glBegin(GL_TRIANGLE_FAN);
//					glTexCoord2f( 0.5f, 0.5f );
//					glVertex3f(0, 0, 0);  /* center */
//					for (float i = 2 * M_PI; i >= 0.0f; i -= 0.2f)
//					{
//						glTexCoord2f( 0.5f * cos(i) + 0.5f, 0.5f * sin(i) + 0.5f );
//						glVertex3f(radius_base * cos(i), 0.0f, radius_base * sin(i));
//					}
//					glTexCoord2f( 0.5f, 0.5f );
//					glVertex3f(radius_base, 0.0f, 0.0f);
//				glEnd();
//			}

			/*------------------*/
			/* draw middle body */
			/*------------------*/
			glBegin(GL_QUAD_STRIP);
				for(float i = 0; i <= 2*M_PI; i+=0.2)
				{
					const float tc = ( i / (float)( 2*M_PI ) );
					glTexCoord2f( tc, 0.0);
					glVertex3f(radius_base * cos(i), 0, radius_base*sin(i));
					glTexCoord2f( tc, 1.0 );
					glVertex3f(radius_top * cos(i), height, radius_top*sin(i));
				}
				/* close the loop back to zero degrees */
				glTexCoord2f( 1.0, 0.0 );
				glVertex3f(radius_base, 0, 0);
				glTexCoord2f( 1.0, 1.0 );
				glVertex3f(radius_top, height, 0);
			glEnd();

			glDisable( GL_TEXTURE_2D );
		}

	}


	virtual void render_scene()
	{
		glPushMatrix();

		/* need to rotate, because I assume draw is local posiion drawing */
		glTranslatef( state_of(X_POS), state_of(Y_POS), state_of(Z_POS) );

		glRotated( RAD( state_of(Z_ROT)), 0, 0, 1 );
		glRotated( RAD( state_of(Y_ROT)), 0, 1, 0 );
		glRotated( RAD( state_of(X_ROT)), 1, 0, 0 );

		draw();
		glPopMatrix();
	}

};



/*------------------------------------------------------------------------------------
	                 _ _   _ _               _               _     _           _
		            | | | (_) |             | |             | |   (_)         | |
     _ __ ___  _   _| | |_ _| |__   ___   __| |_   _    ___ | |__  _  ___  ___| |_
	| '_ ` _ \| | | | | __| | '_ \ / _ \ / _` | | | |  / _ \| '_ \| |/ _ \/ __| __|
	| | | | | | |_| | | |_| | |_) | (_) | (_| | |_| | | (_) | |_) | |  __/ (__| |_
	|_| |_| |_|\__,_|_|\__|_|_.__/ \___/ \__,_|\__, |  \___/|_.__/| |\___|\___|\__|
									            __/ |            _/ |
								                |___/           |__/

-------------------------------------------------------------------------------------*/
class multi_body: public primitive_base
{

public:

	std::vector<primitive_base*> bodyv; //bodyv - vector of colliable object
	multi_body(const vec3 frame_pos, object_base* ref=NULL)
		:primitive_base(MULTI_BODY_ID, frame_pos, ref)
	{

	}


	virtual GLint* load_texture(const std::string filename)
	{
		texture = object_base::load_texture( filename );
		for(auto iter=bodyv.begin(); iter != bodyv.end(); ++iter)
		{
			if( (*iter) == NULL )
				continue;

			(*iter)->set_texture( texture );
		}

		return texture;
	}


	void set_UV(int iobject, int ivertex, int u, int v)
	{
		assert( iobject < bodyv.size() );
		object_base* body = bodyv[iobject];
		body->set_UV( ivertex, u, v );
	}


	virtual void post_update() = 0;

protected:

	virtual void draw()
	{
		silver_material.apply_material();
		for(size_t i=0; i<bodyv.size(); i++)
		{
			object_base* body  = bodyv[i];
			if( body == NULL )
				continue;

			body->render_scene();
		}
	}

};







class box_t: public multi_body
{

protected:


public:

	std::vector<primitive_base*> activev; /* store active collision_object */


	box_t(double width, double height, double depth, const vec3& pos, object_base* ref=NULL)
		:multi_body(pos, ref)
	{
		vec3 pointA(0    ,      0, depth);
		vec3 pointB(width,      0, depth);
		vec3 pointC(width, height, depth);
		vec3 pointD(    0, height, depth);

		vec3 pointE(0    ,0      , 0.0 );
		vec3 pointF(width,      0, 0.0 );
		vec3 pointG(width, height, 0.0 );
		vec3 pointH(    0, height, 0.0 );

		/*---------------------------------------*/
		/* make a active collision "segment" set */
		/*---------------------------------------*/
		segment_t* ab = new segment_t( pointA, pointB, this );
		segment_t* bc = new segment_t( pointB, pointC, this );
		segment_t* cd = new segment_t( pointC, pointD, this );
		segment_t* da = new segment_t( pointD, pointA, this );
		segment_t* ef = new segment_t( pointE, pointF, this );
		segment_t* fg = new segment_t( pointF, pointG, this );
		segment_t* gh = new segment_t( pointG, pointH, this );
		segment_t* he = new segment_t( pointH, pointE, this );

		segment_t* ae = new segment_t( pointA, pointE, this );
		segment_t* bf = new segment_t( pointB, pointF, this );
		segment_t* cg = new segment_t( pointC, pointG, this );
		segment_t* dh = new segment_t( pointD, pointH, this );

		ab->direc = false; ab->name = "box_ab";
		bc->direc = false; bc->name = "box_bc";
		cd->direc = false; cd->name = "box_cd";
		da->direc = false; da->name = "box_da";
		ef->direc = false; ef->name = "box_ef";
		fg->direc = false; fg->name = "box_fg";
		gh->direc = false; gh->name = "box_gh";
		he->direc = false; he->name = "box_he";

		ae->direc = false; ae->name = "box_ae";
		bf->direc = false; bf->name = "box_bf";
		cg->direc = false; cg->name = "box_cg";
		dh->direc = false; dh->name = "box_dh";
		activev.push_back( ab );
		activev.push_back( bc );
		activev.push_back( cd );
		activev.push_back( da );
		activev.push_back( ef );
		activev.push_back( fg );
		activev.push_back( gh );
		activev.push_back( he );
		activev.push_back( ae );
		activev.push_back( bf );
		activev.push_back( cg );
		activev.push_back( dh );


		bodyv.resize( 12 );
		/* front */
		bodyv[0] =  triangle_t::create( pointA, pointB, pointC, this );
		bodyv[1] =  triangle_t::create( pointA, pointC, pointD, this );

		/* back */
		bodyv[2] =  triangle_t::create( pointE, pointG, pointF, this );
		bodyv[3] =  triangle_t::create( pointE, pointH, pointG, this );

		/* buttom */
		bodyv[4] =  triangle_t::create( pointA, pointF, pointB, this );
		bodyv[5] =  triangle_t::create( pointA, pointE, pointF, this );

		/* top */
		bodyv[6] =  triangle_t::create( pointD, pointG, pointH, this );
		bodyv[7] =  triangle_t::create( pointD, pointC, pointG, this );

		/* left */
		bodyv[8] =  triangle_t::create( pointA, pointH, pointE, this );
		bodyv[9] =  triangle_t::create( pointA, pointD, pointH, this );

		/* right */
		bodyv[10] = triangle_t::create( pointB, pointF, pointG, this );
		bodyv[11] = triangle_t::create( pointB, pointG, pointC, this );
	}


	/* override method of update sub_body */
	virtual void post_update()
	{
		/* update composite body segment is for active collision*/
		for(size_t i=0; i<12; i++)
		{
			segment_t* seg = (segment_t*)activev[i];
			seg->update_global_frame();
		}

		/* update composited body */
		for(size_t i=0; i<12; i++)
		{
			triangle_t* tri = (triangle_t*)bodyv[i];
			assert( tri != NULL );

			tri->update_global_frame();
			/* no need for simulate, because all triangles is ref to "this" */
			tri->update_normal();
		}

	}


	void set_UV_auto()
	{
		/* texture front */
		set_UV(0,0, 0.0, 0.0);
		set_UV(0,1, 1.0, 0.0);
		set_UV(0,2, 1.0, 1.0);
		set_UV(1,0, 0.0, 0.0);
		set_UV(1,1, 1.0, 1.0);
		set_UV(1,2, 0.0, 1.0);

		/* texture back */
		set_UV(2,0, 1.0, 0.0); /* E */
		set_UV(2,1, 0.0, 1.0); /* G */
		set_UV(2,2, 0.0, 0.0); /* F */
		set_UV(3,0, 1.0, 0.0); /* E */
		set_UV(3,1, 1.0, 1.0); /* H */
		set_UV(3,2, 0.0, 1.0); /* G */

		/* texture bottom */
		set_UV(4,0, 1.0, 0.0); /* A */
		set_UV(4,1, 0.0, 1.0); /* F */
		set_UV(4,2, 0.0, 0.0); /* B */
		set_UV(5,0, 1.0, 0.0); /* A */
		set_UV(5,1, 1.0, 1.0); /* E */
		set_UV(5,2, 0.0, 1.0); /* F */

		/* texture top */
		set_UV(6,0, 0.0, 0.0); /* D */
		set_UV(6,1, 1.0, 1.0); /* G */
		set_UV(6,2, 0.0, 1.0); /* H */
		set_UV(7,0, 0.0, 0.0); /* D */
		set_UV(7,1, 1.0, 0.0); /* C */
		set_UV(7,2, 1.0, 1.0); /* G */

		/* texture left */
		set_UV(8,0, 1.0, 0.0); /* A */
		set_UV(8,1, 0.0, 1.0); /* H */
		set_UV(8,2, 0.0, 0.0); /* E */
		set_UV(9,0, 1.0, 0.0); /* A */
		set_UV(9,1, 1.0, 1.0); /* D */
		set_UV(9,2, 0.0, 1.0); /* H */

		/* texture right */
		set_UV(10,0, 0.0, 0.0); /* B */
		set_UV(10,1, 1.0, 0.0); /* F */
		set_UV(10,2, 1.0, 1.0); /* G */
		set_UV(11,0, 0.0, 0.0); /* B */
		set_UV(11,1, 1.0, 1.0); /* G */
		set_UV(11,2, 0.0, 1.0); /* C */
	}


	virtual bool get_active_collision(std::vector<primitive_base*>& objv )
	{
		objv.insert( objv.end(), activev.begin(), activev.end() );
		return true;
	}


	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		objv.insert( objv.end(), bodyv.begin(), bodyv.end() );
		return true;
	}

};



class dumbbell_t: public multi_body
{

public:

	dumbbell_t(const vec3& pos, object_base* ref=NULL)
		:multi_body( pos, ref )
	{
		bodyv.resize( 3 );
		double r = 2.0;
		bodyv[0] = new sphere_t  ( r, vec3( 0.0, 0.0, 0.0), this );
		bodyv[1] = new cylinder_t( 1.0, 1.0, r*3.0, vec3( 0.0, 0.0, 0.0), bodyv[0] );
		bodyv[2] = new sphere_t  (  r, vec3( r*3.0 ,0.0, 0.0), this );
		bodyv[1]->set_local_rot( 0.0, 0.0, DEG(-90.0) );
	}


	void post_update()
	{	/* update compisited body */
		for(size_t i=0; i<3; i++)
		{
			object_base* obj = (object_base*)bodyv[i];
			if( obj == NULL )
				continue;

			obj->update_global_frame();
		}
	}


};



/*----------------------------------------------------------------------------*
*		                  _               _               _                   *
*		                 (_)             | |             | |                  *
*		   ___ _ __  _ __ _ _ __   __ _  | |__   ___   __| |_   _             *
*		  / __| '_ \| '__| | '_ \ / _` | | '_ \ / _ \ / _` | | | |            *
*		  \__ \ |_) | |  | | | | | (_| | | |_) | (_) | (_| | |_| |            *
*		  |___/ .__/|_|  |_|_| |_|\__, | |_.__/ \___/ \__,_|\__, |            *
*		      | |                  __/ |                     __/ |            *
*		      |_|                 |___/                     |___/             *
*		                                                          	          *
*-----------------------------------------------------------------------------*/


const double SPRING_FORCE_MAX  =  10000.0;
const double SPRING_DAMPER_MAX =  10000.0;



// shouldn't call this class from main. should call from sdot_t only //
class spring_t: public object_base
{

protected:


public:

	double Kspring;
	double Kdamper;
	double Lrest;

	dot_t* dotA;
	dot_t* dotB;


	spring_t()
		:dotA(NULL), dotB(NULL)
	{
		Kspring = 1.0f;
		Kdamper = 1.0f;
		Lrest   = 1.0f;

		color( 0.0, 0.0, 0.0 );
	}


	double get_Kspring()
	{
		return Kspring;
	}


	double get_Kdamper()
	{
		return Kdamper;
	}


	spring_t(double Kspring, double Kdamper, double Lrest)
		:dotA(NULL), dotB(NULL)
	{
		this->Kspring = Kspring;
		this->Kdamper = Kdamper;
		this->Lrest   = Lrest;

		color( 0.0, 0.0, 0.0 );
	}


	void set_connection(dot_t* dotA, dot_t* dotB)
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotA != dotB );


		if( dotA < dotB )
		{
			this->dotA = dotA;
			this->dotB = dotB;
		}
		else
		{
			this->dotA = dotB;
			this->dotB = dotA;
		}
	}


	virtual void render_scene()
	{

#ifdef DEBUG
		vec3 axis;
		double angle;

		assert( dotA != NULL );
		assert( dotB != NULL );

		vec3 pointA = dotA->get_pos();
		vec3 pointB = dotB->get_pos();


		stoke(1.0, 0.0, 0.0, 0.0 );
		draw_line( pointA, pointB );

		vec3 v0 = vec3(0.0, 0.0, 1.0 ); //original object normal
		vec3 v1 = (pointB - pointA).get_normalize(); //new direction

		//v0.debug("v0");
		//v1.debug("v1");
		vec3 cross_v = cross( v0, v1 );

		double mag = cross_v.mag();
		//-----------------//
		// find axis-angle //
		//-----------------//
		if( mag == 0.0 ) //no need for translating or rotating
		{
			angle = 0.0;
			axis = v0;
		}
		else
		{
			angle = acos( dot(v0,v1) );
			axis  = cross_v/mag;
		}

		glPushMatrix();
		glTranslated( pointA[0], pointA[1], pointA[2] );
		glRotated( RAD(angle), axis[0], axis[1], axis[2] );
		draw();

		glPopMatrix();
#else
//		vec3 A = dotA->get_pos();
//		vec3 B = dotB->get_pos();
//		glBegin(GL_LINES);
//			glVertex3f( A[0], A[1], A[2]);
//			glVertex3f( B[0], B[1], B[2]);
//		glEnd();
#endif
	}

protected:

	virtual void draw()
	{
		assert( dotA != NULL );
		assert( dotB != NULL );

		vec3 pointA = dotA->get_pos();
		vec3 pointB = dotB->get_pos();

		const int nstack = 20;
		const float spring_radius = 0.1;

		double u = (pointB - pointA).mag() / nstack;
		vec3 center = vec3(0.0, 0.0, 1.0) * u;

		glColor3f( color[0], color[1], color[2] );
		glLineWidth( 1.0 );
		draw_circle( spring_radius, 2.0, nstack );
		for(int i=0; i<nstack; i++)
		{
			glTranslated( center[0], center[1], center[2] );
			draw_circle( spring_radius, 2.0, nstack );
		}
	}

public:

	// Xa - neighbor's state
	// Xi - current focus state
	vec3 spring_force_ab(const dot_t& Xa, const dot_t& Xi) const
	{
		static vec3 normal;
		static vec3 pos_diff;

		pos_diff[0] = (Xa[X_POS] - Xi[X_POS]);
		pos_diff[1] = (Xa[Y_POS] - Xi[Y_POS]);
		pos_diff[2] = (Xa[Z_POS] - Xi[Z_POS]);


		float mag = sqrt(pow(pos_diff[0],2) + pow(pos_diff[1],2) + pow(pos_diff[2],2) );
		assert( mag != 0.0 );
		normal[0] = pos_diff[0]/mag;
		normal[1] = pos_diff[1]/mag;
		normal[2] = pos_diff[2]/mag;

		//----------------------------------------------//
		vec3 Fspring = Kspring * (mag - Lrest) * normal;

		if( Fspring.mag() > SPRING_FORCE_MAX )
			Fspring = Fspring.get_normalize() * SPRING_FORCE_MAX;


		return Fspring;
	}


	// Xa - neighbor's state
	// Xi - current focus state
	// using damper bound limitation from paper
	// "Bound for Damping that Guarantee Stability in Mass-Spring Systems"
	//  Yogendra Bhasin and Alan Liu, Medicine Meets Virtual Reality, 2006
	vec3 damper_force_a(const dot_t& Xa, const dot_t& Xi, const vec3& Fspring_sum)
	{
		double mass = Xi[MASS];
		// if mass = 0, just skip //
		if( mass == 0.0 )
			return vec3( 0.0, 0.0, 0.0);

		double Ktemp = Kdamper;
		// find lower-bound Kdamper //
		double Kdamper_lower_bound = -2.0* sqrt( Xi[MASS]*Kspring );
		if( abs(Kdamper) < abs(Kdamper_lower_bound) )
		{
			//printf("warning: fix spring Kdamper lower-bound: %f\n", Kdamper); fflush( stdout );
			Ktemp = Kdamper_lower_bound;
		}


		// find upper-bound Kdamper //
		vec3 vel_diff;
		// relative velocity //
		vel_diff[0] = (Xi[X_VEL] - Xa[X_VEL]);
		vel_diff[1] = (Xi[Y_VEL] - Xa[Y_VEL]);
		vel_diff[2] = (Xi[Z_VEL] - Xa[Z_VEL]);

		double dt = 0.005; //approx dt
		vec3 vel( Xi[X_VEL], Xi[Y_VEL], Xi[Z_VEL] );
		double vel_mag = vel.mag();
		if( vel_mag == 0.0 )
			return vec3( 0.0, 0.0, 0.0);


		double Kdamper_upper_bound = -( vel * mass/dt + Fspring_sum ).mag() / vel_mag;


		// instalibity detect: no real value to change damper *reduce Kspring //
		//printf("lower bound:%f upper bound: %f\n", Kdamper_lower_bound, Kdamper_upper_bound );
		//fflush( stdout );

		// instability detect. cannot handle anything. good luck!
		//assert( abs(Kdamper_lower_bound) < abs(Kdamper_upper_bound) );
		if(  abs(Kdamper_lower_bound) > abs(Kdamper_upper_bound) )
		{
			//printf("unstable spring-mass-damper detected\n"); fflush( stdout );
			//printf("--- reduce mass to %f---\n", mass); fflush( stdout );
			//vel = 0.1 * vel;
			//Ktemp = Kdamper_lower_bound;//(Kdamper_lower_bound + Kdamper_upper_bound)/2.0;
			//printf("--- reduce Kspring to %f---\n", Kspring); fflush( stdout );
			//Kspring = 0.5 * Kspring;
		}
			//Ktemp = (Kdamper_lower_bound + Kdamper_upper_bound)/2.0;


		vec3 Fdamper = Ktemp* vel_diff ;
		if( Fdamper.mag() > SPRING_DAMPER_MAX )
			Fdamper = Fdamper.get_normalize() * SPRING_DAMPER_MAX;

		//printf("Ktemp: %f\n", Ktemp); fflush( stdout );
		//vel_diff.debug("vel difff: ");
		//Fdamper.debug("Fdamper: ");

		return Fdamper;
	}


};



const size_t MAX_DOTx = 50000;
const size_t MAX_SPRING_PER_DOT = 30;
const size_t MAX_SPING = MAX_DOTx*MAX_SPRING_PER_DOT;
const size_t MAX_NEIGHBOR = MAX_SPRING_PER_DOT;
const size_t MAX_FACE = 50000;



/* spring dot */
class sdot_t: public dot_t
{
protected:

	virtual void draw()
	{
		vec3 pos = A();
		glColor3f( color[0], color[1], color[2] );
		glPointSize( 5.0 );
		glBegin(GL_POINTS); // render with points
			glVertex3f( pos[0], pos[1], pos[2] ); //display a point
		glEnd();

		/*-------------------*/
		/*    debug string   */
		/*-------------------*/
		//std::string s = to_string( myid );
		//draw_string(s, pos[0], pos[1] );
	}

	std::vector<sdot_t*>   neighborv;
	std::vector<spring_t*> springv;

public:

	sdot_t( const vec3& pos, size_t id, object_base* ref=NULL )
		:dot_t( pos, ref )
	{
		/* no need to set pos as local or global, the contructor do the work arelady */
	    myid = id;
		name = to_string( myid );

		neighborv.reserve( MAX_NEIGHBOR );
		std::fill( neighborv.begin(), neighborv.end(), (sdot_t*)NULL );
		springv.reserve( MAX_NEIGHBOR );
		std::fill( springv.begin(), springv.end(), (spring_t*)NULL );
	}


	void add_neighbor(sdot_t* neighbor, spring_t* spring)
	{
		assert( neighbor != NULL );
		assert( spring != NULL );
		neighborv.push_back( neighbor );
		springv.push_back( spring );
	}


	size_t neighbor_size()
	{
		return neighborv.size();
	}


	void remove_neighbor(sdot_t* neighbor)
	{
		auto it = std::find( neighborv.begin(), neighborv.end(), neighbor);
		if( it != neighborv.end() )
		{
			size_t pos = it - neighborv.begin();
			neighborv.erase( it );
			springv.erase( springv.begin()+pos, springv.begin()+pos+1 );

			return;
		}
		assert( false );
	}


	bool is_neighbor(sdot_t* neighbor)
	{
		auto it = std::find( neighborv.begin(), neighborv.end(), neighbor);
		if( it == neighborv.end() )
			return false;

		return true;
	}


	void replace_neighbor(sdot_t* neighbor_old, sdot_t* neighbor_new )
	{
		std::replace( neighborv.begin(), neighborv.end(), neighbor_old, neighbor_new);
	}


	spring_t* get_spring_between(sdot_t* neighbor)
	{
		auto it = std::find( neighborv.begin(), neighborv.end(), neighbor);
		if( it != neighborv.end() )
		{
			size_t pos = it - neighborv.begin() ;
			assert( springv[pos] != NULL );
			return springv[ pos ];
		}
		assert( false ); //cannot find neighbor


		return NULL;
	}


	vec3 get_spring_damper_force()
	{
		const size_t siz = neighborv.size();

		//--- sum spring_force ---//
		vec3 spring_force_sum( 0.0, 0.0, 0.0 ); //force of all neighbor spring

		for(size_t i=0; i<siz; i++)
		{
			sdot_t* sdot = neighborv[i];
			assert( sdot != NULL );


			spring_t* spring = springv[i];
			assert( spring != NULL );
			spring_force_sum += spring->spring_force_ab( *sdot, *this );
		}

		//--- sum damper_force ---//
		vec3 damper_force_sum( 0.0, 0.0, 0.0 );
		for(size_t i=0; i<siz; i++)
		{
			sdot_t* sdot = neighborv[i];
			assert( sdot != NULL );


			spring_t* spring = springv[i];
			assert( spring != NULL );
			damper_force_sum += spring->damper_force_a( *sdot, *this, spring_force_sum );
		}

		return spring_force_sum + damper_force_sum;
	}


	void debug_neighborv(const char* prefix="")
	{
		for(size_t i=0; i<neighborv.size(); i++)
		{
			printf("%s neighbor: %d ", prefix, neighborv[i]->myid );
		}
	}

};



/*
 * dot_body_base is a class that have only dot. it doesnt have
 * a surface to render
 */
class dot_body_base: public primitive_base
{
protected:

	std::vector<sdot_t*> sdotv;

public:



	dot_body_base(const vec3 pos, object_base* ref=NULL)
		:primitive_base( DOT_BODY_ID, pos, ref )
	{
		name = "dot_body";
		sdotv.resize( MAX_DOTx );
		std::fill( sdotv.begin(), sdotv.end(), (sdot_t*)NULL );
	}


	size_t find_dot_myid(sdot_t* sdot)
	{
		auto it = std::find( sdotv.begin(), sdotv.end(), sdot );

		if( it == sdotv.end() )
			return -1;

		return it - sdotv.begin();
	}


	// set dot local position related to main_body (this)
	void set_dot_pos(int index, float x, float y, float z)
	{
		assert( sdotv[index] != NULL );
		sdotv[index]->set_local_pos(x, y, z);
	}


	void set_dot_mass(size_t index, float mass)
	{
		assert( index < MAX_DOTx );
		assert( sdotv[index] != NULL );
		sdotv[index]->set_mass( mass );
	}


	sdot_t* create_dot_rigidbody(size_t dot_index, const vec3& lpos)
	{
		assert( sdotv[dot_index] == NULL );
		sdotv[dot_index] = new sdot_t(lpos, dot_index,  this );
		sdotv[dot_index]->set_mass( 0.0 );

		sdotv[dot_index]->update_global_frame();
		return sdotv[dot_index];
	}


	virtual sdot_t* create_dot(size_t dot_index, const vec3& lpos)
	{
		assert( sdotv[dot_index] == NULL );
		sdotv[dot_index] = new sdot_t( lpos, dot_index, this );

		sdotv[dot_index]->update_global_frame();
		return sdotv[dot_index];
	}


	virtual sdot_t* create_dot(const vec3& lpos )
	{
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] != NULL)
				continue;

			sdotv[i] = new sdot_t( lpos, i, this );
			sdotv[i]->update_global_frame();
			return sdotv[i];
		}

		assert( false );

		return NULL;
	}


	void delete_dot(int dot_index)
	{
		assert( sdotv[dot_index] != NULL );
		delete sdotv[ dot_index ];
		sdotv[ dot_index ] = NULL;
	}


	virtual void draw()
	{
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;

			sdotv[i]->render_scene();
		}

	}


	size_t get_dotv_size() const
	{
		size_t siz = 0;
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] != NULL )
				siz++;
		}

		return siz;
	}

	//step_simulation - path that can use timer to solve state_curr and state_next
	virtual void step_simulation(float dt ) = 0;
};




class rigid_body: public dot_body_base
{

public:

	rigid_body(object_base* ref=NULL)
		:dot_body_base( vec3(0,0,0), ref )
	{

	}


	rigid_body(const vec3& frame_pos, object_base* ref=NULL)
		:dot_body_base( frame_pos, ref )
	{

	}


	virtual sdot_t* create_dot(size_t dot_index, const vec3& lpos)
	{
		/* rigid body dot create */
		sdot_t* s = dot_body_base::create_dot( dot_index, lpos );
		s->set_mass( 0.0 );

		return s;
	}


	virtual sdot_t* create_dot(const vec3& lpos )
	{
		/* rigid body dot create */
		sdot_t* s = dot_body_base::create_dot( lpos );
		s->set_mass( 0.0 );

		return s;
	}


	virtual void step_simulation(float dt )
	{
		/* if this object is referent to other object, it need to find its own position in global frame */
		update_global_frame();
		object_base::step_simulation( dt );

		/* now update all dot that inside 'this' */
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;
			/* update sdotv to a currect new global position */
			sdotv[i]->update_global_frame();
		}

		/* simulate each dot */
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;

			sdotv[i]->step_simulation( dt );
		}

	}

};



/*
 * @class spring_body is a class that consists of dot( movable point) and spring
 *                         this is the most basic class of deformable object
 */
class spring_body: public dot_body_base
{

protected:

	spring_t* create_spring(double Kspring, double Kdamper, double Lrest )
	{
		assert( Kspring >= 0.0 );
		assert( Kdamper <= 0.0 );
		assert( Lrest   >  0.0 );


		for(size_t i=0; i<springv.size(); i++)
		{
			if( springv[i] == NULL )
			{
				springv[i] = new spring_t( Kspring, Kdamper, Lrest );
				return springv[i];
			}
		}

		/* springv has reached maximum number, change 'MAX_SPING' */
		assert( false );

		return NULL;
	}


public:

	std::vector<sdot_t*> sdotv;
	std::vector<spring_t*> springv;

	spring_body(const vec3& frame_pos, object_base* ref=NULL)
		:dot_body_base( frame_pos, ref )
	{
		name = "spring_body";

		sdotv.resize( MAX_DOTx );
		std::fill( sdotv.begin(), sdotv.end(), (sdot_t*)NULL );
		springv.resize( MAX_SPING );
		std::fill( springv.begin(), springv.end(), (spring_t*)NULL );

		force_system_max = -10000.0;
		force_system_min =  10000.0;
	}


	size_t find_dot_myid(sdot_t* sdot)
	{
		auto it = std::find( sdotv.begin(), sdotv.end(), sdot );

		if( it == sdotv.end() )
			return -1;

		return it - sdotv.begin();
	}


	virtual~ spring_body()
	{
		for(size_t i=0; i<MAX_DOTx; i++)
		{
			if( sdotv[i] != NULL )
			{
				delete sdotv[i];
			}
		}

		for(size_t i=0; i<MAX_DOTx; i++)
		{
			if( springv[i] != NULL )
			{
				delete springv[i];
			}
		}

	}


	/* local position */
	void set_dot_pos(int index, float x, float y, float z)
	{
		assert( sdotv[index] != NULL );
		sdotv[index]->set_local_pos(x, y, z);
	}


	void set_dot_mass(size_t index, float mass)
	{
		assert( index < MAX_DOTx );
		assert( sdotv[index] != NULL );
		sdotv[index]->set_mass( mass );
	}


	void connect_dot(size_t dotA_index, size_t dotB_index, float Kspring, float Kdamper, float Lrest)
	{
		assert(dotA_index < MAX_DOTx);
		assert(dotB_index < MAX_DOTx);
		assert(dotA_index != dotB_index);


		sdot_t* dotA = sdotv[dotA_index];
		sdot_t* dotB = sdotv[dotB_index];


		connect_dot( dotA, dotB, Kspring, Kdamper, Lrest );
	}


	void connect_dot(sdot_t* dotA, sdot_t* dotB, float Kspring, float Kdamper, float Lrest)
	{
		assert( dotA != NULL );
		assert( dotB != NULL );

		spring_t* spring = create_spring( Kspring, Kdamper, Lrest );
		dotA->add_neighbor( dotB, spring );
		dotB->add_neighbor( dotA, spring );
		spring->set_connection( dotA, dotB );
	}


	sdot_t* split_between(int dotA_index, int dotB_index, double t )
	{
		sdot_t* dotA = sdotv[ dotA_index ];
		assert( dotA != NULL );

		sdot_t* dotB = sdotv[ dotB_index ];
		assert( dotB != NULL );

		return split_between( dotA, dotB, t );
	}


	sdot_t* split_between(sdot_t* dotA, sdot_t* dotB, double t )
	{
		assert( t>=0.0 && t<=1.0 );

		vec3 posA = dotA->get_pos();
		vec3 posB = dotB->get_pos();
		vec3 pos = t*( posB - posA ) + posA;
		sdot_t* dot_new = create_dot( pos );
		// original spring resize it
		// dotA --------- spring_original ------> dot_new

		spring_t* spring_old = dotA->get_spring_between( dotB );

		double Lrest= spring_old->Lrest;
		spring_old->Lrest = t * Lrest;


		spring_t* spring_new = create_spring( spring_old->Kspring, spring_old->Kdamper, spring_old->Lrest );
		spring_new->Lrest = ( 1.0-t ) * Lrest;


		if( dotA < dot_new )
		{
			spring_old->dotB = dot_new;
		}
		else
		{
			sdot_t* temp = dotA;
			spring_old->dotA = dot_new;
			spring_old->dotB = temp;
		}

		// dot_new -------- spring_new --------> dotB
		if( dotB > dot_new)
		{
			spring_new->dotA = dot_new;
			spring_new->dotB = dotB;
		}
		else
		{
			spring_new->dotA = dotB;
			spring_new->dotB = dot_new;
		}

		dotA->replace_neighbor(dotB, dot_new );
		dotB->remove_neighbor( dotA );
		dotB->add_neighbor( dot_new, spring_new );

		dot_new->add_neighbor( dotA, spring_old);
		dot_new->add_neighbor( dotB, spring_new);

		return dot_new;
	}


	void create_spring_between(sdot_t* dotA, sdot_t* dotB, double Kspring, double Kdamper)
	{
		vec3 a = dotA->get_pos();
		vec3 b = dotB->get_pos();
		double Lrest =  sqrt( distance_square(a,b) );

		connect_dot( dotA, dotB, Kspring, Kdamper, Lrest );
	}


	sdot_t* create_sdot_and_spring_between(sdot_t* dotA, sdot_t* dotB, double t, double Kspring, double Kdamper)
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( t>=0.0 && t<=1.0 );

		vec3 posA = dotA->get_pos();
		vec3 posB = dotB->get_pos();
		vec3 pos = t*( posB - posA ) + posA;


		sdot_t* dot_new = create_dot( pos );
		// original spring resize it
		// dotA --------- spring_original ------> dot_new


		double Lrest= sqrt( distance_square( posA, posB ) );
		spring_t* springA = create_spring( Kspring, Kdamper, t*Lrest);
		spring_t* springB = create_spring( Kspring, Kdamper, (1.0-t)*Lrest);


		//--- set parameter for springA ---//
		if( dotA < dot_new )
		{
			springA->dotA = dotA;
			springA->dotB = dot_new;
		}
		else
		{
			springA->dotA = dot_new;
			springA->dotB = dotA;
		}

		//--- set parameter for springB ---//
		if( dotB > dot_new)
		{
			springB->dotA = dot_new;
			springB->dotB = dotB;
		}
		else
		{
			springB->dotA = dotB;
			springB->dotB = dot_new;
		}


		//--- set neighbor for dotA  dotB dot_new ---//
		dotA->add_neighbor( dot_new, springA );
		dotB->add_neighbor( dot_new, springB );

		dot_new->add_neighbor( dotA, springA);
		dot_new->add_neighbor( dotB, springB);

		return dot_new;
	}


	void cut_between(int dotA_index, int dotB_index, double t, sdot_t* &dotA_new, sdot_t* &dotB_new  )
	{
		sdot_t* dotA = sdotv[dotA_index];
		assert( dotA != NULL );

		sdot_t* dotB = sdotv[dotB_index];
		assert( dotB != NULL );

		cut_between( dotA, dotB, t, dotA_new, dotB_new );
	}


	void cut_between(sdot_t* &dotA, sdot_t* &dotB, double t, sdot_t* &dotA_new, sdot_t* &dotB_new)
	{
		// crete new sdot
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( t>=0.0 && t<=1.0 );

		assert( dotA_new == NULL );
		assert( dotB_new == NULL );



		vec3 posA = dotA->get_pos();
		vec3 posB = dotB->get_pos();
		vec3 pos = t*( posB - posA ) + posA;




		dotA_new = create_dot( pos );
		dotB_new = create_dot( pos );

		// original spring resize it
		// dotA --------- spring_original ------> dot_new
		//dotA->debug_neighborv("dotA neighbor:");
		//printf("dot b %d\n", dotB->myid ); fflush( stdout );
		spring_t* spring_old = dotA->get_spring_between( dotB );
		double Lrest = spring_old->Lrest;
		spring_old->Lrest = t * Lrest;
		spring_t* spring_new = create_spring( spring_old->Kspring, spring_old->Kdamper, spring_old->Lrest );
		spring_new->Lrest = ( 1.0-t ) * Lrest;


		//--- old spring ---//
		if( dotA < dotA_new )
		{
			spring_old->dotB = dotA_new;
		}
		else
		{
			sdot_t* temp = dotA;
			spring_old->dotA = dotA_new;
			spring_old->dotB = temp;
		}



		// dot_new -------- spring_new --------> dotB
		if( dotB > dotB_new)
		{
			spring_new->dotA = dotB_new;
			spring_new->dotB = dotB;
		}
		else
		{
			spring_new->dotA = dotB;
			spring_new->dotB = dotB_new;
		}


		dotA->replace_neighbor(dotB, dotA_new );
		dotB->remove_neighbor( dotA );
		dotB->add_neighbor( dotB_new, spring_new );

		dotA_new->add_neighbor( dotA, spring_old);
		dotB_new->add_neighbor( dotB, spring_new);
	}


	void connect_dot(size_t dotA_index, size_t dotB_index, float Kspring, float Kdamper)
	{
		assert(dotA_index < MAX_DOTx);
		assert(dotB_index < MAX_DOTx);
		assert(dotA_index != dotB_index);


		assert( Kspring >= 0.0 ); /* should be positive */
		assert( Kdamper <= 0.0 ); /* should be nagative */
		sdot_t* dotA = sdotv[dotA_index];
		sdot_t* dotB = sdotv[dotB_index];

		assert( dotA != NULL );
		assert( dotB != NULL );

		//fixme: why do i need to minus on
		//double Lrest = sqrt( distance_square(dotA->get_pos(), dotB->get_pos()) - 1.0);
		double Lrest = sqrt( distance_square(dotA->get_pos(), dotB->get_pos()) );

		spring_t* spring = create_spring( Kspring, Kdamper, Lrest );
		dotA->add_neighbor( dotB, spring );
		dotB->add_neighbor( dotA, spring );
		spring->set_connection( dotA, dotB );
	}


	sdot_t* create_dot_rigidbody(size_t dot_index, const vec3& lpos)
	{
		assert( sdotv[dot_index] == NULL );
		sdotv[dot_index] = new sdot_t( lpos, dot_index, this );
		sdotv[dot_index]->set_mass( 0.0 );

		sdotv[dot_index]->update_global_frame();
		return sdotv[dot_index];
	}


	sdot_t* create_dot(size_t dot_index, const vec3& lpos)
	{
		assert( sdotv[dot_index] == NULL );
		sdotv[dot_index] = new sdot_t( lpos, dot_index, this );

		sdotv[dot_index]->update_global_frame();
		return sdotv[dot_index];
	}


	sdot_t* create_dot(const vec3& lpos, object_base* ref=NULL )
	{
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] != NULL)
				continue;

			if( ref == NULL )
				sdotv[i] = new sdot_t( lpos, i, this ); /* dot references to main body */
			else
				sdotv[i] = new sdot_t( lpos, i, ref ); /*  dot reference to other object */

			sdotv[i]->update_global_frame();
			return sdotv[i];
		}

		assert( false ); /* no space left for dot */

		return NULL;
	}


	void delete_dot(int dot_index)
	{
		assert( sdotv[dot_index] != NULL );
		delete sdotv[ dot_index ];
		sdotv[ dot_index ] = NULL;
	}


	void delete_spring(int spring_index)
	{
		assert( springv[spring_index] != NULL );
		delete springv[ spring_index ];
		springv[ spring_index ] = NULL;
	}


	virtual void draw()
	{
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;

			sdotv[i]->render_scene();
		}
	}


	void debug_sdotv()
	{
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] != NULL)
			{
				printf("[%ld]\n", i); fflush( stdout );
				sdotv[i]->debug_neighborv("\t-> ");
			}
		}

		fflush( stdout );
	}


	double force_system_max;
	double force_system_min;
	//step_simulation - path that can use timer to solve state_curr and state_next
	virtual void step_simulation(float dt )
	{
		/* simulate itself */
		update_global_frame();
		if( ref == NULL )
			object_base::step_simulation( dt );


		/* gravity force */
		vec3 force_ex(0.0, GRAVITY_Y*200.0 , 0.0 );
		vec3 force_system_sum( 0.0, 0.0, 0.0 );
		static vec3 spring_damper_force;

		/*---------------------------------*/
		/* update all dots to global frame */
		/*---------------------------------*/
		//may be does tneed to updat eto global frame
//		for(size_t i=0; i<sdotv.size(); i++)
//		{
//			if( sdotv[i] == NULL )
//				continue;
//
//			sdotv[i]->update_global_frame();
//		}

		/*-----------------------------------------*/
		/* add spring and damper force to each dot */
		/*-----------------------------------------*/
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;
			/* calculate internal spring force ( spring + damper ) */
			spring_damper_force = sdotv[i]->get_spring_damper_force();
			/* add internal force */
			sdotv[i]->set_force( spring_damper_force[0], spring_damper_force[1], spring_damper_force[2] );
			/* add external force (gravity + etc. ) */
			sdotv[i]->add_force( force_ex[0], force_ex[1], force_ex[2] );

			force_system_sum += spring_damper_force; /* debuging */

		}


		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;

			if( sdotv[i]->ref == this)
				sdotv[i]->step_simulation_lframe( dt ); /* update spring movement */
			else
				sdotv[i]->step_simulation( dt ); /*update with rigid transformation */
			/* if rigid body noneed to update local because their local is fixed */
			sdotv[i]->update_local_frame();
			sdotv[i]->set_force( 0.0f, 0.0f, 0.0f );
		}

	}


};



class suture_t: public spring_body
{

public:

	suture_t(size_t ndot, const vec3& pos, object_base* ref=NULL)
		:spring_body( pos, ref )
	{
		assert( ndot >= 2 );
		double y = pos[1];
		const double Kspring =  6000;
		const double Kdamper = -1000.0;
		const double Lrest   = 0.01;
		const double rigid_dist = 1.0;
		sdot_t* a;
		sdot_t* b;


		a = create_dot( vec3( pos[0], y-=Lrest, pos[2]) );
		b = create_dot( vec3( pos[0], y-=Lrest, pos[2]), a ); /* hard connect */
		for(size_t i=2; i<ndot; i+=2)
		{
			a = create_dot( vec3( pos[0], y-=Lrest, pos[2]) );
			b = create_dot( vec3( pos[0], y-=Lrest, pos[2]), a ); /* hard connect */
			connect_dot( i-1, i, Kspring, Kdamper );
		}

	}


	sdot_t* get_dot_at(size_t index)
	{
		assert( sdotv[index] != NULL );

		return sdotv[index];
	}

};



/*--------------------------------------------------------------------------------------------------------

	      ___.        __               __                    _____
	  ____\_ |__     |__| ____   _____/  |_  _______   _____/ ____\___________   ____   ____   ____  ____
	 /  _ \| __ \    |  |/ __ \_/ ___\   __\ \_  __ \_/ __ \   __\/ __ \_  __ \_/ __ \ /    \_/ ___\/ __ \
	(  <_> ) \_\ \   |  \  ___/\  \___|  |    |  | \/\  ___/|  | \  ___/|  | \/\  ___/|   |  \  \__\  ___/
	 \____/|___  /\__|  |\___  >\___  >__|    |__|    \___  >__|  \___  >__|    \___  >___|  /\___  >___  >
			   \/\______|    \/     \/                    \/          \/            \/     \/     \/    \/


----------------------------------------------------------------------------------------------------------*/


class object_ref
{

public:

	dot_t** dotv;
	size_t  ndot;

	object_ref(int ndot)
		:ndot( ndot )
	{
		dotv = new dot_t*[ndot];
	}


	virtual ~object_ref()
	{
		delete[] dotv;
	}


	void reref_dot(size_t dot_index, dot_t* dot_new )
	{
		assert( dot_index >= 0 && dot_index < ndot );
		assert( dotv[dot_index] != NULL );

		dotv[ dot_index ] = dot_new;
	}

};



/* this class for fill the structure of spring_body */
class triangle_ref: public triangle_base, public object_ref
{

public:

	triangle_ref(dot_t* dotA, dot_t* dotB, dot_t* dotC)
		:triangle_base( 0.0, vec3(0.0,1.0,0.0)), object_ref(3)
	{
		dotv[0] = dotA;
		dotv[1] = dotB;
		dotv[2] = dotC;

		vec3 pointA = dotA->get_pos();
		vec3 pointB = dotB->get_pos();
		vec3 pointC = dotC->get_pos();

		vec3 A = pointB - pointA;
		vec3 B = pointC - pointA;
		vec3 C = cross( A, B );

		normal = C.get_normalize();
		offset = dot( normal, pointA );
	}


	triangle_ref(dot_t* dotA, dot_t* dotB, dot_t* dotC, const vec3& normal)
		:triangle_base( 0.0, normal ), object_ref(3)
	{
		offset = dot( normal, dotA->get_pos() );
	}


	virtual vec3 A() const
	{
		return dotv[0]->get_pos();
	}


	virtual vec3 B() const
	{
		return dotv[1]->get_pos();
	}


	virtual vec3 C() const
	{
		return dotv[2]->get_pos();
	}


	virtual void add_force(double x, double y, double z)
	{
		object_base::add_force( x, y, z );
		dotv[0]->add_force( x, y, z);
		dotv[1]->add_force( x, y, z);
		dotv[2]->add_force( x, y, z);
	}


	void reref_A( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[0] = dot_new;
	}


	void reref_B( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[1] = dot_new;
	}


	void reref_C( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[2] = dot_new;
	}


};



class rectangle_ref: public rectangle_base, public object_ref
{

public:

	rectangle_ref(dot_t* dotA, dot_t* dotB, dot_t* dotC, dot_t* dotD)
		:rectangle_base( 0.0, vec3(0.0, 1.0, 0.0) ), object_ref(4)
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotC != NULL );
		assert( dotD != NULL );

		dotv[0] = dotA;
		dotv[1] = dotB;
		dotv[2] = dotC;
		dotv[3] = dotD;

		vec3 pointA = dotA->get_pos();
		vec3 pointB = dotB->get_pos();
		vec3 pointC = dotC->get_pos();

		vec3 A = pointB - pointA;
		vec3 B = pointC - pointA;
		vec3 C = cross( A, B );

		normal = C.get_normalize();
		offset = dot( normal, pointA );
	}


	vec3 A() const
	{
		return dotv[0]->get_pos();
	}


	vec3 B() const
	{
		return dotv[1]->get_pos();
	}


	vec3 C() const
	{
		return dotv[2]->get_pos();
	}


	vec3 D() const
	{
		return dotv[3]->get_pos();
	}


	virtual void add_force(double x, double y, double z)
	{
		object_base::add_force( x, y, z );

		dotv[0]->add_force( x, y, z);
		dotv[1]->add_force( x, y, z);
		dotv[2]->add_force( x, y, z);
		dotv[3]->add_force( x, y, z);
	}


	void reref_A( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[0] = dot_new;
	}


	void reref_B( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[1] = dot_new;
	}

	void reref_C( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[2] = dot_new;
	}


	void reref_D( dot_t* dot_new )
	{
		assert( dot_new != NULL );
		dotv[3] = dot_new;
	}

};





/*

			_            _          _               _        _
		  /\ \         /\ \       /\ \            /\ \     /\ \
		 /  \ \        \ \ \     /  \ \           \ \ \   /  \ \____
		/ /\ \ \       /\ \_\   / /\ \_\          /\ \_\ / /\ \_____\
	   / / /\ \_\     / /\/_/  / / /\/_/         / /\/_// / /\/___  /
	  / / /_/ / /    / / /    / / / ______      / / /  / / /   / / /
	 / / /__\/ /    / / /    / / / /\_____\    / / /  / / /   / / /
    / / /_____/    / / /    / / /  \/____ /   / / /  / / /   / / /
   / / /\ \ \  ___/ / /__  / / /_____/ / /___/ / /__ \ \ \__/ / /
  / / /  \ \ \/\__\/_/___\/ / /______\/ //\__\/_/___\ \ \___\/ /
  \/_/    \_\/\/_________/\/___________/ \/_________/  \/_____/


*/


class rigid_triangle: public rigid_body
{

protected:
	std::vector<triangle_ref*> facev;

public:

	rigid_triangle(object_base* ref=NULL)
		:rigid_body(ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (triangle_ref*)NULL );
	}


	rigid_triangle(const vec3& frame_pos, object_base* ref=NULL)
		:rigid_body(frame_pos, ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (triangle_ref*)NULL );
	}


	triangle_ref* make_face(int indexA, int indexB, int indexC)
	{
		sdot_t* dotA = sdotv[indexA];
		sdot_t* dotB = sdotv[indexB];
		sdot_t* dotC = sdotv[indexC];

		return make_face( dotA, dotB, dotC );
	}


	triangle_ref* make_face(dot_t* dotA, dot_t* dotB, dot_t* dotC )
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotC != NULL );

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL )
				continue;

			facev[i] = new triangle_ref( dotA, dotB, dotC );

			return facev[i];
		}

		assert( false );/* face is full */

		return NULL;
	}


	triangle_ref* make_face(dot_t* dotA, dot_t* dotB, dot_t* dotC, const vec3& normal)
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotC != NULL );

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL )
				continue;

			facev[i] = new triangle_ref( dotA, dotB, dotC, normal );

			return facev[i];
		}

		assert( false );/* face is full */

		return NULL;
	}


	size_t get_facev_size()
	{
		size_t siz = 0;
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL)
				siz++;
		}

		return siz;
	}


	virtual void draw()
	{
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] == NULL )
				continue;

			facev[i]->render_scene();
		}


		/*****************************
		 *   draw dot for debugging  *
		 ****************************/
//		for(size_t i=0; i<sdotv.size(); i++)
//		{
//
//			if( sdotv[i] == NULL )
//				continue;
//
//			sdotv[i]->render_scene();
//		}

	}


};



class rigid_rectangle: public rigid_body
{

protected:
	std::vector<rectangle_ref*> facev;

public:

	rigid_rectangle(object_base* ref=NULL)
		:rigid_body(ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (rectangle_ref*)NULL );
	}


	rectangle_ref* make_face(int indexA, int indexB, int indexC, int indexD)
	{
		sdot_t* dotA = sdotv[indexA];
		sdot_t* dotB = sdotv[indexB];
		sdot_t* dotC = sdotv[indexC];
		sdot_t* dotD = sdotv[indexD];

		return make_face( dotA, dotB, dotC, dotD );
	}


	rectangle_ref* make_face(dot_t* dotA, dot_t* dotB, dot_t* dotC, dot_t* dotD )
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotC != NULL );

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL )
				continue;

			facev[i] = new rectangle_ref( dotA, dotB, dotC, dotD );

			return facev[i];
		}

		assert( false );//face is full

		return NULL;
	}


	size_t get_facev_size()
	{
		size_t siz = 0;
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL)
				siz++;
		}

		return siz;
	}


	virtual void draw()
	{

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] == NULL )
				continue;

			facev[i]->render_scene();
		}

		/*---------------------------------
		 |           draw dot              |
		 ---------------------------------*/
		for(size_t i=0; i<sdotv.size(); i++)
		{

			if( sdotv[i] == NULL )
				continue;

			sdotv[i]->render_scene();
		}

	}


	virtual GLint* load_texture(const std::string filename)
	{
		/* now put texture to memory and store at this->texture */
		texture = object_base::load_texture( filename );

		for(auto iter=facev.begin(); iter!=facev.end(); ++iter)
		{
			if( *iter == NULL )
				continue;
			//(*iter)->texture = texture; //TODO//
			(*iter)->set_texture( texture );
			(*iter)->set_UV( 0, 0.0f, 0.0f );
			(*iter)->set_UV( 1, 0.0f, 1.0f );
			(*iter)->set_UV( 2, 1.0f, 1.0f );
			(*iter)->set_UV( 3, 1.0f, 0.0f );
		}

		return texture;
	}


};



/*------------------------------------------------------------------------
	                __                                  _
	               / _|                                (_)
     ___ _   _ _ __| |_ __ _  ___ ___    ___ _ __  _ __ _ _ __   __ _
	/ __| | | | '__|  _/ _` |/ __/ _ \  / __| '_ \| '__| | '_ \ / _` |
	\__ \ |_| | |  | || (_| | (_|  __/  \__ \ |_) | |  | | | | | (_| |
	|___/\__,_|_|  |_| \__,_|\___\___|  |___/ .__/|_|  |_|_| |_|\__, |
						                    | |                  __/ |
						                    |_|                 |___/

-------------------------------------------------------------------------*/


class surface_triangle_spring: public spring_body
{

public:

	std::vector<triangle_ref*> facev;
	surface_triangle_spring(object_base* ref=NULL)
		:spring_body( vec3(0,0,0), ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (triangle_ref*)NULL );
	}


	surface_triangle_spring(const vec3& frame_pos, object_base* ref=NULL)
		:spring_body( frame_pos, ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (triangle_ref*)NULL );
	}


	triangle_ref* make_face(int indexA, int indexB, int indexC)
	{
		sdot_t* dotA = sdotv[indexA];
		sdot_t* dotB = sdotv[indexB];
		sdot_t* dotC = sdotv[indexC];

		return make_face( dotA, dotB, dotC );
	}


	triangle_ref* make_face(dot_t* dotA, dot_t* dotB, dot_t* dotC )
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotC != NULL );

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL )
				continue;

			facev[i] = new triangle_ref( dotA, dotB, dotC );

			return facev[i];
		}

		assert( false );//face is full

		return NULL;
	}


	size_t get_facev_size()
	{
		size_t siz = 0;
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL)
				siz++;
		}

		return siz;
	}


	virtual void draw()
	{
		/*--------------------------*/
		/*  draw dot for debugging  */
		/*--------------------------*/
//		for(size_t i=0; i<sdotv.size(); i++)
//		{
//
//			if( sdotv[i] == NULL )
//				continue;
//
//			sdotv[i]->render_scene();
//		}

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] == NULL )
				continue;

			facev[i]->render_scene();
		}

	}


};







class surface_rectangle_spring: public spring_body
{


public:

	std::vector<rectangle_ref*> facev;
	surface_rectangle_spring(object_base* ref=NULL)
		:spring_body(vec3(0,0,0), ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (rectangle_ref*)NULL );
	}


	surface_rectangle_spring(const vec3& frame_ref, object_base* ref=NULL)
		:spring_body( frame_ref, ref)
	{
		facev.resize( MAX_FACE );
		std::fill( facev.begin(), facev.end(), (rectangle_ref*)NULL );
	}


	rectangle_ref* make_face(int indexA, int indexB, int indexC, int indexD)
	{
		sdot_t* dotA = sdotv[indexA];
		sdot_t* dotB = sdotv[indexB];
		sdot_t* dotC = sdotv[indexC];
		sdot_t* dotD = sdotv[indexD];

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL )
				continue;

			facev[i] = new rectangle_ref( dotA, dotB, dotC, dotD );
			facev[i]->myid = i;
			return facev[i];
		}
		assert( false );

		return NULL;
	}


	rectangle_ref* make_face(sdot_t* dotA, sdot_t* dotB, sdot_t* dotC, sdot_t* dotD)
	{
		assert( dotA != NULL );
		assert( dotB != NULL );
		assert( dotC != NULL );
		assert( dotD != NULL );

		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != NULL )
				continue;

			facev[i] = new rectangle_ref( dotA, dotB, dotC, dotD );
			facev[i]->myid = i;
			return facev[i];
		}
		assert( false );

		return NULL;
	}


	void remove_face(const rectangle_ref* face)
	{
		auto it = std::find( facev.begin(), facev.end(), face);
		assert( it != facev.end() );

		facev.erase( it );
	}


	size_t get_facev_size()
	{
		size_t siz = 0;
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] != 0)
				siz++;
		}

		return siz;
	}


	virtual void draw()
	{

		auto face_it = facev.begin();
		auto face_end = facev.end();
		for(; face_it != face_end ; ++face_it) //delay a lot
		{
			if( *face_it == NULL )
				continue;

			(*face_it)->render_scene();
		}


		//---------------------------//
		//       draw dot         //
		//---------------------------//
//		for(size_t i=0; i<sdotv.size(); i++)
//		{
//			if( sdotv[i] == NULL )
//				continue;
//
//			sdotv[i]->render_scene();
//		}

		//---------------------------//
		//       draw spring         //
		//---------------------------//
//		glLineWidth( 0.5 );
//		glColor3f( 0.0f, 0.0f, 0.0f );
//		for(size_t i=0; i<springv.size(); i++)
//		{
//			if( springv[i] == NULL )
//				continue;
//
//			springv[i]->render_scene();
//		}


	}


	virtual void step_simulation(float dt )
	{
		spring_body::step_simulation( dt );
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] == NULL )
				continue;

			facev[i]->update_normal();
		}
	}


	/* A and A_dat are absolute position in the world */
	void cut_face( int index, const vec3& A, const vec3& A_dat)
	{
		rectangle_ref* face = facev[index];
		std::cerr << "warning: assert face != NULL " << face << std::endl;
		std::cerr << "face index is: " << index << std::endl;
		assert( face != NULL );

		vec3 lineA = A_dat - A;
		assert( lineA.mag() > EPSILON );

		stoke( 20, 0.2, 0.0, 0.0 );
		draw_line( A_dat, A );




		vec3 A_normal = cross(lineA, face->normal).get_normalize();
		double A_offset = dot( A_normal, A );

		sdot_t* dotA = (sdot_t*)face->dotv[0];
		sdot_t* dotB = (sdot_t*)face->dotv[1];
		sdot_t* dotC = (sdot_t*)face->dotv[2];
		sdot_t* dotD = (sdot_t*)face->dotv[3];


		vec3 a = dotA->get_pos().debug("a");
		vec3 b = dotB->get_pos().debug("b");
		vec3 c = dotC->get_pos().debug("c");
		vec3 d = dotD->get_pos().debug("d");






		double t_ab = 0.0;
		double t_bc = 0.0;
		double t_cd = 0.0;
		double t_da = 0.0;


		sdot_t* dotABa = NULL;
		sdot_t* dotABb = NULL;
		sdot_t* dotBCb = NULL;
		sdot_t* dotBCc = NULL;
		sdot_t* dotCDc = NULL;
		sdot_t* dotCDd = NULL;
		sdot_t* dotDAd = NULL;
		sdot_t* dotDAa = NULL;


		stoke( 5, 0.0, 0.0, 0.0 );

		vec3 edge_ab = b - a;
		t_ab = ( A_offset - (dot( A_normal, a ))) / dot(A_normal, edge_ab);
		if( t_ab >= 0.0f && t_ab < 1.0f )
		{
			cut_between( dotA, dotB, t_ab, dotABa, dotABb );
			printf("cut between AB\n"); fflush( stdout );
		}


		vec3 edge_bc = c - b;
		t_bc = ( A_offset - (dot( A_normal, b ))) / dot(A_normal, edge_bc);
		if( t_bc >= 0.0f && t_bc < 1.0f )
		{
			cut_between( dotB, dotC, t_bc, dotBCb, dotBCc );
			printf("cut between BC\n"); fflush( stdout );
		}

		vec3 edge_cd = d - c;
		t_cd = ( A_offset - (dot( A_normal, c ))) / dot(A_normal, edge_cd);
		if( t_cd >= 0.0f && t_cd < 1.0f )
		{
			cut_between( dotC, dotD, t_cd, dotCDc, dotCDd );
			printf("cut between CD\n"); fflush( stdout );
		}

		vec3 edge_da = a - d;
		t_da = ( A_offset - (dot( A_normal, d ))) / dot(A_normal, edge_da);
		if( t_da >= 0.0f && t_da < 1.0f )
		{
			cut_between( dotD, dotA, t_da, dotDAd, dotDAa );
			printf("cut between AD\n"); fflush( stdout );
		}



		printf( "ab  %f\n", t_ab );
		printf( "bc  %f\n", t_bc );
		printf( "cd  %f\n", t_cd );
		printf( "da  %f\n", t_da );





		/*
		 *
		 *
		 *     a ------------------ d
		 *     |                    |
		 *     |                    |
		 *     b ------------------ c
		 *
		 *
		 *       dotA ---------dotDAa dotDAd------- dotD
		 *         |                                |
		 *      dotABa                            dotCDd
		 *      dotABb                            dotCDc
		 *         |                                |
		 *       dotB ---------dotBCb dotBCc ------- dotC
		 *
		 *
		 */

		/* parameter for new spring(s) the will be created */
		double Kspring = 2000.0;
		double Kdamper = -100.0;

		if( dotABa != NULL )
		{
			dotABa->debug_pos("Dot AB");

			if( dotBCb != NULL )
			{
				dotBCb->debug_pos("Dot BC");

				//cut AB  BC
				printf("cut AB BC\n"); fflush( stdout );
				sdot_t* cd_dat = split_between( dotC, dotD, 0.5 );
				//--- face1 ---//
				auto face1 = make_face( dotABa, dotBCc, dotC, cd_dat);
				face1->set_color( 0.5, 0.0, 0.0 );

				//--- original face0 ---//
				face->reref_dot( 1, dotABa ); /* reref dotB to dotABa */
				face->reref_dot( 2, cd_dat ); /* reref dotC to cd_dat */
				create_spring_between( dotABa, dotBCc, Kspring, Kdamper ); //cutting line
				create_spring_between( dotABa, cd_dat, Kspring, Kdamper ); //folding line

				//--- split face ---//
				auto dot_new  = create_sdot_and_spring_between( dotABb, dotBCb, 0.5, Kspring, Kdamper );
				auto face_new = make_face( dotB, dotBCb, dot_new, dotABb );
				face_new->set_color( 0.0, 0.5, 0.0 );

			}
			else if( dotCDc != NULL ) //cut half
			{
				dotCDc->debug_pos("Dot CD");

				//cut AB CD
				printf("cut AB CD\n"); fflush( stdout );
				auto face_new = make_face( dotB, dotC, dotCDc, dotABb );
				face_new->set_color( 0.0, 0.5, 0.0 );

				//--- original face0 ---//
				face->reref_dot( 1, dotABa ); /* reref dotB to dotABa */
				face->reref_dot( 2, dotCDd ); /* reref dotC to dotCDd */
			}
			else if( dotDAd != NULL )
			{
				dotDAd->debug_pos("Dot DA");
				//cut AB DA
				printf("cut AB DA\n"); fflush( stdout );
				sdot_t* cd_dat = split_between( dotC, dotD, 0.5 );

				//--- face1 ---//
				auto face1 = make_face( dotABb, cd_dat, dotD, dotDAd);
				face1->set_color( 0.5, 0.0, 0.0 );

				//--- original face0 ---//
				face->reref_dot( 0, dotABb );
				face->reref_dot( 3, cd_dat );
				create_spring_between( dotABb, dotDAd, Kspring, Kdamper ); //cutting line
				create_spring_between( dotABb, cd_dat, Kspring, Kdamper ); //folding line


				//--- split face ---//
				auto dot_new  = create_sdot_and_spring_between( dotABa, dotDAa, 0.5, Kspring, Kdamper );
				auto face_new = make_face( dotA, dotABa, dot_new, dotDAa );
				face_new->set_color( 0.0, 0.5, 0.0 );
			}
			else
			{
				std::cerr << "waring doesnt cut anything" << "- line 4343" <<std::endl;
				//assert( false );
			}

		}
		else if( dotBCb != NULL )
		{
			dotBCb->debug_pos("Dot BC");
			if( dotCDc != NULL )
			{
				dotCDc->debug_pos("Dot CD");
				// cut BC CD
				printf("cut BC CD\n"); fflush( stdout );
				sdot_t* da_dat = split_between( dotD, dotA, 0.5 );
				//--- face1 ---//
				auto face1 = make_face( dotBCb, dotCDd, dotD, da_dat);
				face1->set_color( 0.5, 0.0, 0.0 );

				//--- original face0 ---//
				face->reref_dot( 2, dotBCb );
				face->reref_dot( 3, da_dat );
				create_spring_between( dotBCb, dotCDd, Kspring, Kdamper ); //cutting line
				create_spring_between( dotBCb, da_dat, Kspring, Kdamper ); //folding line

				//--- split face ---//
				auto dot_new  = create_sdot_and_spring_between( dotBCc, dotCDc, 0.5, Kspring, Kdamper );
				auto face_new = make_face( dotC, dotCDc, dot_new, dotBCc );
				face_new->set_color( 0.0, 0.5, 0.0 );

			}
			else if( dotDAd != NULL ) //cut half /* focus here */
			{
				dotDAd->debug_pos("Dot DAd");
				dotDAa->debug_pos("Dot DAa");

				// cut BC DA
				printf("cut BC DA\n"); fflush( stdout );
				auto face_new = make_face( dotC, dotD, dotDAd, dotBCc );
				face_new->set_color( 0.0, 0.5, 0.0 );

				//--- original face0 ---//
				face->reref_dot( 2, dotBCb );
				face->reref_dot( 3, dotDAa );
			}
			else
			{
				std::cerr << "waring doesnt cut anything" << "- line 4388" <<std::endl;
				//assert( false );
			}
		}
		else if( dotCDc != NULL )
		{
			dotCDc->debug_pos("Dot CD");
			if( dotDAd != NULL ) //cut corner
			{
				dotDAd->debug_pos("Dot DA");
				/* cut CD DA */
				printf("cut CD DA\n"); fflush( stdout );
				sdot_t* bc_dat = split_between( dotB, dotC, 0.5 ); //folding point
				/*--- face1 ---*/
				auto face1 = make_face( dotC, dotCDc, dotDAa, bc_dat );
				face1->set_color( 0.5, 0.0, 0.0 );

				/*--- original face0 ---*/
				face->reref_dot( 2, bc_dat );
				face->reref_dot( 3, dotDAa );
				create_spring_between( dotDAa, dotCDc, Kspring, Kdamper ); //cutting line
				create_spring_between( bc_dat, dotDAa, Kspring, Kdamper ); //folding line

				/*--- split face ---*/
				auto dot_new  = create_sdot_and_spring_between( dotCDd, dotDAd, 0.5, Kspring, Kdamper );
				auto face_new = make_face( dotD, dotDAd, dot_new, dotCDd );
				face_new->set_color( 0.0, 0.5, 0.0 );
			}
			else
			{
				std::cerr << "waring doesnt cut anything" << "- line 4417" <<std::endl;
				//assert( false );
			}
		}
		else
		{
			std::cerr << "waring doesnt cut anything" << std::endl;
			//assert( false ); /* doesnt cut anything.. */
		}


	}//------------










	virtual GLint* load_texture(const std::string filename)
	{
		/* now put texture to memory and store at this->texture */
		texture = object_base::load_texture( filename );
		assert( texture != NULL );

		for(auto iter=facev.begin(); iter!=facev.end(); ++iter)
		{
			if( *iter == NULL )
				continue;

			(*iter)->texture = texture;
			(*iter)->set_UV( 0, 0.0f, 0.0f );
			(*iter)->set_UV( 1, 0.0f, 1.0f );
			(*iter)->set_UV( 2, 1.0f, 1.0f );
			(*iter)->set_UV( 3, 1.0f, 0.0f );

		}

		return texture;
	}


};





const double cloth_scale = 5.0;

class cloth_rectangle: public surface_rectangle_spring
{

public:

	size_t nheight;
	size_t nwidth;



	/* width and height are dot_number / not face_number */
	cloth_rectangle(const vec3& frame_pos, size_t nwidth, size_t nheight, object_base* ref=NULL)
		:surface_rectangle_spring(frame_pos, ref)
	{
		this->nwidth   = nwidth;
		this->nheight  = nheight;
		size_t ndot  = nwidth * nheight;

		assert( ndot <= MAX_DOTx );
		size_t id = 0;

		assert( nwidth >= 2 );
		assert( nheight >= 2 );

		for(size_t i=0; i<nheight; i++)
		{
			for(size_t j=0; j<nwidth; j++)
			{
				int x = j  + 1.0;
				int z = i  + 1.0;

				sdot_t* s = create_dot(id++, vec3(x*cloth_scale, 0.0, z*cloth_scale) );
				if( i % (nheight-1) == 0 || j % (nwidth-1)==0 )
				//if( (i == 0) && (j == nwidth-1 || j == 0 )  )
				{
					assert( s != NULL );
					s->set_mass( 0.0 );
				}
			}// for j
		}// for i


		double Kspring =  100.0; //500 is good
		double Kdamper = -2.0; //damper -1.0 is good

		double Lrest   =  1*cloth_scale; //very tight
		//double Lrest   =  1.0*cloth_scale;
		for(size_t i=0; i<nheight; i++)
		{
			for(size_t j=0; j<nwidth; j++)
			{
				size_t id = i*nwidth + j;

				if( j != nwidth-1 )
					connect_dot( id, id+1     , Kspring, Kdamper, Lrest );

				if( i != nheight-1)
					connect_dot( id, id+nwidth, Kspring, Kdamper, Lrest );

			}
		}

		connect_dot( (ndot-1)-nwidth, ndot-1, Kspring, Kdamper, Lrest );
		connect_dot(  ndot-2        , ndot-1, Kspring, Kdamper, Lrest );


		for(size_t i=0; i<nheight-1; i++)
		{
			for(size_t j=0; j<nwidth-1; j++)
			{
				int id = i*nwidth + j;
				make_face( id, id+nwidth, (id)+nwidth+1, id+1 );
			}
		}


		/*--------------------------*/
		/* create structural spring */
		/*--------------------------*/

		for(auto iter=facev.begin(); iter<facev.end(); ++iter)
		{
			if( (*iter) == NULL )
				continue;

			/* - frame_pos make mid to local again */
			vec3 mid = ( (*iter)->A() + (*iter)->B() + (*iter)->C() + (*iter)->D() )/4 - frame_pos;
			double siz = 2;
			mid = mid + (*iter)->normal*siz;
			sdot_t* s = create_dot( id++, mid );
			s->set_mass( 0.0 );
			s->set_color( 1.0, 0.0, 0.0 );

			double Kspring = 20;
			double Kdamper = -1;
			double Lrest   = 0.1;
			this->create_spring_between( s, dynamic_cast<sdot_t*>( (*iter)->dotv[0] ), Kspring, Kdamper );
		}

	}



	virtual void step_simulation(float dt )
	{
		/* simulate itself */
//		update_global_frame();
//		if( ref == NULL )
//			object_base::step_simulation( dt );

		/* gravity force */
		vec3 force_ex( 0.0, GRAVITY_Y*10 , 0.0 );
		vec3 force_system_sum( 0.0, 0.0, 0.0 );
		static vec3 spring_damper_force;

//		/*---------------------------------*/
//		/* update all dots to global frame */
//		/*---------------------------------*/
//		//may be does tneed to updat eto global frame
//		for(size_t i=0; i<sdotv.size(); i++)
//		{
//			if( sdotv[i] == NULL )
//				continue;
//
//			sdotv[i]->update_global_frame();
//		}

		/*-----------------------------------------*/
		/* add spring and damper force to each dot */
		/*-----------------------------------------*/
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;
			/* calculate internal spring force ( spring + damper ) */
			spring_damper_force = sdotv[i]->get_spring_damper_force();
			/* add internal force */
			/* todo just change */
			sdotv[i]->add_force( spring_damper_force[0], spring_damper_force[1], spring_damper_force[2] );
			/* add external force (gravity + etc. ) */
			sdotv[i]->add_force( force_ex[0], force_ex[1], force_ex[2] );

			force_system_sum += spring_damper_force; /* debuging */
		}


		/*-------------------------*/
		/*      simulate dot       */
		/*-------------------------*/
		for(size_t i=0; i<sdotv.size(); i++)
		{
			if( sdotv[i] == NULL )
				continue;
			sdotv[i]->step_simulation_lframe( dt );
			/* if rigid body noneed to update local because their local is fixed */
			sdotv[i]->update_local_frame();
			sdotv[i]->set_force( 0.0f, 0.0f, 0.0f );
		}



		/*-------------------------*/
		/*      update face        */
		/*-------------------------*/
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] == NULL )
				continue;

			facev[i]->update_normal();
		}

	}


	virtual GLint* load_texture(const std::string filename)
	{
		/* now put texture to memory and store at this->texture */
		texture = object_base::load_texture( filename );
		assert( texture != NULL );

		for(auto iter=facev.begin(); iter!=facev.end(); ++iter)
		{
			if( *iter == NULL )
				continue;

			int face_id = (*iter)->myid;

			int i = face_id / (nwidth-1);
			int j = face_id % (nwidth-1);

			float dist_i = 1.0f / (nwidth-1);
			float dist_j = 1.0f / (nheight-1);

			(*iter)->texture = texture;
			(*iter)->set_UV( 0, i    *dist_i,     j*dist_j );
			(*iter)->set_UV( 1, (i+1)*dist_i,     j*dist_j );
			(*iter)->set_UV( 2, (i+1)*dist_i, (j+1)*dist_j );
			(*iter)->set_UV( 3, i    *dist_i, (j+1)*dist_j );
		}

		return texture;
	}



	virtual bool get_active_collision(std::vector<primitive_base*>& objv )
	{
		//objv.insert( objv.end(), activev.begin(), activev.end() );
		return true;
	}


	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		objv.insert( objv.end(), facev.begin(), facev.end() );
		return true;
	}



	void cut(size_t index)
	{
		for(auto iter=facev.begin(); iter<facev.end(); ++iter)
		{
			rectangle_ref* rec = (rectangle_ref*)(*iter);

		}
	}
};





class cloth_triangle: public surface_triangle_spring
{

public:

	size_t nheight;
	size_t nwidth;
	//width and height are dot_number / not face_number
	cloth_triangle(const vec3& frame_pos, size_t nwidth, size_t nheight, double reso=1.0, object_base* ref=NULL)
		:surface_triangle_spring(frame_pos,ref)
	{
		this->nwidth   = nwidth;
		this->nheight  = nheight;
		size_t ndot  = nwidth * nheight;

		assert( ndot <= MAX_DOTx );
		size_t id = 0;

		assert( nwidth  >= 2 );
		assert( nheight >= 2 );

		for(size_t i=0; i<nheight; i++)
		{
			for(size_t j=0; j<nwidth; j++)
			{
				int x = j + 1.0;
				int z = i + 1.0;

				create_dot(id++, vec3(x*cloth_scale, 0.0, z*cloth_scale) );
				if( i % (nheight-1) == 0 || j % (nwidth-1)==0) // % 0 is undefined
				{
					assert(sdotv[id-1] != NULL );
					sdotv[id-1]->set_mass( 0.0 );
				}
			}// for j
		}// for i

		double Kspring = 500.0; //500 is good
		double Kdamper = -10.0; //damper -1.0 is good
		double Lrest   =  0.1;
		for(size_t i=0; i<nheight; i++)
		{
			for(size_t j=0; j<nwidth; j++)
			{
				int id = i*nwidth + j;

				if( j != nwidth-1 )
					connect_dot( id, id+1     , Kspring, Kdamper, Lrest );

				if( i != nheight-1)
					connect_dot( id, id+nwidth, Kspring, Kdamper, Lrest );
			}
		}

		connect_dot( (ndot-1)-nwidth, ndot-1, Kspring, Kdamper, Lrest );
		connect_dot(  ndot-2        , ndot-1, Kspring, Kdamper, Lrest );

		for(size_t i=0; i<nheight-1; i++)
		{
			for(size_t j=0; j<nwidth-1; j++)
			{
				int id = i*nwidth + j;
				make_face( id, id+nwidth     , (id)+nwidth+1 );
				make_face( id, (id)+nwidth+1 , id+1 );
			}
		}

	}


};


//todo make it is larger
//todo timestamp

static double t = -6000;
int wave_collide_x = 30;
int wave_collide_z = 30;


//static double z = -0.16; //FIXME assume user input
//static double z = -10; //FIXME assume user input
class wave_surface: public cloth_rectangle
{

	double b;

public:

	wave_surface()
		:cloth_rectangle( vec3(0,0,0), 100, 100 )
	{
		b = 0.0;
	}


	wave_surface(const vec3& frame_pos)
		:cloth_rectangle( frame_pos, 100, 100 )
	{
		b = 0.0;
	}




	virtual void step_simulation(float dt )
	{

		draw_string( std::string("wave_collide_x: ") + to_string(wave_collide_x), 10, 45, 0 );
		draw_string( std::string("wave_collide_z: ") + to_string(wave_collide_z), 10, 30, 0 );
		draw_string( std::string("sneha depth: ") + to_string(sneha_depth), 10, 25, 0 );


		double amp = 1.0;


		//sneha.. modify here !
		for(size_t i=0; i<nheight; i++)
		{
			for(size_t j=0; j<nwidth; j++)
			{

				/* limit ouyside boudary to 0 */
				sdot_t* sdot = sdotv[i*nwidth + j];


//				if(  (i  > wave_collide_z + 30) || (i  < wave_collide_z - 30) )
//				{
//					sdot->state_of(Y_POS) = 0;
//					continue;
//				}
//				if(  (j  > wave_collide_x + 30) || (j  < wave_collide_x - 30) )
//				{
//					sdot->state_of(Y_POS) = 0;
//					continue;
//				}

				t = sneha_depth;

				int shift_i = i-wave_collide_z - 30;
				int shift_j = j-wave_collide_x - 30;

				double vert =  amp*((1.5527*pow(10.0,2))*cos(6.849*pow(10.0,-4)*t))*sin(M_PI*shift_i/60)*sin(M_PI*shift_j/60);


				sdot->state_of(Y_POS) = vert < 0 ? vert : 0;
				//sdot->state_of(Y_POS) = vert;

				assert( !isnan( sdot->state_of(Y_POS) ) );
			} //for j
		}// for i


		//z -= 10.0; //FIXME this is just assume that the user penaltrates 0.01 every loop



		/*-------------------------*/
		/*      update face        */
		/*-------------------------*/
		for(size_t i=0; i<facev.size(); i++)
		{
			if( facev[i] == NULL )
				continue;

			facev[i]->update_normal();
		}
	}

};





/*


	███████╗████████╗██╗          ██████╗ ██████╗      ██╗███████╗ ██████╗████████╗
	██╔════╝╚══██╔══╝██║         ██╔═══██╗██╔══██╗     ██║██╔════╝██╔════╝╚══██╔══╝
	███████╗   ██║   ██║         ██║   ██║██████╔╝     ██║█████╗  ██║        ██║
	╚════██║   ██║   ██║         ██║   ██║██╔══██╗██   ██║██╔══╝  ██║        ██║
	███████║   ██║   ███████╗    ╚██████╔╝██████╔╝╚█████╔╝███████╗╚██████╗   ██║
	╚══════╝   ╚═╝   ╚══════╝     ╚═════╝ ╚═════╝  ╚════╝ ╚══════╝ ╚═════╝   ╚═╝


 */




class stl_rigid_triangle: public rigid_triangle
{

protected:


//	// todo just add to show for vietname
//	// todo  remove this function
//	virtual void draw()
//	{
		//want to draw line for collision
//		for(auto iter=activev.begin(); iter<activev.end(); ++iter)
//		{
//			(*iter)->render_scene();
//		}
//
//		rigid_triangle::draw();
//	}


	int stl_to_memory(const char *stlfile, bool is_binary_format)
	{
		size_t idot = 0;
		size_t iface = 0;
		if (!is_binary_format) //ASCII format
		{
			assert( false ); //i haven't implemented binary format
			std::ifstream in(stlfile);
			if (!in.good())
				return 1;

			char title[80];
			std::string s0, s1;
			float n0, n1, n2, f0, f1, f2, f3, f4, f5, f6, f7, f8;
			in.read(title, 80);
			while( !in.eof() )
			{
				in >> s0;                               // facet || endsolid
				if (s0 == "facet")
				{
					in >> s1 >> n0 >> n1 >> n2;            // normal x y z
					in >> s0 >> s1;                        // outer loop
					in >> s0 >> f0 >> f1 >> f2;         // vertex x y z
					in >> s0 >> f3 >> f4 >> f5;         // vertex x y z
					in >> s0 >> f6 >> f7 >> f8;         // vertex x y z
					in >> s0;                            // endloop
					in >> s0;                            // endfacet
					// Generate a new Triangle with Normal as 3 Vertices
				//	triangle_t t(v3(n0, n1, n2), v3(f0, f1, f2),
				//			v3(f3, f4, f5), v3(f6, f7, f8));
					//mesh->push_back(t);

					vec3 A( f0, f1, f2);
					vec3 B( f3, f4, f5);
					vec3 C( f6, f7, f8);
					vec3 normal(n0, n1, n2);
					//planev.push_back( new plane(A, B, C, normal) ); //TODO: fix this - this is hvnt fix for  long time ..mostly update

				}
				else if (s0 == "endsolid")
				{
					break;
				}
			}
			in.close();
		}
		else //BINARY format
		{
			FILE *f = fopen(stlfile, "rb");
			if (!f)
			{
				fprintf( stderr, "cannot open %s\n", stlfile );
				exit(-1);
			}

			char title[80];
			size_t nFaces;
			fread(title, 80, 1, f);
			fread((void*) &nFaces, 4, 1, f);
			float v[12]; // normal=3, vertices=3*3 = 12
			unsigned short uint16;


			for (size_t i = 0; i < nFaces; ++i)
			{
				for (size_t j = 0; j < 12; ++j)
				{
					fread((void*) &v[j], sizeof(float), 1, f);
				}
				fread((void*) &uint16, sizeof(unsigned short), 1, f); // spacer between successive faces


				vec3 A =  vec3( v[3], v[4], v[5]);
				vec3 B =  vec3( v[6], v[7], v[8]);
				vec3 C =  vec3( v[9], v[10], v[11]);
				vec3 normal(v[0], v[1], v[2]);


				sdot_t* dotA = create_dot( idot++, A ); //dot that dont need to simulate, only simulate parent enough
				sdot_t* dotB = create_dot( idot++, B );
				sdot_t* dotC = create_dot( idot++, C );

				triangle_ref* tri = make_face( dotA, dotB, dotC );
				iface++;

				//fixme: this line is disable all collision
				tri->is_colli = false;
			}


			printf("\n[ stl_rigid_triangle ] dotv: %ld face: %ld\n", idot, iface  );
			fclose(f);
		}

		return 0;
	}


public:

	std::vector<primitive_base*> activev;

	stl_rigid_triangle(object_base* ref = NULL)
		:rigid_triangle( ref )
	{

//		double t = 0.3;
//		segment_t* seg = new segment_t( vec3(0.0,0.0,-20.0),vec3(0.0,0.0,0.0), vec3(0.0, t*10.0, t*-12.0), this);
//		seg->direc = false;
//		activev.push_back( seg );
//
//		seg = new segment_t( vec3(0.0,0.0,-20.0),vec3(0.0,0.0,0.0), vec3(0.0, t*-10.0, t*-12.0), this);
//		seg->direc = false;
//		activev.push_back( seg );
//
//		seg = new segment_t( vec3(0.0,0.0,-20.0),vec3(0.0,0.0,0.0),vec3(0.0,0.0,60.0), this);
//		seg->direc = false;
//		activev.push_back( seg );

	}


	stl_rigid_triangle(const vec3& frame_pos, object_base* ref = NULL)
		:rigid_triangle( frame_pos, ref )
	{

	}


	void load_from_file(const std::string& filename, bool is_binary_format)
	{
		stl_to_memory( filename.c_str(), is_binary_format );
	}


	virtual GLint* load_texture(const std::string filename)
	{
		texture = object_base::load_texture( filename );
		for(auto iter=facev.begin(); iter != facev.end(); ++iter)
		{
			if( (*iter) == NULL )
				continue;

			(*iter)->set_texture( texture );
			(*iter)->set_UV(0, 0.0, 0.0 );
			(*iter)->set_UV(1, 1.0, 0.0 );
			(*iter)->set_UV(2, 1.0, 1.0 );

		}

		return texture;
	}



	virtual bool get_active_collision(std::vector<primitive_base*>& objv )
	{
		objv.insert( objv.end(), activev.begin(), activev.end() );
		return true;
	}


	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		//TODO TODO this one is remove all collision
		//objv.insert( objv.end(), facev.begin(), facev.end() );
		return true;
	}


	//todo this is creaeted  to make one show who
	virtual void post_update()
	{
		/* update composite body segment is for active collision*/
		for(size_t i=0; i<activev.size(); i++)
		{
			segment_t* seg = (segment_t*)activev[i];
			seg->update_global_frame();
		}

	}



	virtual void clear_all_force()
	{
		object_base::clear_all_force();
		for(int i=0; i<activev.size(); i++)
		{
			activev[i]->clear_all_force();
			activev[i]->is_colli = false;
			activev[i]->colli_depth = 0.0;
		}
	}

};














std::vector<std::string> split(const std::string& s, const std::string& delim, const bool keep_empty = true)
{
    std::vector<std::string> result;
    if (delim.empty())
    {
        result.push_back(s);
        return result;
    }

    std::string::const_iterator substart = s.begin(), subend;
    while (true) {
        subend = search(substart, s.end(), delim.begin(), delim.end());
        std::string temp(substart, subend);
        if (keep_empty || !temp.empty())
        {
            result.push_back(temp);
        }
        if (subend == s.end()) {
            break;
        }
        substart = subend + delim.size();
    }

    return result;
}



template <class ContainerT>
void tokenize(const std::string& str, ContainerT& tokenv, const std::string& delim = " ", bool keep_empty = true)
{
   std::string::size_type pos, last_pos = 0;

   using value_type = typename ContainerT::value_type;
   using size_type  = typename ContainerT::size_type;

   while(true)
   {
      pos = str.find_first_of(delim, last_pos);
      if(pos == std::string::npos)
      {
         pos = str.length();

         if(pos != last_pos || keep_empty)
            tokenv.push_back( value_type(str.data()+last_pos,(size_type)pos-last_pos) );

         break;
      }
      else
      {
         if(pos != last_pos || keep_empty)
            tokenv.push_back( value_type(str.data()+last_pos,(size_type)pos-last_pos ) );
      }

      last_pos = pos + 1;
   }

}



void skip_line(std::istream& is)
{
    char next;
	is >> std::noskipws;
    while( (is >> next) && ('\n' != next) );
}



bool skip_comment(std::istream& is)
{
	char next;
	while( is >> std::skipws >> next )
    {
		is.putback(next);
		if ('#' == next)
			skip_line(is);
		else
			return true;
    }
    return false;
}








class obj_rigid_triangle: public rigid_triangle
{

	double scale;
protected:

	std::vector<vec3> UVtemp;

	bool parse_line(std::istream& is)
	{
		std::string mytype; /* store define tag for that line */

		if( !(is >> mytype) )
			return false;

		std::cout << mytype << std::endl;

		if( mytype == "mtllib")
		{	/* reference material using .mtl */
			std::string mat_filename;
			is >> mat_filename;
			//printf("mat file: %s\n", mat_filename.c_str() );
		}
		else if( mytype == "usemtl" )
		{
			std::string myline;
			getline( is, myline );
		}
		else if( mytype == "v" )
		{	/* vertex position */
			float x, y, z; /* w is weight */

			std::string myline;
			getline( is, myline );

			std::istringstream(myline) >> x >> y >> z;

			vec3 A =  vec3( x*scale, y*scale, z*scale);
			sdot_t* dotA = create_dot( A );
		}
		else if( mytype == "vt" )
		{	/* texture UV coordinate */
			float u, v, w;

			is >> u >> v >> w;
			is.clear();      /* handle w (wight) is not available, then clear error flag */

			vec3 uv( u, v, 0.0);

			/* store UV to array */
			UVtemp.push_back( uv );
		}
		else if( mytype == "vn" )
		{	/* vertex normal */
			float x, y, z;
			is >> x >> y >> z;
			if( !is.good() )
			{  /* handle error -1#IND00 */
				x = y = z = 0.0;
				is.clear();
				skip_line( is );
			}

			vec3 normal(x,y,z);

			/* dont care normal */
		}
		else if( mytype == "f" )
		{	/*	face index */
			const int Nmax = 10;
			int vi[Nmax];                         /* vertex indices.  */
			int ni[Nmax] = { -1, -1, -1, -1, };   /* normal indices   */
			int ti[Nmax] = { -1, -1, -1, -1, };   /* texture indices  */

			std::string myline;
			getline( is, myline );
			std::cout << myline << std::endl;

			std::vector<std::string>  tokenv = split(myline," ", false );
			size_t token_siz = tokenv.size();

			//assert( token_siz == 3 ); //now only make triangle
			for( auto iter = tokenv.begin(); iter <tokenv.end(); ++iter)
				std::cout << (*iter) << std::endl;

			for(int i= 0; i<token_siz && i<Nmax; i++)
			{
				std::vector<std::string> substr = split( tokenv[i], "//", false );
				size_t siz = substr.size();

				if( siz == 1 )
				{	/*  siz == 1, cannot split with "//"  */
					std::vector<std::string> sub;
					sub = split( tokenv[i], "/", false );
					size_t sz = sub.size();
					if( sz == 1 )
					{ 	/* vertex */
						std::istringstream(sub[0]) >> vi[i];
					}
					else if( sz == 2 )
					{	/* vertex_index/texture_index */
						std::istringstream(sub[0]) >> vi[i];
						std::istringstream(sub[1]) >> ti[i];
					}
					else if( sz == 3 )
					{	/* vertex_index/texture_index/normal_index */
						std::istringstream(sub[0]) >> vi[i];
						std::istringstream(sub[1]) >> ti[i];
						std::istringstream(sub[2]) >> ni[i];
					}
				}
				else if( siz == 2 )
				{	/* vertex_index//normal_index */
					std::istringstream(substr[0]) >> vi[i];
					std::istringstream(substr[1]) >> ni[i];
				}

				//printf("[%d]    vi: %d ti: %d  ni: %d\n",i, vi[i], ti[i], ni[i] ); fflush( stdout );
			}

			/*-----------------------*/
			/*     create face       */
			/*-----------------------*/
			assert( vi[0] - 1 >= 0 );
			assert( vi[1] - 1 >= 0 );
			assert( vi[2] - 1 >= 0 );
			sdot_t* dotA = sdotv[ vi[0]-1 ];
			sdot_t* dotB = sdotv[ vi[1]-1 ];
			sdot_t* dotC = sdotv[ vi[2]-1 ];
			assert( dotA != NULL );
			assert( dotB != NULL );
			assert( dotC != NULL );
//			dotA->get_pos().debug("dotA: ");
//			dotB->get_pos().debug("dotB: ");
//			dotC->get_pos().debug("dotC: ");
			triangle_ref* tri = make_face( dotA, dotB, dotC );


			if( ti[0] != -1 )
			{
				assert( ti[0] - 1 >= 0 );
				assert( ti[1] - 1 >= 0 );
				assert( ti[2] - 1 >= 0 );

				vec3* uvA = &UVtemp[ ti[0]-1 ];
				vec3* uvB = &UVtemp[ ti[1]-1 ];
				vec3* uvC = &UVtemp[ ti[2]-1 ];

				assert( uvA != NULL );
				assert( uvB != NULL );
				assert( uvC != NULL );

				tri->set_UV( 0, (*uvA)[0], (*uvA)[1] );
				tri->set_UV( 1, (*uvB)[0], (*uvB)[1] );
				tri->set_UV( 2, (*uvC)[0], (*uvC)[1] );
			}


			if( ni[0] != -1 )
			{
				/* normal vertex */
			}


			is.clear(); /* clear error flag */
		}

		else
		{
			std::string str;
			is >> str;
			std::cerr << "unknown: " << mytype << " " << str << std::endl;
			exit(-1);
			skip_line(is);
		}

		return true;
	}



	void parse_obj(const std::string& filename )
	{
		std::ifstream in(filename, std::ios::in|std::ios::ate );
		if (!in.good())
		{
			std::cerr << "warning: cannot file " << filename << std::endl;
			return;
		}

		/* get file size */
		const size_t siz = in.tellg();
		/* go back to the beginning of the file */
		in.seekg( 0, std::ios::beg);

		while( skip_comment(in) )
		{
			if( !parse_line(in) )
				break;
		}

		in.close();

		printf("=== load obj complete ===\n" ); fflush( stdout );
	}

public:


	obj_rigid_triangle(object_base* ref = NULL)
		:rigid_triangle( ref ), scale(1.0)
	{
	}


	obj_rigid_triangle(const vec3& frame_pos, object_base* ref = NULL)
		:rigid_triangle( frame_pos, ref ), scale(1.0)
	{

	}


	void load_from_file(const std::string& filename, double scale=1.0)
	{
		this->scale = scale;
		parse_obj( filename );
	}


	virtual GLint* load_texture(const std::string filename)
	{
		texture = object_base::load_texture( filename );

		for(auto iter=facev.begin(); iter != facev.end(); ++iter)
		{
			if( (*iter) == NULL )
				continue;

			(*iter)->set_texture( texture );
		}

		return texture;
	}


};


























class obj_spring_triangle: public surface_triangle_spring
{


protected:

	std::vector<vec3> UVtemp;

	bool parse_line(std::istream& is)
	{
		std::string mytype; /* store define tag for that line */

		if( !(is >> mytype) )
			return false;

		//std::cout << mytype << std::endl;

		if( mytype == "mtllib")
		{	/* reference material using .mtl */
			std::string mat_filename;
			is >> mat_filename;
			//printf("mat file: %s\n", mat_filename.c_str() );
		}
		else if( mytype == "usemtl" )
		{
			std::string myline;
			getline( is, myline );
		}
		else if( mytype == "v" )
		{	/* vertex position */
			float x, y, z; /* w is weight */

			std::string myline;
			getline( is, myline );

			std::istringstream(myline) >> x >> y >> z;

			const double scale = 10.0; //fixme just for testing
			vec3 A =  vec3( x*scale, y*scale, z*scale);
			sdot_t* dotA = create_dot( A );
		}
		else if( mytype == "vt" )
		{	/* texture UV coordinate */
			float u, v, w;

			is >> u >> v >> w;
			is.clear();      /* handle w (wight) is not available, then clear error flag */

			vec3 uv( u, v, 0.0);

			/* store UV to array */
			UVtemp.push_back( uv );
		}
		else if( mytype == "vn" )
		{	/* vertex normal */
			float x, y, z;
			is >> x >> y >> z;
			if( !is.good() )
			{  /* handle error -1#IND00 */
				x = y = z = 0.0;
				is.clear();
				skip_line( is );
			}

			vec3 normal(x,y,z);

			/* dont care normal */
		}
		else if( mytype == "f" )
		{	/*	face index */
			const int Nmax = 10;
			int vi[Nmax];                         /* vertex indices.  */
			int ni[Nmax] = { -1, -1, -1, -1, };   /* normal indices   */
			int ti[Nmax] = { -1, -1, -1, -1, };   /* texture indices  */

			std::string myline;
			getline( is, myline );
			//std::cout << myline << std::endl;

			std::vector<std::string>  tokenv = split(myline," ", false );
			size_t token_siz = tokenv.size();

			assert( token_siz == 3 ); //now only make triangle
			//for( auto iter = tokenv.begin(); iter <tokenv.end(); ++iter)
			//	std::cout << (*iter) << std::endl;

			for(int i= 0; i<token_siz && i<Nmax; i++)
			{
				std::vector<std::string> substr = split( tokenv[i], "//", false );
				size_t siz = substr.size();
				if( siz == 1 )
				{	/*  siz == 1, cannot split with "//"  */
					std::vector<std::string> sub;
					sub = split( tokenv[i], "/", false );
					size_t sz = sub.size();
					if( sz == 1 )
					{ 	/* vertex */
						std::istringstream(sub[0]) >> vi[i];
					}
					else if( sz == 2 )
					{	/* vertex_index/texture_index */
						std::istringstream(sub[0]) >> vi[i];
						std::istringstream(sub[1]) >> ti[i];
					}
					else if( sz == 3 )
					{	/* vertex_index/texture_index/normal_index */
						std::istringstream(sub[0]) >> vi[i];
						std::istringstream(sub[1]) >> ti[i];
						std::istringstream(sub[2]) >> ni[i];
					}
				}
				else if( siz == 2 )
				{	/* vertex_index//normal_index */
					std::istringstream(substr[0]) >> vi[i];
					std::istringstream(substr[1]) >> ti[i];
				}

				//printf("[%d]    vi: %d ti: %d  ni: %d\n",i, vi[i], ti[i], ni[i] );
			}

			sdot_t* dotA = sdotv[ vi[0]-1 ];
			sdot_t* dotB = sdotv[ vi[1]-1 ];
			sdot_t* dotC = sdotv[ vi[2]-1 ];

			//dotA->get_pos().debug("dotA: ");
			//dotB->get_pos().debug("dotB: ");
			//dotC->get_pos().debug("dotC: ");

			triangle_ref* tri = make_face( dotA, dotB, dotC );
			vec3* uvA = &UVtemp[ ti[0]-1 ];
			vec3* uvB = &UVtemp[ ti[1]-1 ];
			vec3* uvC = &UVtemp[ ti[2]-1 ];


			tri->set_UV( 0, (*uvA)[0], (*uvA)[1] );
			tri->set_UV( 1, (*uvB)[0], (*uvB)[1] );
			tri->set_UV( 2, (*uvC)[0], (*uvC)[1] );

			is.clear(); /* clear error flag */
		}

		else
		{
			std::string str;
			is >> str;
			std::cerr << "unknown: " << mytype << " " << str << std::endl;
			skip_line(is);
		}



		return true;
	}



	void parse_obj(const std::string& filename )
	{
		std::ifstream in(filename, std::ios::in|std::ios::ate );
		if (!in.good())
		{
			std::cerr << "warning: cannot file " << filename << std::endl;
			return;
		}

		/* get file size */
		const size_t siz = in.tellg();
		/* go back to the beginning of the file */
		in.seekg( 0, std::ios::beg);

		while( skip_comment(in) )
		{
			if( !parse_line(in) )
				break;
		}
		in.close();




		/*----------------------------*/
		/* make spring for every face */
		/*----------------------------*/
		float Kspring = 500.0;
		float Kdamper = -1.0;
		float Lrest   = 0.1;
		float zz  = 1;

		for(auto iter=facev.begin(); iter<facev.end(); ++iter)
		{
			if( (*iter) == NULL )
				continue;

			triangle_ref* tri = (triangle_ref*)(*iter);
			vec3 mid = tri->A() + tri->B() + tri->C();
			vec3 pos = mid + (*iter)->normal*zz;

			sdot_t* s = create_dot( pos, this );
			s->set_mass( 0.0 );
			connect_dot( s, (sdot_t*)tri->dotv[0], Kspring, Kdamper, Lrest );
			connect_dot( s, (sdot_t*)tri->dotv[1], Kspring, Kdamper, Lrest );
			connect_dot( s, (sdot_t*)tri->dotv[2], Kspring, Kdamper, Lrest );


			//printf("dot size %d\n", get_dotv_size() ); fflush( stdout );
		}



		printf("=== load obj complete ===\n" ); fflush( stdout );
	}

public:


	obj_spring_triangle(object_base* ref = NULL)
		:surface_triangle_spring( ref )
	{
	}



	void load_from_file(const std::string& filename)
	{
		parse_obj( filename );
	}


	virtual GLint* load_texture(const std::string filename)
	{
		texture = object_base::load_texture( filename );

		for(auto iter=facev.begin(); iter != facev.end(); ++iter)
		{
			if( (*iter) == NULL )
				continue;

			(*iter)->set_texture( texture );
		}

		return texture;
	}


	virtual bool get_active_collision(std::vector<primitive_base*>& objv )
	{
		return true;
	}


	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		//TODO TODO this one is remove all collision
		objv.insert( objv.end(), facev.begin(), facev.end() );
		return true;
	}


};
























#endif /* PRIMITIVE_HPP_ */
