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
 * mygl.hpp
 * Copyright (C) 2014 [theparitt]
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by theparit peerasathien <theparitt@gmail.com>, December 2014
 */


#pragma once

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




#include <iostream>
#include <cassert>
#include <cstdint>
#include "kinematics.hpp"
#include "SOIL.h"



//--------- OPENGL constant -------//
const int   WIDTH = 1024;
const int   HEIGHT = 800;
const char* TITLE = "moodaeng";
//---------------------------------//





//-------- CAMERA CONSTANT ------------//
const double MIN_ZOOM =  2.0;
const double MAX_ZOOM = 200.0;




int screen_width  = WIDTH;
int screen_height = HEIGHT;


float near_plane  = 1.0f;
float far_plane   = 2000.0f;
vec3 camera_pos;
vec3 camera_target_pos;
vec3 camera_prev;
vec3 camera_target_prev;
vec3 camera_up;


double camera_pitch_deg;
double camera_yaw_deg;
double camera_distance;


void update_camera();
void reshape_callback(int w, int h);
void idle_callback();
void zoom_camera( double distance );
void rotate_camera(double& pitch_yaw, double step);


static void init_GL()
{
	char *argv[1];
	int argc=1;
	argv [0]= strdup( TITLE );

	glutInit( &argc, argv );
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow( TITLE );

	printf("OpenGL Renderer version: %s\nOpenGL Version: %s\n",
			glGetString(GL_RENDERER), glGetString(GL_VERSION) );


	glShadeModel(GL_SMOOTH);	             /* Enable Smooth Shading */
	glClearColor(0.6f, 0.65f, 0.85f, 0.0f);	 /* Black Background */
	glClearDepth(1.0f);	                     /* Depth Buffer Setup */
	glEnable(GL_DEPTH_TEST);	             /* Enables Depth Testing */
	glDepthFunc(GL_LEQUAL);	                 /* The Type Of Depth Testing To Do */
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	/* Really Nice Perspective Calculations */


	/* create some floats for our ambient, diffuse, specular and position */
	GLfloat ambient[] = { 0.9f, 0.9f, 0.9f, 1.0f };  /* dark grey */
	GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };  /* white */
	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; /* white */
	GLfloat position[] = { 5.0f, 10.0f, 1.0f, 0.0f };
	/* set the ambient, diffuse, specular and position for LIGHT0 */
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glEnable(GL_LIGHTING); 		 /* enables lighting */
	glEnable(GL_LIGHT0);   		 /* enables the 0th light */
	glEnable(GL_COLOR_MATERIAL); /* colors materials when lighting is enabled */


	/* enable specular lighting via materials */
	//glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	//glMateriali(GL_FRONT, GL_SHININESS, 15);

	/* enable smooth shading */
	//glShadeModel(GL_SMOOTH);
	/* enable depth testing to be 'less than' */
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	/* set the backbuffer clearing color to a lightish blue */
	glClearColor(0.6, 0.65, 0.85, 0);

	/* enable alpha channel (transparent) */
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	/*------------------------------*/
	/*     initialize variables     */
	/*------------------------------*/
	camera_pitch_deg = 0.0;
	camera_yaw_deg   = 0.0;
	camera_distance = 100.0;

	camera_pos( 0.0, 0.0, camera_distance );
	camera_target_pos( 0.0, 0.0, 0.0 );
	camera_up( 0.0, 1.0, 0.0 );
}



/* this function is called when the window is resized */
void reshape_callback(int w, int h)
{
	screen_width = w;
	screen_height = h;
	glViewport( 0, 0, w, h);

	update_camera();
}



const double CAMARA_STEP = 1.0;
/* using for zoom-in / zoom-out */
void keyboard_callback(unsigned char key, int x, int y)
{

	if(key == 'w' )
	{
		zoom_camera( CAMARA_STEP );
	}
	else if(key == 's')
	{
		zoom_camera( -CAMARA_STEP );
	}

}



void special_callback(int key, int x, int y)
{
	if(key == GLUT_KEY_LEFT)
	{
		rotate_camera( camera_yaw_deg, CAMARA_STEP );
	}
	else if(key == GLUT_KEY_RIGHT)
	{
		rotate_camera( camera_yaw_deg, -CAMARA_STEP );
	}
	else if(key == GLUT_KEY_UP)
	{
		rotate_camera( camera_pitch_deg, CAMARA_STEP );
	}
	else if(key == GLUT_KEY_DOWN)
	{
		rotate_camera( camera_pitch_deg, -CAMARA_STEP );
	}
}




bool mouse_left_hold_start = false;
bool mouse_middle_hold_start = false;
bool mouse_right_hold_start = false;
int mouse_hold_start_x = 0;
int mouse_hold_start_y = 0;
int mouse_hold_last_x = 0;
int mouse_hold_last_y = 0;



//zoon in/zoom out around m_cameraTarget
void mousewheel_callback(int button, int dir, int x, int y)
{
	float mag = 1.0f;

	vec3 eye_normal = camera_pos - camera_target_pos;
	double mymag = eye_normal.mag();

	eye_normal.normalize();
	eye_normal * mag;


	if( dir > 0 )
	{
		if( mymag >= MIN_ZOOM)
			camera_pos += eye_normal;
	}
	else
	{
		if( mymag <= MAX_ZOOM )
			camera_pos -= eye_normal;
	}

}



void mousebutton_callback(int button, int state, int x, int y)
{
	if( button==GLUT_MIDDLE_BUTTON && state == GLUT_DOWN  )
	{
		mouse_middle_hold_start = true;
		mouse_hold_start_x = x;
		mouse_hold_start_y = y;

		camera_prev = camera_pos;
		//camera_pos.debug("camera:\n");
	}
	else if( button==GLUT_MIDDLE_BUTTON && state == GLUT_UP )
	{
		mouse_middle_hold_start = false;
		mouse_hold_start_x = x;
		mouse_hold_start_y = y;
	}

}



void mousemove_callback(int x, int y)
{

	if(mouse_middle_hold_start)
	{
		float dist_x =  (x-mouse_hold_start_x)/(HEIGHT/2.0f) * (M_PI);
		float dist_y =  (y-mouse_hold_start_y)/(HEIGHT/2.0f) * (M_PI); //use height instead of width


		if( dist_y < 0)
			dist_y += 2*M_PI;
		if( dist_y >= 2*M_PI )
			dist_y -= 2*M_PI;


		if( dist_x < 0)
			dist_x += 2*M_PI;
		if( dist_x >= 2*M_PI )
			dist_x -= 2*M_PI;


		/*------------------------------*/
		/*   rotate about a point(a,b)  */
		/*------------------------------*/
		static matrix_homo T0, T1;
		static matrix_homo R0, R1, R;


		T0.set_identity();
		T0.set_pos( camera_target_pos * -1 );
		T0.set_rotx( -dist_y );

		T1.set_identity();
		T1.set_roty( -dist_x );
		T1.set_pos( camera_target_pos );

		/*------------------------------------------------------------------------------------*/
		/* R = Translate(back to current position) * RotateX * RotateY * Translate(to origin) */
		/*------------------------------------------------------------------------------------*/
		R = T1*T0;
		camera_pos =  R * camera_prev;
	}

}




/*----------------------------------------*/
/*    simple draw  openGL function        */
/*----------------------------------------*/



void draw_origin(float scale=1.0f)
{
	glLineWidth(20);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( scale*1.0f, 0.0f, 0.0f);
	glEnd();

	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f, scale*1.0f, 0.0f);
	glEnd();

	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f, 0.0f, scale*1.0f);
	glEnd();
}



void draw_axis(float scale=1.0f)
{
	glColor3ub(140, 35, 24);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( scale*1.0f, 0.0f, 0.0f);
	glEnd();

	glColor3ub(94, 140, 106);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f, scale*1.0f, 0.0f);
	glEnd();

	glColor3ub(242, 196, 90);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f, 0.0f, scale*1.0f);
	glEnd();
}


void draw_axis(const vec3& pos, const vec3& rot, float scale=1.0f)
{
	glPushMatrix();
	glTranslated( pos[0], pos[1], pos[2] );
	glRotated( RAD( rot[2] ), 0, 0, 1 );
	glRotated( RAD( rot[1] ), 0, 1, 0 );
	glRotated( RAD( rot[0] ), 1, 0, 0 );

	glColor3ub(218, 118, 152);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( scale*1.0f, 0.0f, 0.0f);
	glEnd();

	glColor3ub(76, 147, 133);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f, scale*1.0f, 0.0f);
	glEnd();

	glColor3ub(242, 209, 179);
	glBegin(GL_LINES);
		glVertex3f( 0.0f, 0.0f, 0.0f);
		glVertex3f( 0.0f, 0.0f, scale*1.0f);
	glEnd();

	glPopMatrix();
}


void draw_circle(double radius, double point_siz=2.0, double nslice=1000.0)
{
	glPointSize( point_siz );
	glBegin(GL_LINE_LOOP);
	for(int i=0;i<nslice;++i)
	{
		glVertex3f( radius*cos(2*M_PI*i/nslice) , radius*sin(2*M_PI*i/nslice), 0 );
	}
	glEnd();
}



void draw_sphere(const vec3& center, double radius=1.0 )
{
	glPushMatrix();
		glTranslatef( center(0),center(1), center(2) );
		glutSolidSphere(radius, 20, 20);
	glPopMatrix();
}


void stoke_color(const double r, const double g, const double b)
{
	glColor3f( r, g, b);
}


void stoke_colori(const int r, const int g, const int b)
{
	glColor3ub( r, g, b);
}


void stoke_color(const float r, const float g, const float b, const float a)
{
	glColor4f( r, g, b, a);
}


void stoke(const float line_width, const float r, const float g, const float b )
{
	glLineWidth(line_width);
	glColor3f( r, g, b );
}


void stoke(const float line_width, float r, float g, float b, float a )
{
	glLineWidth(line_width);
	glColor4f( r, g, b, a );
}



void stoke(float line_width)
{
	glLineWidth(line_width);
}



template <typename T>
void draw_string(T& a, float x, float y )
{
	glDisable(GL_LIGHTING);

	std::string s = to_string( a );
	glRasterPos2f(x, y);

	auto iter = s.begin();
	auto end  = s.end();
	for (; iter !=  end; ++iter )
	{
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *iter);
	}

	glEnable(GL_LIGHTING);
}



template <typename T>
void draw_string(T& a, float x, float y, float z )
{
	glDisable(GL_LIGHTING);

	std::string s = to_string( a );
	glRasterPos3f(x, y, z);

	auto iter = s.begin();
	auto end  = s.end();
	for (; iter !=  end; ++iter )
	{
		glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, *iter);
	}

	glEnable(GL_LIGHTING);
}



void draw_dot(const vec3& A, double siz)
{
	glPointSize( siz );
	glBegin(GL_POINTS);
		glVertex3f( A[0],A[1], A[2] );
	glEnd();
}



void draw_line(const vec3& A, const vec3& B)
{
	glBegin(GL_LINES);
		glVertex3f( A[0], A[1], A[2]);
		glVertex3f( B[0], B[1], B[2]);
	glEnd();
}


void draw_line(const vec3& A, const vec3& normal, double siz)
{
	vec3 B = A + siz * normal;
	draw_line( A, B );
}


void draw_line_ortho_plane(const vec3& pos, const vec3& normal)
{
	double siz = 5;
	draw_line( pos, pos + normal * siz );
}


template <typename T>
void draw_line_and_string(const vec3& A, const vec3& B, T& s)
{
	draw_line( A, B );
	vec3 mid = ( A + B ) /2;
	draw_string( s, mid[0], mid[1] );
}



//segment a --> b
void draw_ray(const vec3& A, const vec3& B)
{
	glLineWidth(2.0f);
	glColor4f( 0.6, 0.6, 0.6, 1.0 );
	glBegin(GL_LINES);
		glVertex3f( A[0], A[1], A[2]);
		glVertex3f( B[0], B[1], B[2]);
	glEnd();

	glPushMatrix();
		glColor4f(0.6, 0.6, 0.6, 1.0);
		glTranslatef( B[0], B[1], B[2] );
		glutSolidSphere(0.5, 20, 20);
	glPopMatrix();


	glPushMatrix();
		glColor4f(0.6, 0.6, 0.0, 1.0);
		glTranslatef( A[0], A[1], A[2] );
		glutSolidSphere(0.5, 20, 20);
	glPopMatrix();
}




void update_camera()
{
	if(screen_width == 0 )
		screen_width = 1;

	if(screen_width == 0 )
		screen_height = 1;


	glMatrixMode(GL_PROJECTION);
	/* set it to the matrix-equivalent of 1 */
	glLoadIdentity();
	/* determine the aspect ratio of the screen */
	float aspectRatio = screen_width / (float)screen_height;

	glFrustum (-aspectRatio * near_plane, aspectRatio * near_plane, -near_plane, near_plane, near_plane, far_plane);
	
	glMatrixMode(GL_MODELVIEW);
	/* set matrix to identity */
	glLoadIdentity();

	double pitch_rad = DEG( camera_pitch_deg );
	double yaw_rad   = DEG( camera_yaw_deg );

	quaternion camera_yaw( yaw_rad, camera_up[0], camera_up[1], camera_up[2] );

	/* relative distance from camera_target_pos */
	vec3 forward( 0.0, 0.0, camera_distance );
	vec3 lookat( 0.0, 0.0, -camera_distance);

	lookat.normalize();
	/* find axis on rihgt direction */
	vec3 camera_right = cross( lookat, camera_up );
	/* rotae around camera_right */
	quaternion camera_pitch( pitch_rad, camera_right );

	camera_pos = (camera_yaw.to_matrix_rot()*camera_pitch.to_matrix_rot())*forward + camera_target_pos;


	gluLookAt( camera_pos(0), camera_pos(1), camera_pos(2),
				camera_target_pos(0), camera_target_pos(1), camera_target_pos(2),
				camera_up(0), camera_up(1), camera_up(2) );
}



void zoom_camera(double step )
{
	camera_distance -= step;
	if( camera_distance < MIN_ZOOM )
		camera_distance = MIN_ZOOM;

	update_camera();
}



void rotate_camera(double& pitch_yaw, double step)
{
	pitch_yaw -= step;
	/* keep the value within bounds */
	if (pitch_yaw < 0.0) pitch_yaw += 360.0;
	if (pitch_yaw >= 360.0) pitch_yaw -= 360.0;
	/* update the camera since we changed the angular value */
	update_camera();
}





GLuint load_texture_bmp( const char * filename, bool wrap=true)
{
    GLuint texture;
    int width, height;
    char* data;
    FILE* file;


    /* open texture data */
    file = fopen( filename, "rb" );
    if ( file == NULL )
    {
    	assert( false );
    	exit(-1);
    }

    /* allocate buffer */
    width = 1164;
    height = 1162;
    data = new char[ width * height * 3 ];

    /* read texture data */
    fread( data, width * height * 3, 1, file );


    fclose( file );

    /* allocate a texture name */
    glGenTextures( 1, &texture );

    /* select our current texture */
    glBindTexture( GL_TEXTURE_2D, texture );

    /* select modulate to mix texture with color for shading */
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );

    /* when texture area is small, bilinear filter the closest mipmap */
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                     GL_LINEAR_MIPMAP_NEAREST );
    /* when texture area is large, bilinear filter the first mipmap */
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

    /* if wrap is true, the texture wraps over at the edges (repeat) */
    /*       ... false, the texture ends at the edges (clamp)        */
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP );
    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP );

    /* build our texture mipmaps */
    gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data );

    /* free buffer */
    free( data );

    return texture;
}






class material_t
{

public:


	float ambient[4];
	float diffuse[4];
	float specular[4];
	float emission[3];
	float shiness;

	material_t( float amb0, float amb1, float amb2,
				float dif0, float dif1, float dif2,
				float spec0, float spec1, float spec2,
				float shiness = 100.0f)
	{
		ambient[0] = amb0;
		ambient[1] = amb1;
		ambient[2] = amb2;
		ambient[3] = 1.0f;

		diffuse[0] = dif0;
		diffuse[1] = dif1;
		diffuse[2] = dif2;
		diffuse[3] = 1.0f;

		specular[0] = spec0;
		specular[1] = spec1;
		specular[2] = spec2;
		specular[3] = 1.0f;

		this->shiness = shiness;
	}


	void operator()( float amb0, float amb1, float amb2,
					 float dif0, float dif1, float dif2,
					 float spec0, float spec1, float spec2,
					 float shiness = 100.0f)
	{
		ambient[0] = amb0;
		ambient[1] = amb1;
		ambient[2] = amb2;
		ambient[3] = 1.0f;

		diffuse[0] = dif0;
		diffuse[1] = dif1;
		diffuse[2] = dif2;
		diffuse[3] = 1.0f;

		specular[0] = spec0;
		specular[1] = spec1;
		specular[2] = spec2;
		specular[3] = 1.0f;

		this->shiness = shiness;
	}


	void apply_material()
	{

		glColorMaterial ( GL_FRONT_AND_BACK, GL_EMISSION ) ;
		glEnable( GL_COLOR_MATERIAL );
		glMaterialfv( GL_FRONT, GL_AMBIENT  , ambient);
		glMaterialfv( GL_FRONT, GL_DIFFUSE  , diffuse);
		glMaterialfv( GL_FRONT, GL_SPECULAR , specular);
		glMaterialfv( GL_FRONT, GL_EMISSION , emission);
		glMaterialf(  GL_FRONT, GL_SHININESS, shiness );

		glDisable( GL_COLOR_MATERIAL );
	}


	void apply_material_with_emission()
	{
		glEnable( GL_COLOR_MATERIAL );
		glMaterialfv( GL_FRONT, GL_AMBIENT  , ambient);
		glMaterialfv( GL_FRONT, GL_DIFFUSE  , diffuse);
		glMaterialfv( GL_FRONT, GL_SPECULAR , specular);
		glMaterialfv( GL_FRONT, GL_EMISSION , emission);
		glMaterialf(  GL_FRONT, GL_SHININESS, shiness );
	}


	void disable_material()
	{
		glDisable( GL_COLOR_MATERIAL );
	}

};





/*--------------------------------*/
/* example of material parameters */
/*--------------------------------*/

material_t silver_material( 0.19225, 0.19225, 0.19225,    /* ambient material */
							0.50754, 0.50754, 0.50754,    /* diffuse material */
							0.508273, 0.508273, 0.508273, /* specular material */
							51.2 );                       /* shineess          */




