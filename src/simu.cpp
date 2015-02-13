//============================================================================
// Name        : simu_osx.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "mygl.hpp"


#include "primitive.hpp"
#include "collision.hpp"
#include "tictoc.hpp"

#define HAPTICS
#include "myhaptics.hpp"
#include "myexample.hpp"

using namespace std;


world_t* world;
stl_rigid_triangle* rabbit;
cloth_rectangle* mycloth;
cloth_triangle*  mycloth3;


rectangle_t* rec;
sphere_t* sphere;
cylinder_t* cylin;
triangle_t* tri;
segment_t* seg;
segment_t* seg2;
dot_t* dotdot;
dot_t* dotdot2;
sphere_t* ball;
sphere_t* ball2;
sphere_t* sky;
box_t* box;
dumbbell_t* db;
suture_t* suture;
haptics_holder* haptics_left;
haptics_holder* haptics_right;
segment_t* line;
ray_t* ray;
plane_t* plane;
wave_surface* sneha;
stl_rigid_triangle* laparo;
stl_rigid_triangle* racquet;
obj_spring_triangle* mario;
obj_rigid_triangle* yoshi;

laparo_t* laparox;
laparo_t* laparoy;
laparo_t* laparox2;
laparo_t* laparoy2;
void idle_callback();
void display_callback();

GLuint texture ;
spring_body* spring2;



vec3 cutA;
vec3 cutB;






int main()
{

	init_GL();
	init_haptics();

	world = world_t::instance();


	/* example of creating sphere_t */
	sphere = new sphere_t( vec3(10,10,0), 5, vec3(0,10,0) );
	sphere->name = "sphere";
	sphere->load_texture( "basketball2.jpg" );
	sphere->set_mass( 0.0 );




	/* example of creating triangle_t */
	tri = new triangle_t( vec3(10.0,0.0,-50.0), vec3(-20.0,10.0,0.0), vec3(20.0,10.0,0.0), vec3(0.0,30.0,0.0) );
	tri->name = "tri";
	tri->set_rot( DEG(45), 0, 0 );
	tri->load_texture("001.png");
	tri->set_UV(0, 0.0, 1.0 );
	tri->set_UV(1, 0.0, 0.0 );
	tri->set_UV(2, 1.0, 1.0 );
	tri->set_mass( 0.0 );


	/* example of creating cloth_rectangle */
	mycloth = new cloth_rectangle( vec3(-50,0,0), 20, 20 );
	mycloth->set_mass( 0.0 );
	cutA = vec3(14.0, 0.0, 0.0);
	cutB = vec3(14.0, 0.0, 50.0) ;
	mycloth->cut_face(0, cutA, cutB);
	mycloth->load_texture( "human2.jpg" );



	/* example of creating rectangle_t */
	double plane_half_siz = 100;
	rec = new rectangle_t( vec3(0,-10,0), vec3( plane_half_siz, -20, plane_half_siz),
								   vec3( plane_half_siz, -20,-plane_half_siz),
								   vec3(-plane_half_siz, -20,-plane_half_siz),
								   vec3(-plane_half_siz, -20, plane_half_siz)
								    );
	rec->load_texture( "balloon.jpg");
	rec->name = "rec";
	rec->set_UV( 0, 0.0f, 0.0f );
	rec->set_UV( 1, 0.0f, 1.0f );
	rec->set_UV( 2, 1.0f, 1.0f );
	rec->set_UV( 3, 1.0f, 0.0f );
	rec->set_mass( 0.0 );




	/* example of creating sphere_t */
	ball = new sphere_t( vec3(10,20,0), 5, vec3(10,5,0 ) );
	ball->name = "ball";
	ball->load_texture("tennis.jpg");
	//ball->set_mass( 0.0 );



	/* example of creating sphere_t for evnrinment sphere */
	sky = new sphere_t(1000, vec3(0,0,0) );
	sky->collidable = false;
	//sky->load_texture("night.png");
	//sky->load_texture("city.jpg");
	//sky->load_texture("city2.jpg");
	//sky->load_texture("city4.jpg");
	//sky->load_texture("japan.jpg");
	sky->load_texture("hospital.jpg");
	//sky->load_texture("paris.jpg");
	//sky->load_texture("castle.jpg");
	//sky->load_texture("restaurant.jpg");
	//sky->load_texture("virginia.jpg");
	//sky->load_texture("venice.jpg");
	//sky->load_texture("mars.jpg");
	sky->set_color( 1.0, 1.0, 1.0 );
	sky->set_rot( DEG(-90), DEG(180), 0 );
	sky->set_nslice_nstack( 100, 100 );
	sky->set_mass( 0.0 );




	/* example of creating laparo_t which is an example of how to creating a custom tool */
	laparox = new laparo_t();
	laparox->load_from_file("model/laparo4color.obj", 0.2);
	laparox->set_rot( DEG(90), DEG(90), DEG(90) );
	laparox->load_texture("model/laparo4color_default_color.bmp");
	laparox->set_mass( 0.0 );


	/* example of creating laparo_t which is an example of how to creating a custom tool */
	laparoy = new laparo_t();
	laparoy->load_from_file("laparo4x.obj", 0.2);
	laparoy->set_rot( DEG(90), DEG(90), DEG(90) );
	laparoy->load_texture("metal2.jpg");
	laparoy->set_mass( 0.0 );







	/* example of creating laparo_t which is an example of how to creating a custom tool */
	laparox2 = new laparo_t();
	laparox2->load_from_file("model/laparo4color.obj", 0.2);
	laparox2->set_rot( DEG(90), DEG(90), DEG(90) );
	laparox2->load_texture("model/laparo4color_default_color.bmp");
	laparox2->set_mass( 0.0 );


	/* example of creating laparo_t which is an example of how to creating a custom tool */
	laparoy2 = new laparo_t();
	laparoy2->load_from_file("laparo4x.obj", 0.2);
	laparoy2->set_rot( DEG(90), DEG(90), DEG(90) );
	laparoy2->load_texture("metal2.jpg");
	laparoy2->set_mass( 0.0 );



	seg = new segment_t( vec3(0.0,0.0,0.0), vec3(0.0,0.0,0.0), vec3(0.0,0.0,-100.0) );
	seg->name = "seg";
	seg->direc = false;
	seg->set_mass( 0.0 );



	seg2 = new segment_t( vec3(0.0,0.0,0.0), vec3(0.0,0.0,0.0), vec3(0.0,0.0,-100.0) );
	seg2->name = "seg2";
	seg2->direc = false;
	seg2->set_mass( 0.0 );


	/* example of creating ray_t */
	ray = new ray_t( vec3(0.0,0.0,-10.0), vec3(0.0,0.0,0.0), vec3(0.0,0.0,-10.0) );
	ray->name = "ray";
	ray->set_color( 1.0, 0.0, 0.0 );


	/* example of creating box_t */
	box = new box_t(25, 25, 25, vec3(-50, 20,-20) );
	box->set_rot( DEG(45), 0, 0 );
	box->load_texture("woodbox.jpg");
	box->set_UV_auto();
	box->name = "box";
	box->set_mass( 0.0 );


	/* example of creating Sneha's model */
	sneha = new wave_surface( vec3( 0.0, 10.0, 0.0) );
	sneha->set_mass( 0.0 );


	/* example of creating obj object using spring-mass with triangle as a primitve */
	mario = new obj_spring_triangle();
	mario->load_from_file("mario.obj");
	mario->load_texture("mario_main.png");
	mario->set_mass( 0.0 );


	/* example of creating obj object using spring-mass with triangle as a primitve */
	yoshi = new obj_rigid_triangle( vec3(0.0,0.0,50.0) );
	yoshi->set_rot( 0, DEG(135), 0 );
	yoshi->load_from_file("yoshi.obj", 5);
	yoshi->load_texture("yoshi.png");
	yoshi->set_mass( 0.0 );



	/* the example of using haptics holder with laparo_t class */
	haptics_left = new haptics_holder( laparox, laparoy, phantom_left_tran, &phantom_left_force, phantom_left_button );
	haptics_left->name = "hap left";



	haptics_right = new haptics_holder(  laparox2, laparoy2, phantom_right_tran, &phantom_right_force, phantom_right_button );
	haptics_right->name = "hap right";



//	world->add_object( seg );
//	world->add_object( tri );
//	world->add_object( rec );
//	world->add_object( ball );
	world->add_object( mycloth );
	world->add_object( sky );
//	world->add_object( rabbit );
//	world->add_object( box );
//	world->add_object( yoshi );
	//world->add_object( sneha );


	world->add_object( haptics_left );
	world->add_object( haptics_right );

	glutKeyboardFunc( keyboard_callback );
	glutSpecialFunc(  special_callback );
	glutDisplayFunc( display_callback );
	glutIdleFunc( idle_callback );
	glutMainLoop();
	return 0;
};



void display_callback()
{

};



double deg = 0;

void idle_callback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	update_camera();

	float dt = tt.toctoc();
	float loop_dt = dt;
	tt.tic();

	if( dt > 0.005)
		dt = 0.005;

	//stoke( 20 );
	//draw_axis(100); /* draw XYZ axis for  debugging */


	world->clear_all_force();


	world->add_enviroment_force( vec3(0.0, GRAVITY_Y, 0.0) );
	world->solve_collision_force();
	world->simulate( dt );
	world->render_scene();

	glutSwapBuffers();
};
