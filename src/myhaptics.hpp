/*
 * myhaptics.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: Imagelayer
 */

#ifndef MYHAPTICS_HPP_
#define MYHAPTICS_HPP_



#ifdef HAPTICS
/*
 *
 *
 *        HAPTICS
 *
 *
 */


#include <HL/hl.h>
#include <HLU/hlu.h>

#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>



#define DEVICE_NAME_1 "PHANTOM_LEFT"
#define DEVICE_NAME_2 "PHANTOM_RIGHT"


HHD phantom_left;
HHD phantom_right;

HDSchedulerHandle hdSchedule = HD_INVALID_HANDLE;


HDdouble     phantom_left_tran[16];
hduVector3Dd phantom_left_force;
int          phantom_left_button;

HDdouble     phantom_right_tran[16];
hduVector3Dd phantom_right_force;
int          phantom_right_button;


HDCallbackCode HDCALLBACK haptics_callback(void *data)
{
	hduVector3Dd hvec3;

	hdBeginFrame(phantom_left);
	hdGetDoublev( HD_CURRENT_TRANSFORM, phantom_left_tran );
	hdSetDoublev(HD_CURRENT_FORCE, phantom_left_force);
    hdGetIntegerv(HD_CURRENT_BUTTONS, &phantom_left_button);
	hdEndFrame(phantom_left);


	hdBeginFrame(phantom_right);
	hdGetDoublev( HD_CURRENT_TRANSFORM, phantom_right_tran );
	hdMakeCurrentDevice(phantom_right);
	hdSetDoublev(HD_CURRENT_FORCE, phantom_right_force);
	hdGetIntegerv(HD_CURRENT_BUTTONS, &phantom_right_button);
	hdEndFrame(phantom_right);




	sneha_depth = phantom_left_tran[13];




	return HD_CALLBACK_CONTINUE;
}



void init_haptics()
{
	HDErrorInfo error;

	phantom_left = hdInitDevice("PHANTOM_LEFT");
	if( HD_DEVICE_ERROR(error = hdGetError()) )
	{
		fprintf(stderr,"Failed to initialize haptic device (left) \nPress any key to quit.\n");
		getchar();
	}

	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);

	phantom_right = hdInitDevice("PHANTOM_RIGHT");
	if( HD_DEVICE_ERROR(error = hdGetError()) )
	{
		fprintf(stderr,"Failed to initialize haptic device (right) \nPress any key to quit.\n");
		getchar();
	}

	printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE)); fflush( stdout );
	hdSchedule = hdScheduleAsynchronous(haptics_callback, 0, HD_MAX_SCHEDULER_PRIORITY);
	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_FORCE_RAMPING);
	hdStartScheduler();

	/* Check for errors and abort if so. */
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		fprintf(stderr, "Failed to start scheduler\nPress any key to quit.\n");
		exit(-1);
	}

	printf(" haptics init complete\n"); fflush( stdout );
}




const int HAPTICS_ID = 100;
const double FORCE_RATIO = 0.2;

class haptics_holder: public primitive_base
{


protected:

	HDdouble* phantom_tran;

	primitive_base* obj;
	primitive_base* obj0;
	primitive_base* obj1;

	hduVector3Dd* phantom_force;
	int* phantom_button;

	vec3 force_prev;

	virtual void render_scene()
	{
		if( obj0 != NULL && obj1 != NULL )
		{
			obj = (*phantom_button)? obj0 : obj1;
		}

		assert( obj != NULL );
		obj->render_scene();
	}


	virtual void draw()
	{

	}

public:



	haptics_holder(primitive_base* obj, HDdouble* phantom_tran, hduVector3Dd* phantom_force, int& phantom_button)
		:obj(obj), obj0(NULL), obj1(NULL), phantom_tran( phantom_tran ), phantom_force(phantom_force), phantom_button(& phantom_button), primitive_base( HAPTICS_ID, vec3(0,0,0), NULL )
	{
		assert( obj != NULL );
	}


	haptics_holder(primitive_base* obj0, primitive_base* obj1, HDdouble* phantom_tran, hduVector3Dd* phantom_force, int& phantom_button)
		:obj(NULL), obj0(obj0), obj1(obj1), phantom_tran( phantom_tran ), phantom_force(phantom_force), phantom_button(& phantom_button), primitive_base( HAPTICS_ID, vec3(0,0,0), NULL )
	{
		assert( obj0 != NULL );
		assert( obj1 != NULL );
	}





	virtual void step_simulation(float dt)
	{
		if( obj0 != NULL && obj1 != NULL )
		{
			obj = (*phantom_button)? obj0 : obj1;
		}

		assert( obj != NULL );
		obj->set_pos( phantom_tran[12], phantom_tran[13], phantom_tran[14] );
		/* just make sub-object update according to new transform */
		obj->step_simulation(0.0);

		matrix_rot omni_rot;
		omni_rot(0,0) = phantom_tran[0];
		omni_rot(0,1) = phantom_tran[4];
		omni_rot(0,2) = phantom_tran[8];
		omni_rot(1,0) = phantom_tran[1];
		omni_rot(1,1) = phantom_tran[5];
		omni_rot(1,2) = phantom_tran[9];
		omni_rot(2,0) = phantom_tran[2];
		omni_rot(2,1) = phantom_tran[6];
		omni_rot(2,2) = phantom_tran[10];

		vec3 rot = omni_rot.get_roll_pitch_yaw();
		obj->state_of(X_ROT) = rot(0);
		obj->state_of(Y_ROT) = rot(1);
		obj->state_of(Z_ROT) = rot(2);

		obj->post_update();


		/*------------------*/
		/*   render force   */
		/*------------------*/
		if( !is_colli )
		{
			(*phantom_force)[0] = 0.0;
			(*phantom_force)[1] = 0.5;
			(*phantom_force)[2] = 0.0;

			force_prev( 0.0, 0.0, 0.0 );
			//printf("NOT HIT NOT HIT\n"); fflush( stdout );
			return;
		}


		vec3 force(0.0,0.0,0.0);
		std::vector<primitive_base*> activev;
		obj->get_active_collision( activev );
		int hit_count = 0;
		for( int i=0; i<activev.size(); i++)
		{
			if( !activev[i]->is_colli )
				continue;

			double dist = activev[i]->colli_depth;

			force += FORCE_RATIO*(dist)*activev[i]->get_collision_normal();

			hit_count++;
		}


		/* handle some weird behaviours which i have no time to fix */
		if( hit_count == 0)
		{
			(*phantom_force)[0] = force_prev[0];
			(*phantom_force)[1] = force_prev[1];
			(*phantom_force)[2] = force_prev[2];

			force_prev.debug("some shitty thing");
			printf("========in ERROR handle CASE=========\n"); fflush( stdout );
			return;
		}


		force.debug("before divide force: ");
		force /= hit_count;

		force += vec3( 0.0,   0.5     , 0.0 ); /* pan's mass compensation     */
		(*phantom_force)[0] = force[0];
		(*phantom_force)[1] = force[1];
		(*phantom_force)[2] = force[2];



		force_prev = force;
		//force.debug("output force: ");
		//printf("hit count %d\n", hit_count ); fflush( stdout );

	};


	virtual bool get_active_collision(std::vector<primitive_base*>& colliv)
	{
		if( obj0 != NULL && obj1 != NULL )
		{
			obj = (*phantom_button)? obj0 : obj1;
		}


		assert( obj != NULL );
		return obj->get_active_collision( colliv );
	}


	virtual bool get_passive_collision(std::vector<primitive_base*>& colliv)
	{
		if( obj0 != NULL && obj1 != NULL )
		{
			obj = (*phantom_button)? obj0 : obj1;
		}


		assert( obj != NULL );
		return obj->get_passive_collision( colliv );
	}


	virtual void clear_all_force()
	{
		object_base::clear_all_force();

		if( obj0 != NULL && obj1 != NULL )
		{
			obj0->clear_all_force();
			obj0->is_colli = false;

			obj1->clear_all_force();
			obj1->is_colli = false;
		}
		else
		{
			obj->clear_all_force();
			obj->is_colli = false;
		}

		is_colli = false;
	}


};























#endif /* HAPTICS */



#endif /* MYHAPTICS_HPP_ */
