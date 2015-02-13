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


#ifndef PENDULUM_MODEL_HPP_
#define PENDULUM_MODEL_HPP_




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


#define _USE_MATH_DEFINES
#include <cmath>

#include "model.hpp"




/*------------------------*/
/*         Pendulum       */
/*------------------------*/
const int STATE_SIZ= 2;
/*--- state variable ---*/
const int THETA = 0;
const int THETA_DAT  = 1;
/*-----  parameter  ----*/
const float g = 9.8;
const float R = 4; /* length of rod */
/*----- variable ------*/



/**
 * @class pendulum_model
 */
class pendulum_model: public model_t
{

protected:

	/**
	 *
	 * @param t time
	 * @param X
	 * @param dX
	 */
	virtual void operator()( float t, const state_t& X, state_t& dX )
	{
		dX[THETA]     = X[THETA_DAT];
		dX[THETA_DAT] = -g/R * sin( X[THETA] );
	}


public:

	/**
	 * the pendulum model is a model consist of 2 state, theta( circular position) and theta_dat (circular velocity)
	 */
	pendulum_model()
		:model_t(STATE_SIZ)
	{
		state_curr[THETA]     = M_PI/2; /* init with cicular position at PI/2 */
		state_curr[THETA_DAT] = 0;      /* init with cicular velocity equals to zero */
	}


	virtual void step_simulation(float dt )
	{
		solver->solve(0, dt, state_curr, state_next );
	}


	/**
	 * display sphere by its current state
	 */
	virtual void render_scene()
	{
		glPushMatrix();
			glTranslatef( R*cos(state_curr[THETA] - M_PI_2), R*sin(state_curr[THETA] - M_PI_2), 0.0f);

			std::cout << "pos" << R*cos(state_curr[THETA] - M_PI_2) << std::endl;
			glutSolidSphere(2, 20, 20);
		glPopMatrix();
	}

};










#endif /* PENDULUM_MODEL_HPP_ */
