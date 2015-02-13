/*----------------------------------------------------------------
//
//     ▄▄▄▄▀ ▄  █ ▄███▄   █ ▄▄  ██   █▄▄▄▄ ▄█    ▄▄▄▄▀ ▄▄▄▄▀
//  ▀▀▀ █   █   █ █▀   ▀  █   █ █ █  █  ▄▀ ██ ▀▀▀ █ ▀▀▀ █
//      █   ██▀▀█ ██▄▄    █▀▀▀  █▄▄█ █▀▀▌  ██     █     █
//     █    █   █ █▄   ▄▀ █     █  █ █  █  ▐█    █     █
//    ▀        █  ▀███▀    █       █   █    ▐   ▀     ▀
//            ▀             ▀     █   ▀
----------------------------------------------------------------*/



#ifndef MODEL_HPP_
#define MODEL_HPP_


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




/**
 *  model_t is an abstract class with already-define ODE solver
 */

class model_t:  public deriv_functor
{

protected:


public:

	ode_solver* solver;
	size_t state_siz;
	state_t state_curr;
	state_t state_next;


	/**
	 *
	 * @param state_siz	number of state that use in this model
	 */
	model_t(size_t state_siz)
		:solver(NULL), state_siz(state_siz), state_curr( state_siz ), state_next( state_siz )
	{
		assert( state_siz > 0);
		this->solver = rk4_solver::instance( state_siz );   /* set solver to rk4 */
		//this->solver = new rk45_solver( state_siz );      /* set solver to rk45 */
		this->solver->set_deriv_callback( this );
	}



	virtual ~model_t()
	{
		delete solver;
	}


	/**
	 * copy state from state_next ( next state ) to state_curr ( current state )
	 */
	virtual void update_state()
	{
		memcpy(state_curr.data, state_next.data, sizeof(state_next.data) );
	}


	/**
	 * simulate the object with force and time
	 * before calling this function all FORCE parameter in state_curr must be stated.
	 * @param dt  delta_time
	 */
	virtual void step_simulation(float dt )=0;
};






#endif /* MODEL_HPP_ */
















