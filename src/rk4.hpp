/*--------------------------------------------------------------------------------
//
//
//  ▄▄▄█████▓ ██░ ██ ▓█████  ██▓███   ▄▄▄       ██▀███   ██▓▄▄▄█████▓▄▄▄█████▓
//  ▓  ██▒ ▓▒▓██░ ██▒▓█   ▀ ▓██░  ██▒▒████▄    ▓██ ▒ ██▒▓██▒▓  ██▒ ▓▒▓  ██▒ ▓▒
//  ▒ ▓██░ ▒░▒██▀▀██░▒███   ▓██░ ██▓▒▒██  ▀█▄  ▓██ ░▄█ ▒▒██▒▒ ▓██░ ▒░▒ ▓██░ ▒░
//  ░ ▓██▓ ░ ░▓█ ░██ ▒▓█  ▄ ▒██▄█▓▒ ▒░██▄▄▄▄██ ▒██▀▀█▄  ░██░░ ▓██▓ ░ ░ ▓██▓ ░
//    ▒██▒ ░ ░▓█▒░██▓░▒████▒▒██▒ ░  ░ ▓█   ▓██▒░██▓ ▒██▒░██░  ▒██▒ ░   ▒██▒ ░
//    ▒ ░░    ▒ ░░▒░▒░░ ▒░ ░▒▓▒░ ░  ░ ▒▒   ▓▒█░░ ▒▓ ░▒▓░░▓    ▒ ░░     ▒ ░░
//      ░     ▒ ░▒░ ░ ░ ░  ░░▒ ░       ▒   ▒▒ ░  ░▒ ░ ▒░ ▒ ░    ░        ░
//    ░       ░  ░░ ░   ░   ░░         ░   ▒     ░░   ░  ▒ ░  ░        ░
//            ░  ░  ░   ░  ░               ░  ░   ░      ░
//
----------------------------------------------------------------------------------*/






#include <iostream>
#include <cstring>
#include <cstddef>
#include <cassert>
#include <vector>


float EPSILON = 0.0001;
const size_t STATE_SIZ_MAX = 21;




class state_t
{

protected:
	size_t siz;

public:
	float data[STATE_SIZ_MAX]; /* state variable */

	state_t(size_t size)
		:siz( size )
	{
		assert( size > 0 );
		assert( size<STATE_SIZ_MAX );
		memset( data, 0, sizeof(data) );
	}


	virtual ~state_t()
	{
	}


	size_t size()
	{
		return siz;
	}


	float& operator[](size_t i)
	{
		assert( i < siz );
		return data[i];
	}


	float operator[](size_t i) const
	{
		assert( i < siz );
		return data[i];
	}


	void debug(const char* s="", const char* e="")
	{
		printf("%s", s);
		for(size_t i=0; i<siz; i++)
			printf("%.15f ", data[i] );
		printf("%s", e);
	}

};





class deriv_functor
{

public:

	virtual ~deriv_functor()
	{
	}

	virtual void operator()(float t, const state_t& X, state_t& dX) = 0;
};



class ode_solver
{

protected:


public:
	size_t siz;
	deriv_functor* func;


	ode_solver(int state_siz)
		:siz(state_siz), func(NULL)
	{

	}


	void set_deriv_callback(deriv_functor* func)
	{
		this->func = func;
	}


	virtual~ ode_solver()
	{

	}


	/**
	 * @param[in] t			time
	 * @param[in] h			delta time
	 * @param[in] X			current state
	 * @param[out] X_next	next state output at (t+h)
	 */
	virtual void solve(float t, float h, const state_t& X, state_t& X_next) = 0;


	//  F: derivative function
	//  X: current state
	// dX: output state when time at t
	void F( float t, const state_t& X, state_t& dX)
	{
		assert( func != NULL );
		(*func)( t, X, dX );
	}


	//the output will be dV
	virtual state_t hF(const state_t& X0, float t, float h)
	{
		state_t dX_next(siz);

		F( t, X0, dX_next );
		//dX = h * F()
		for(size_t i=0; i<siz; i++)
			dX_next[i] *= h;

		return dX_next;
	}

};




/**
 * runge-kutta order4 ode solver
 * using singleton pattern
 */
class rk4_solver: public ode_solver
{

	static rk4_solver* myself;

	//size - state size
	rk4_solver(size_t state_siz)
		:ode_solver( state_siz )
	{
		func = NULL;
	}


public:

	/* singleton style */
	static rk4_solver* instance(size_t siz)
	{
		if( myself == NULL )
			myself = new rk4_solver( siz );
		else
			assert( myself->siz == siz );

		return myself;
	}


	virtual ~rk4_solver()
	{

	}


	virtual void solve(float t, float h, const state_t& X,  state_t& X_next)
	{
		assert( siz > 0 );

		static state_t delta0(siz);
		static state_t deltaA(siz), deltaB(siz), deltaC(siz), deltaD(siz);
		state_t deltaA_dat(siz), deltaB_dat(siz), deltaC_dat(siz), deltaD_dat(siz);

		deltaA = hF(         X, t, h       );
		for(size_t i=0; i<siz; i++)
			deltaA_dat[i] = X[i] + deltaA[i]/2.0;

		deltaB = hF(deltaA_dat, t+h/2.0, h   );
		for(size_t i=0; i<siz; i++)
			deltaB_dat[i] = X[i] + deltaB[i]/2.0;

		deltaC = hF(deltaB_dat, t+h/2.0, h   );
		for(size_t i=0; i<siz; i++)
			deltaC_dat[i] = X[i] + deltaC[i];

		deltaD = hF(deltaC_dat, t+h  , h   );

		//--- update state: X(t+dt) = X(t) + 1/6 (A + 1/2*B + 1/2*C + D) ---//
		for(size_t i=0; i<siz; i++)
		{
			X_next[i] = X[i] + ( deltaA[i] + 2.0*deltaB[i] + 2.0*deltaC[i] + deltaD[i] )/6.0 ;
		}


	}

};


rk4_solver* rk4_solver::myself = NULL;





class rk45_solver: public ode_solver
{

public:


	//size - state size
	rk45_solver(size_t state_siz)
		:ode_solver( state_siz )
	{
		func = NULL;
	}


	virtual ~rk45_solver()
	{

	}



	//call this cuntion
	virtual void solve(float t, float h, const state_t& X, state_t& X_next)
	{
		int  recomp = 0;
		state_t delta0(siz);
		state_t k1(siz), k2(siz), k3(siz), k4(siz), k5(siz), k6(siz);
		state_t k1_dat(siz), k2_dat(siz), k3_dat(siz), k4_dat(siz), k5_dat(siz);

RECALCULATE:

		assert( recomp <= 5);
		k1 = hF(         X, t, h       );
		for(int i=0; i<siz; i++)
			k1_dat[i] = X[i] + k1[i]/4;

		k2 = hF(k1_dat, t+h/4, h   );
		for(int i=0; i<siz; i++)
			k2_dat[i] = X[i] + (3*k1[i] + 9*k2[i])/32;

		k3 = hF(k2_dat, t+3*h/8, h   );
		for(int i=0; i<siz; i++)
			k3_dat[i] = X[i] + (1932*k1[i] - 7200*k2[i] + 7296*k3[i])/2197;

		k4 = hF(k3_dat, t+12*h/13, h   );
		for(int i=0; i<siz; i++)
			k4_dat[i] = X[i] + 439*k1[i]/216 - 8*k2[i] + 3680*k3[i]/513 - 845*k4[i]/4104;

		k5 = hF(k4_dat, t+h, h   );
		for(int i=0; i<siz; i++)
			k5_dat[i] = X[i] - 8*k1[i]/27 + 2*k2[i] - 3544*k3[i]/2565 + 1859*k4[i]/4104 - 11*k5[i]/40;

		k6 = hF(k5_dat, t+h/2, h   );


		//update state: X(t+dt) = X(t) + 1/6 (A + 1/2*B + 1/2*C + D)
		state_t X_dat(siz);
		for(int i=0; i<siz; i++)
		{
			X_dat[i]  = X[i] + 16*k1[i]/135 + 6656*k3[i]/12825 + 28561*k4[i]/56430 - 9*k5[i]/50 + 2*k6[i]/55;
			X_next[i] = X[i] + 25*k1[i]/216 + 1408*k3[i]/2565  +  2197*k4[i]/4104  -   k5[i]/5;
		}


		float R = 0.0f;
		for(int i=0; i<siz; i++)
		{
			R += abs(X_dat[i] - X_next[i])/h;
		}

//		if( R > EPSILON)
//		{
//
//			float gamma = 0.84f * (EPSILON / (R * siz), 0.25);
//			h = gamma * h;
//
//			fprintf(stderr, "error > EPSILON\n h: %f  h new: %f\n", h/gamma, h);
//			fprintf(stderr, "R %f  gamma: %f\n", R, gamma );
//
//			recomp++;
//			goto RECALCULATE;
//		}
	}

};







////--------------------------//
////  example of how to use   //
////--------------------------//
//void dV_callback( float t, state_t& X, state_t& dX)
//{
//	size_t siz = dX.siz;
//
//	//-- equation for the equation order //
//	dX[siz-1] = X[0] - t*t + 1;
//}
//
//
//int main(int argc, char** argv)
//{
//	float t0 = 0.0;
//	float t1 = 2;
//	float tstep = 0.2;
//
//	state_t X_init(1);
//
//	X_init[0] = 0.5;
//
//	rk4_solver  rk4(1);
//	
//	rk4.set_callback( &dV_callback );
//	//-----------------------//
//	//          rk4          //
//	//-----------------------//
//	
//	state_t A(1);
//	A[0] = X_init[0];
//	printf("t: %f \n", t0);
//	rk4.solve(t0, 0, A);   //@t0 tstep = 0
//	A.debug("");
//	printf("\n======\n");
//	for(float t=t0; t<t1; t+=tstep)
//	{
//		printf("t: %f \n", t+tstep);
//
//		rk4.solve(t, tstep, A);
//		A.debug("");
//		printf("\n======\n");
//	}
//
//	getchar();
//
//	return 0;
//}
