/*
 * tictoc.hpp
 *
 *  Created on: Dec 18, 2012
 *      Author: theparitt
 */
#ifndef TICTOC_HPP
#define TICTOC_HPP


#include <iostream>

#include <ctime>

#ifdef _WIN32
#include <sys\timeb.h> 
#endif



using namespace std;


class tictoc
{
	clock_t start, stop;

public:
	tictoc()
		:start(0), stop(0)
	{
	}

	void tic()
	{
		start = clock();
	}



	//return in sec.milli
	float toctoc()
	{
		stop = clock();
		return float( stop - start ) / CLOCKS_PER_SEC;
	}



	void toc(const char* text="")
	{
		stop = clock();
		std::cout << text << (stop - start) / (double)CLOCKS_PER_SEC << " sec" << std::endl;
	}
};


static tictoc tt;



#endif

