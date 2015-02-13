/*
 * myexample.hpp
 *
 *  Created on: Feb 3, 2015
 *      Author: Imagelayer
 */

#ifndef MYEXAMPLE_HPP_
#define MYEXAMPLE_HPP_


#include <iostream>
#include "mygl.hpp"


#include "primitive.hpp"
#include "collision.hpp"
#include "tictoc.hpp"




class laparo_t: public obj_rigid_triangle
{

protected:



	virtual void draw()
	{
		/*------------------*/
		/*  debug activev   */
		/*------------------*/
//		for(auto iter=activev.begin(); iter<activev.end(); ++iter)
//		{
//			(*iter)->render_scene();
//		}

		rigid_triangle::draw();
	}



public:

	std::vector<primitive_base*> activev;



	laparo_t()
	{
		double t = 0.35;

		segment_t* seg = new segment_t( vec3(0.0,0.0,-40.0),vec3(0.0,0.0,40.0), vec3(0.0, t*3.0, t*-12.0), this);
		seg->direc = false;
		activev.push_back( seg );

		seg = new segment_t( vec3(0.0,0.0,-40.0),vec3(0.0,0.0,40.0), vec3(0.0, t*-3.0, t*-12.0), this);
		seg->direc = false;
		activev.push_back( seg );
	}



	virtual bool get_active_collision(std::vector<primitive_base*>& objv )
	{
		objv.insert( objv.end(), activev.begin(), activev.end() );
		return true;
	}


	virtual bool get_passive_collision(std::vector<primitive_base*>& objv)
	{
		return true;
	}


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
		is_colli = false;
	}

};









#endif /* MYEXAMPLE_HPP_ */
