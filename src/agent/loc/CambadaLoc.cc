/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA AGENT
 *
 * CAMBADA AGENT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA AGENT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "log.h"
#include "FieldLUT.h"
#include "CambadaLoc.h"
#include "random.h"
#include "VisionInfo.h"
#include <time.h>
#include <sys/time.h>
#include <cmath>
#include <fstream>


using namespace std;

namespace cambada {
namespace loc {


inline double hoch6(double x)
{
	double y = x * x * x;
	return y * y;
}

CambadaLoc::CambadaLoc(ConfigXML* config)
{
	field_lut = new FieldLUT ( config , 50 );
	double err_width = 250;
	double dist_param = 1e4; // default: practically weighting resembles independently of distance 
	
	ref_error = 1e6;
	latest_error = 1e6;
  
	vis_optimiser = new VisualPositionOptimiser (*field_lut, err_width, dist_param);

	robot_pos.x = 0;
	robot_pos.y = 0;
	robot_heading.set_rad(0.0);

	cfield_length = config->getField("field_length");
	cfield_width = config->getField("field_width");
	cside_band_width = config->getField("side_band_width");
	cgoal_band_width = config->getField("goal_band_width");
	
	
  	double max_x = 0.5 * cfield_width + cside_band_width;
  	double max_y = 0.5 * cfield_length + cgoal_band_width;
	
	struct timeval tv;
	gettimeofday(&tv, NULL);
	random_seed(tv.tv_sec);
	kalman_filter.set ( Vec(urandom(-max_x, max_x), urandom(-max_y, max_y)), Angle::zero, Vec(1e10, 1e10), 400 );
}


CambadaLoc::~CambadaLoc()
{
	delete field_lut;
	delete vis_optimiser;
}


double CambadaLoc::UpdateRobotPosition (vector< Vec >& lines)
{
	if (lines.size() > 5) {
		vis_optimiser->calculate_distance_weights (lines, lines.size());
			return vis_optimiser->optimise (robot_pos, robot_heading, lines, lines.size() > 20 ? 10 : 20);
	} else
		return 1e3;
}

double CambadaLoc::GetRobotPosition (Vec &pos, Angle &heading)
{
	pos = robot_pos;
	heading = robot_heading;
	return robot_pos_quality;
}


void CambadaLoc::SetRobotPosition(Vec p, Angle h)
{
	robot_pos = p;
	robot_heading = h;
	kalman_filter.set(robot_pos, robot_heading, Vec(1e10, 1e10), 400);	// Initialize Kalman Filter
}

double CambadaLoc::FindInitialPosition( vector< Vec >& lines, int fieldHalf )
{
	Vec offsets[] = { Vec(400, 0), Vec(0, 400), Vec(-400, 0), Vec(0, -400)};
	struct Pos {
		Vec pos;
		Angle heading;
		double error;
	};

	Pos array[4];
	int j = 0;
	
  	double max_x = 0.5 * cfield_width + cside_band_width;
  	double max_y = 0.5 * cfield_length + cgoal_band_width;

	double err;
	double best_error = 1e6;
	int i = 0;
	do 
	{
		if(fieldHalf == MY_HALF)
			robot_pos = Vec( urandom(-max_x, max_x), urandom(-max_y, 0) );
		else
			robot_pos = Vec( urandom(-max_x, max_x), urandom(0, max_y) );

		//robot_heading.set_rad( urandom(0, M_2_PI) );
		robot_heading.set_rad( urandom(-M_PI, M_PI) );
		
		err =  UpdateRobotPosition (lines);
		if( err < best_error )
		{
			best_error = err;
			array[j].pos = robot_pos;
			array[j].heading = robot_heading;
			array[j].error = err;
			++j;
			j = (j & 0x03);
		}
	} while ( (best_error > 3.5) && (++i < 2000) );
	
	for( j = 0; j < 4; j++)
	{
		for(int k = 0; k < 4; k++)
		{
			robot_pos = array[j].pos + offsets[k];
			for(int m = 0; m < 25; m++)	
			{
				robot_heading.set_rad( array[j].heading.get_rad() + urandom(-M_PI_4, M_PI_4) );
				
				err =  UpdateRobotPosition (lines);
				if(err < best_error) 
				{
					myprintf("Better :-)\n");
					best_error = err;
					array[j].pos = robot_pos;
					array[j].heading = robot_heading;
					array[j].error = err;
				}
			}
		}
	}
	j = 0;
	while(array[j].error != best_error) j++;

	robot_pos = array[j].pos;
	robot_heading = array[j].heading;
	err =  UpdateRobotPosition (lines);
	kalman_filter.set(robot_pos, robot_heading, Vec(1e10, 1e10), 400);	// Initialize Kalman Filter
	
	
	dq_pos.clear();
	for( unsigned int i = 0 ; i < 5 ; i++ )
		dq_pos.push_back(robot_pos);

	return err;
}


double CambadaLoc::FindInitialPositionWithKnownOrientation(vector< Vec >& lines, Angle orientation )
{
	Vec offsets[] = { Vec(400, 0), Vec(0, 400), Vec(-400, 0), Vec(0, -400)};
	struct Pos {
		Vec pos;
		Angle heading;
		double error;
	};

	Pos array[4];
	int j = 0;
	
  	double max_x = 0.5 * cfield_width + cside_band_width;
  	double max_y = 0.5 * cfield_length + cgoal_band_width;
	
	
	double err;
	double best_error = 1e6;


	for( double x = - max_x ; x < max_x + 501 ; x+=500 )
		for( double y = -max_y ; y < max_y + 501 ; y+=500 )
		{
			robot_pos = Vec( x , y);
			robot_heading = orientation;

			err =  UpdateRobotPosition (lines);
			if( err < best_error )
			{
				best_error = err;
				array[j].pos = robot_pos;
				array[j].heading = robot_heading;
				array[j].error = err;
				++j;
				j = (j & 0x03);
			}
		}

	for( j = 0; j < 4; j++)
	{
		for(int k = 0; k < 4; k++)
		{
			robot_pos = array[j].pos + offsets[k];
			robot_heading = array[j].heading;
			for(int m = 0; m < 10; m++) // 25	
			{
				//robot_heading.set_rad( array[j].heading.get_rad() + urandom(-M_PI_4, M_PI_4) );
				
				err =  UpdateRobotPosition (lines);
				if(err < best_error) 
				{
					myprintf("Better :-)\n");
					best_error = err;
					array[j].pos = robot_pos;
					array[j].heading = robot_heading;
					array[j].error = err;
				}
			}
		}
	}
	j = 0;
	while(array[j].error != best_error) j++;

	robot_pos = array[j].pos;
	robot_heading = array[j].heading;
	
	err =  UpdateRobotPosition (lines);
	kalman_filter.set(robot_pos, robot_heading, Vec(1e10, 1e10), 400);	// Initialize Kalman Filter
	
	kf.reset();
	Eigen::Vector3f measure(robot_pos.x, robot_pos.y, robot_heading.get_rad_pi());
	Eigen::Matrix3f Q;
	Q << 1e3, 0, 0,
		 0, 1e3, 0,
		 0, 0, 6.26;
	kf.update(measure, Q);

	dq_pos.clear();
	for( unsigned int i = 0 ; i < 5 ; i++ )
		dq_pos.push_back(robot_pos);

	return err;
}



double CambadaLoc::UpdateRobotPosition(vector< Vec >& lines,  double odo_deltax, double odo_deltay, double odo_deltaphi  )
{
	if(odo_deltax != 0.0 || odo_deltay != 0.0 || odo_deltaphi != 0.0){
		myprintf("\n\nDELTAS ODO ( %.2f , %.2f , %.2f )\n", odo_deltax, odo_deltay, odo_deltaphi);
	}

	Angle vis_heading = robot_heading;
	Vec vis_pos = robot_pos;
		
	Vec pos(0.0,0.0);

	if (lines.size() > 5) 
	{
		vis_optimiser->calculate_distance_weights (lines, lines.size());
		ref_error = vis_optimiser->optimise(vis_pos, vis_heading, lines, lines.size() > 20 ? 10 : 20 );
			
		dq_pos.push_back(vis_pos);
		dq_pos.pop_front();
		
		for( unsigned int i = 0 ; i < dq_pos.size() ; i++ )
			pos += ( dq_pos[i] );
	} 
	else
		ref_error = 1e6;
	
	robot_pos = (pos/dq_pos.size());
	robot_heading = vis_heading;

	return ref_error;
}

double CambadaLoc::UpdateRobotPosition_KF(vector< Vec >& lines, double odo_deltax, double odo_deltay, double odo_deltaphi  )
{


	double ddphi, var_phi;
	double fnum;
	Vec ddxy;

	Vec delta_pos( odo_deltax, odo_deltay );	// robot delta_movement obtained through odometry (in robot coordinates)

	Angle delta_heading( odo_deltaphi );		// robot rotation during movement (can be 0, as it is omni) 

	Vec old_pos = robot_pos;					// Last cumputed position
	Angle old_heading = robot_heading;			// Last computed heading

	Vec delta_pos_world = delta_pos.rotate( old_heading.get_rad());	// robot delta movement in absolute coordinates.
																// It is obtained through rotation of the vector  
																// by "robot_heading" (it can be seen as equivalent
																// to rotate the world cartesian system). For instance
																// if "robot_heading" is +30� (relative to YY) the
																// vector has to be rotated +30� in order to obtain
																// the absolute movement
	
	if (lines.size() > 5) 
	{
		Vec visual_pos = old_pos + delta_pos_world;				// new trial position in absolute coordinates
		Vec odometry_pos = visual_pos;
		Angle visual_heading = old_heading + delta_heading;		// new trial heading in absolute coordinates
		Angle odometry_heading = visual_heading;
		
		vis_optimiser->calculate_distance_weights (lines, lines.size());

		ref_error = vis_optimiser->optimise( visual_pos, visual_heading, lines, lines.size() > 20 ? 10 : 20 );

		Vec offsets[]={ 
				Vec( 500.0,   0.0),
				Vec(-500.0,   0.0),
				Vec(   0.0, 500.0),
				Vec(   0.0,-500.0)
		};

		for (int i=0; i< 4; i++) {
				Vec trial_pos = odometry_pos+offsets[i];
				Angle trial_heading = odometry_heading + 10.0/180.0*M_PI * urandom(-1.0,1.0);

				double err = vis_optimiser->optimise( trial_pos, trial_heading, lines, lines.size() > 20 ? 10 : 20 );

				if (err < ref_error*0.9) {
						visual_pos = trial_pos;
						visual_heading = trial_heading;
						ref_error= err;
				}
		}
		
		latest_error = vis_optimiser->analyse( ddxy, ddphi, visual_pos, visual_heading, lines );
		// Derive variances from the curvatures (heuristic formula); to avoid 1e-6 around numeric singularity; 
		// trans_world.x^2 and/or trans_world.y^2, in order to model a larger Ungtenauigkeit 
		// transverse to the driving direction (swings of the robot)

		fnum = 16.0 / (lines.size() + 4.0) + 0.7;   // Factor, in order to consider the number of seen lines
		Vec var_xy (delta_pos_world.y * delta_pos_world.y + fnum * 225 * hoch6( log( abs( ddxy.x + 1e-6) ) + 7 ), \
					delta_pos_world.x * delta_pos_world.x + fnum * 225 * hoch6( log( abs( ddxy.y ) + 1e-6 ) + 7) ); 
   
		var_phi = fnum * 3.0461741978670859865 / ( ddphi * ddphi + 1 );  // dito

		kalman_filter.update (delta_pos_world, delta_heading, visual_pos, visual_heading, var_xy, var_phi);

		Eigen::Vector3f control(delta_pos_world.x, delta_pos_world.y, delta_heading.get_rad_pi());
		Eigen::Vector3f measure(visual_pos.x, visual_pos.y, visual_heading.get_rad_pi());

		Eigen::Matrix3f R; // Process noise
		R << fabs(0.3*delta_pos_world.x) + 50, 0, 0, // +50 FIXME (after robotica 2012
			 0, fabs(0.3*delta_pos_world.y) + 50, 0, // +50 --
			 0, 0, fabs(0.1*delta_heading.get_rad_pi()) + 0.05 + 0;

		Eigen::Matrix3f Q; // Measurement noise
		Q << latest_error*10+200, 0, 0,
			 0, latest_error*10+200, 0,
			 0, 0, 0.01*latest_error+0.05;
	/*	Q << 1e5, 0, 0,
			 0, 1e5, 0,
			 0, 0, 1e5;
*/
		//fprintf(stderr,"CHALLENGE %.2f\n",delta_heading.get_rad_pi());

		kf.filter(control, measure, R, Q);
	} 
	else
	{
		myprintf("DEBUG: Update_KF\nNot enough visual information\n");
		kalman_filter.update (delta_pos_world, delta_heading, false);	// Not enough visual information
		Eigen::Vector3f control(delta_pos.x, delta_pos.y, delta_heading.get_rad_pi());
		Eigen::Matrix3f R;
		R << 50, 0, 0,
			 0, 50, 0,
			 0, 0, 0.06;
		kf.predict(control, R);
	}
	robot_pos_quality = kalman_filter.get( robot_pos, robot_heading );	// Update robot coordinates and confidence degree (0: excelent / 1: bad)

	Eigen::Vector3f pose;
	kf.get(pose);
	robot_pos.x = pose(0);
	robot_pos.y = pose(1);
	robot_heading = pose(2);

	return ref_error;
}

void CambadaLoc::mirror()
{
	robot_pos.x = -robot_pos.x;
	robot_pos.y = -robot_pos.y;
	robot_heading += Angle(M_PI);
	Eigen::Vector3f mirrorState;
	mirrorState << (float)robot_pos.x, (float)robot_pos.y, robot_heading.get_rad_pi();
	Eigen::Matrix3f mirrorCovar;
	mirrorCovar << 1e3, 0, 0,
				   0, 1e3, 0,
				   0, 0, 6.28;
	kf.set(mirrorState, mirrorCovar);
}

}
}

