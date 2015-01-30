/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA UTILITIES
 *
 * CAMBADA UTILITIES is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA UTILITIES is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _BALLKALMAN_
#define _BALLKALMAN_

#include "Vec.h"
#include <sys/time.h>

namespace cambada {
namespace util {

/*! Filters a position based on a kalman filter. Built specifically for ball position, the model is hardcoded to minimize running time
\brief Kalman filter implementation*/
class BallPositionKalman
{
public:
	/*!Default constructor*/
	BallPositionKalman();
	
	/*!Main constructor
	\param readingDeviation standard deviation of the measures*/
	BallPositionKalman( double readingDeviation );
	
	/*!Class destructor*/
	~BallPositionKalman();
	
	/*!Method to set the measure standard deviation to the desired value
	\param readingDeviation standard deviation of the measures*/
	void setNoise( double readingDeviation );
	
	/*!Method to apply the filter to a measured position
	\param readPosition absolute position of the read sample
	\param instant time instant of the read sample
	\return The absolute position after the filter has been applied*/
	geom::Vec filterPosition( geom::Vec readPosition, unsigned long instant );
	
	/*!Method to restart the kalman filter internal variables
	\param setAsLast initial aproximation to the filter*/
	void resetFilter( geom::Vec setAsLast, unsigned long instant );
	
	/*!Method to set the ball as not visible, in the current cycle (used for reset control)*/
	void setNotVisible();
	
	/*!Tests for a hard deviation between the measures and the estimation, to indicate direction changes
	\return True if a hard deviation has occured*/
	bool hardDeviation();

private:
	geom::Vec lastMeasure;					/*!<Last position estimation of the filter (updated by \link filterPosition \endlink).*/
	geom::Vec lastVel;						/*!<Last velocity estimation of the filter (updated by \link filterPosition \endlink).*/

	bool lastCycleVisible;				/*!<An indication wheter or not the ball was visible on the last cycle (for reset purposes when the ball has been unavailable).*/
	unsigned long lastTime;				/*!<The last time an update to was made to the filter state.*/
	
	static int hardDeviationCount;		/*!<Counter for keeping the number of hard deviations found (for reset purposes).*/

	double readingDeviation;			/*!<The deviation of the measurements (vision sensor error, for sensor model).*/

	double R;				//R = readingDeviation^2
	double Q[2][2];			//Q = (readingDeviation/WHATEVER)^2 in main diagonal

	double S;					//covariance innovation
	double covarMatrix[2][2];	//P
	double modelMatrix[2][2];	//F
	double kalmanGain[2];		//K
};

}}

#endif
