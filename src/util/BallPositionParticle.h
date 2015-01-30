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

#ifndef _BALL_PARTICLE_
#define _BALL_PARTICLE_

#include "Vec.h"
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "MersenneTwister.h"

using namespace std;

namespace cambada {
namespace util {

/*! Filters a position based on a particle filter. Built specifically for ball position, the model is hardcoded to minimize running time.
\brief Particle filter implementation.*/
class BallPositionParticle
{
public:
	/*!Default constructor.*/
	BallPositionParticle();
	
	/*!Main constructor.
	\param readingDeviation standard deviation of the measures.
	\param squareBase the base to define the number of particles to use. <b>Optional</b>.*/
	BallPositionParticle( double readingDeviation, int squareBase=10 );
	
	/*!Class destructor.*/
	~BallPositionParticle();
	
	/*!Method to set the measure standard deviation to the desired value.
	\param readingDeviation standard deviation of the measures.*/
	void setNoise( double readingDeviation );
	
	/*!Method to set the number of particles to use in the filter.
	\param squareBase the number of particles is defined as the square of this input value.*/
	void setMParticles( int squareBase );
	
	/*!Method to create an initial set of particles, spread equally across the field and with zero velocity.*/
	void createInitialSet();
	
	/*!Method to create a set of particles around a given visual position.
	\warning The current deviation attribute is used to spread the particles. The user must setNoise if he wants to use the noise of the parameter position.
	\param initialPosition the reference position of the ball, usually the one measured visually.*/
	void createVisualSet( geom::Vec initialPosition );
	
	/*!Method to update the filter state given a measured position and the respective time instant.
	<b>The read position parameter is optional. If it is not passed, the filter update is the prediction.</b>
	\param instant time instant of the read sample, <b>IN MILISECONDS</b>.
	\param readPosition absolute position of the read sample. <b> Optional.</b>*/
	void updateFilter( unsigned long instant, geom::Vec readPosition = geom::Vec(-1000.0,-1000.0) );
	
	/*!Method to restart the particle filter internal variables
	\param initialPosition reference position used for creating an initial set of particles.*/
	void resetFilter( geom::Vec initialPosition );
	
	/*!Method to set the ball as not visible, in the current cycle (used for reset control)*/
	void setNotVisible();
	
	/*!Method to get the current position estimation.
	\return The current position estimation.*/
	geom::Vec getFilterPosition();
	
	/*!Method to get the current velocity estimation.
	\return The current velocity estimation.*/
	geom::Vec getFilterVelocity();
	
	/*!Method to return the current position particles of the filter
	\return A vector with the current position particles*/ 
	vector<geom::Vec> getPositionParticles();
	
	/*!Method to return the current velocity particles of the filter
	\return A vector with the current velocity particles*/ 
	vector<geom::Vec> getVelocityParticles();

//////////////////// TODO STILL KALMAN METHODS - TO IMPLEMENT //////////////////
	
	
	
	/*!Tests for a hard deviation between the measures and the estimation, to indicate direction changes
	\return true if a hard deviation has occured*/
	bool hardDeviation();

private:
	geom::Vec lastPosition;				/*!<Last position estimation of the filter (updated by \link updateFilter \endlink).*/
	geom::Vec lastVelocity;				/*!<Last velocity estimation of the filter (updated by \link updateFilter \endlink).*/
	
	vector<geom::Vec> positionParticles;/*!<Current internal particles concerning the ball position.*/
	vector<geom::Vec> velocityParticles;/*!<Current internal particles concerning the ball velocity.*/
	vector<double> particlesWeight;		/*!<Weights of each of the current internal particles*/

	bool lastCycleVisible;				/*!<An indication wheter or not the ball was visible on the last cycle (for reset purposes when the ball has been unavailable).*/
	unsigned long lastTime;				/*!<The last time an update to was made to the filter state.*/
	geom::Vec lastMeasure;				/*!<Keep the last position measure used by the filter, for velocity "measure" estimation.*/
	
	static int hardDeviationCount;		/*!<Counter for keeping the number of hard deviations found (for reset purposes).*/

	double readingDeviation;			/*!<The deviation of the position measurements (vision sensor error, for sensor model).*/
	double velocityDeviation;			/*!<The deviation of the velocity measurements.*/
	unsigned int M;						/*!<The number of particles to use on the filter.*/
};

}}

#endif
