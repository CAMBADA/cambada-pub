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

#ifndef _VELOCITYREGRESSION_H_
#define _VELOCITYREGRESSION_H_

#include "VisionInfo.h"
#include "WorldState.h"
#include "WorldStateDefs.h"
#include "Vec.h"
#include "sys/time.h"
#include <deque>

namespace cambada {
namespace util {

/*! Calculates a linear regression, given position and time values, used as an aproximation to speed calculus. It has methods to deal with three components of the speed: linear XX component, linear YY component and angular component
\brief Linear regression algorithm applied to CAMBADA velocity estimation*/
class VelocityRegression
{
private:
	unsigned int MAXSIZE;				/*!<maximum size of the buffers.*/
	struct timeval instant;				/*!<current cycle time instant.*/
	deque<Vec> posBuffer;				/*!<buffer of object position.*/
	deque<float> oriBuffer;				/*!<buffer of object orientation.*/
	deque<struct timeval> timeBuffer;	/*!<buffer of time instants.*/
	float lastXcommand;					/*!<The last command sent to the robot for the X linear velocity component (desired velocity).*/
	float lastYcommand;					/*!<The last command sent to the robot for the Y linear velocity component (desired velocity).*/
	float lastAcommand;					/*!<The last command sent to the robot for the angular velocity component (desired velocity).*/

public:
	/*! Default constructor*/
	VelocityRegression();

	/*! Main constructor
	\param maxSize maximum size for the buffers*/
	VelocityRegression( int maxSize );

	/*! Class destructor*/
	~VelocityRegression();

	/*! Method to insert new values in the buffers
	\param pos position parameter value, with type Vec(x,y) (Y in linear velocity calculations).
	\param ori orientation parameter value, of type float, normalized in [0,2*pi] (Y in angular velocity calculations).
	\param instantIn time instant parameter value, with type struct timeval (X in calculations).*/
	void putNewValues( Vec pos, float ori, struct timeval instantIn);

	/*! Method to clear the buffers*/
	void clearBuffs();

	/*! Method to calculate the slope, based on the linear regression of the current position buffer values
	\return Vec with the current slope value (speed)*/
	geom::Vec getLinearVelocity();
	
	/*! Method to calculate the slope, based on the linear regression of the current orientation buffer values
	\return Vec with the current slope value (speed)*/
	float getAngularVelocity();

	/*! Method to calculate the point in origin, based on the linear regression of the current buffer values
	\return Vec with the value of the point where the derivative intercepts YY axe*/
	geom::Vec getPointInOrig();

	/*! Method to print the values in the position buffer*/
	void printPosBuff();

	/*! Method to print the values in the orientation buffer*/
	void printOriBuff();

	/*! Method to print the values in the time buffer*/
	void printTimeBuff();

	/*!Method to reset the internal time, position and orientation buffers to the last N given samples
	\param nSamples number of samples to keep in the buffers*/
	void resetToNSamples( unsigned int nSamples );

	/*!Method to optimize the new velocity determination after a collision (based on prediction)
	\param t instant of collision
	\param collisionPos prediction ball position at collision instant
	\param newVel predicted velocity after collision*/
	void collision(float t, geom::Vec collisionPos, geom::Vec newVel);

	void checkAngleDiscontinuity(float newAngle);
};

}}

#endif
