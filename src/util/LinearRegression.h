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

#ifndef _LINEARREGRESSION_H_
#define _LINEARREGRESSION_H_

#include "VisionInfo.h"
#include "WorldState.h"
#include "WorldStateDefs.h"
#include "Vec.h"
#include "sys/time.h"
#include <deque>

namespace cambada {
namespace util {

/*! Calculates a linear regression, given position and time values, used as an aproximation to speed calculus. <b>INPUT POSITION VALUES MUST BE MILIMETERS</b>
\brief Linear regression algorithm*/
class LinearRegression
{
private:
	unsigned int MAXSIZE;			//maximum size of the buffers
	struct timeval instant;			//current cycle time instant
	deque<Vec> posBuffer;			//buffer of object position
	deque<struct timeval> timeBuffer;	//buffer of time instants
	Vec origPoint;
	Vec slope;

public:
	/*! Default constructor*/
	LinearRegression();

	/*! Main constructor
	\param maxSize maximum size for the buffers*/
	LinearRegression( int maxSize );

	/*! Class destructor*/
	~LinearRegression();

	/*! Method to insert new values in the buffers
	\param position position parameter value, with type Vec(x,y) (Y in formula)
	\param instantIn time instant parameter value, with type struct timeval (X in formula)*/
	void putNewValues( Vec pos, struct timeval instantIn);

	/*!\todo comment*/
	void calculateLineParameters();
	
	/*!\todo comment*/
	float estimatePointSparsing();

	/*! Method to clear the buffers*/
	void clearBuffs();

	/*! Method to calculate the declivity, based on the linear regression of the current buffer values
	\return Vec with the current declivity value (speed)*/
	Vec getDeclivity();

	/*! Method to calculate the point in origin, based on the linear regression of the current buffer values
	\return Vec with the value of the point where the derivative intercepts YY axe*/
	Vec getPointInOrig();

	/*! Method to print the values in the position buffer*/
	void printPosBuff();

	/*! Method to print the values in the time buffer*/
	void printTimeBuff();

	/*!Method to reset the internal time and position buffers to the last N given samples
	\param nSamples number of samples to keep in the buffers*/
	void resetToNSamples( unsigned int nSamples );

	/*Method to optimize the new velocity determination after a collision (based on prediction)
	\param t instant of collision
	\param collisionPos prediction ball position at collision instant
	\param newVel predicted velocity after collision*/
	void collision(float t, Vec collisionPos, Vec newVel);
	
	void checkAngleDiscontinuity(float newAngle);

	/*! Method that gets the number of points in the Linear Regression */
	int getNumberOfSamples();

	/*! Gets the first point in the buffer*/
	Vec getInitPoint();
};

}}

#endif
