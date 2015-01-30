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

#ifndef FILTER_H_
#define FILTER_H_

#include "Vec.h"
#include <sys/time.h>

namespace cambada {
namespace util {
using namespace geom;

/* Filter interface class*/
class Filter
{
public:
	/*!Default constructor*/
	Filter()
	{
		this->lastPosition = Vec::zero_vector;
		this->lastVelocity = Vec::zero_vector;
	}

	/*!Class destructor*/
	virtual ~Filter(){}

	/*!Method to set the measure standard deviation to the desired value.
	\param readingDeviation standard deviation of the measures.*/
	virtual void setNoise( double readingDeviation ) =0;

	/*!Method to update the filter state given a measured position and the respective time instant.
	\param readPosition absolute position of the read sample. <b> Optional.</b>*/
	virtual void updateFilter( Vec readPosition, struct timeval instant ) =0;

	/*!Method to restart the particle filter internal variables
	\param initialPosition reference position initial aproximation to the filter*/
	virtual void resetFilter( Vec initialPosition, struct timeval instant ) =0;

	/*!Method to set the ball as not visible*/
	virtual void setNotVisible() =0;

	/*!Tests for a hard deviation between the measures and the estimation, to indicate direction changes
	\return True if a hard deviation has occured*/
	virtual bool hardDeviation() =0;

	/*!Method that return last velocity estimated*/
	virtual Vec getVelocity(int omniCyclesNotVisible=0) =0;

	/*!Method that return last position estimated*/
	Vec getPosition(){ return this->lastPosition; }
protected:
	Vec lastPosition;	/*!<Last position estimation of the filter*/
	Vec lastVelocity;	/*!<Last velocity estimation of the filter*/
};

}/* namespace util */
}/* namespace cambada */
#endif /* FILTER_H_ */
