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

#include "PID.h"
#include "math.h"

namespace cambada
{
namespace util
{

PID::PID( float kp , float ki , float kd , float minOut, float maxOut , float maxInt ) 
{
	this->kp		= kp;
	this->ki		= ki; 
	this->kd		= kd;
	this->minOut	= minOut;
	this->maxOut	= maxOut;
	this->maxInt	= maxInt;

	this->lastValue	= 0.0;
	this->integral	= 0.0;
	this->firstTime	= true;
	this->used 		= false;
}

PID::PID( const PID& pid ) 
{
	this->kp		= pid.kp;
	this->ki		= pid.ki; 
	this->kd		= pid.kd;
	this->minOut	= pid.minOut;
	this->maxOut	= pid.maxOut;
	this->maxInt	= pid.maxInt;

	this->lastValue	= pid.lastValue;
	this->integral	= pid.integral;
	this->firstTime	= true;
}

float PID::compensate( float value )
{

	if(isinf(value) || isnan(value)) value = 0.0;

	used = true; // this compensator is used this cycle

	float out = 0.0;

	integral += value;

	if(integral > maxInt)
		integral = maxInt;
	if(integral < -maxInt)
		integral = -maxInt;

	if( firstTime )
	{
		integral = 0;
		out = kp*value;
		firstTime = false;
	}
	else
	{
/*
		if(fabs(value) < 0.001)
			out = kp*value + ki*integral;
		else
			out = kp*value + kd*(value - lastValue)/value + ki*integral;
*/
		out = kp*value + kd*(value - lastValue) + ki*integral;

	}

	/*
	if( out > maxOut )
		out = maxOut;
	else if( out < -maxOut )
		out = -maxOut;
	 */

	lastValue = value;

	return out;
}

float PID::compensate2( float value )
{
	used = true; // this compensator is used this cycle

	float out = 0.0;

	if(value < 0.001) return 0.0;

	if( firstTime )
	{
		integral = 0;
		out = kp*value;
		firstTime = false;
	}
	else
		out = kp*value + kd*(value - lastValue) / value;

	if( out > maxOut )
		out = maxOut;
	else
		if( out < -maxOut )
			out = -maxOut;

	if( out < minOut && out > 0.0 )
		out = minOut;
	else
		if( out > -minOut && out < 0.0 )
			out = -minOut;

	lastValue = value;

	return out;
}

void PID::check()
{
	if(!used)
	{
		reset();
	}
	used = false;
}

void PID::reset()
{
	firstTime = true;
}

void PID::display() 
{
	printf("  p(%.2f) i(%.2f) d(%.2f) maxOut(%.2f) maxInt(%.2f)\n", kp, ki, kd, maxOut, maxInt);	
}

}
}
