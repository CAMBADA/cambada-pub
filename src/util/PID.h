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

#ifndef _PID_H_
#define _PID_H_

#include <stdio.h>
#include <iostream>

using namespace std;

namespace cambada
{
namespace util
{

class PID 
{

public:
	PID( float kp = 1.0 , float ki = 0.0 , float kd = 0.0 , float minOut=0.0,
	  float maxOut = 100.0 , float maxInt = 0.0 ); 
	PID( const PID& pid ) ;

	float compensate( float value );
	float compensate2( float value );

	inline void setP( float p ) { kp = p;}
	inline void setI( float i ) { ki = i; }
	inline void setD( float d ) { kd = d; }
	inline void setMinOut( float mo ) { minOut = mo; }
	inline void setMaxOut( float mo ) { maxOut = mo; }
	inline void setMaxInt( float mi ) { maxInt = mi; }
	
	inline float getP() { return kp; }
	inline float getI() { return ki; }
	inline float getD() { return kd; }
	inline float getMaxOut() { return maxOut; }
	inline float getMaxInt() { return maxInt; }
	inline float getMinOut() { return minOut; }
	
	void reset();
	void display(); 
	void check();
	
private:
	float kp;
	float ki;
	float kd;
	float minOut;
	float maxOut;
	float maxInt;
	float lastValue;
	float integral;
	bool firstTime;
	bool used;
};

}
}

#endif // _PID_H_

