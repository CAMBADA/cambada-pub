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

#include "StuckRobotSensor.h"

#include <iostream>
using namespace std;
using namespace cambada;
using namespace cambada::geom;

namespace cambada {
namespace util {
	
StuckRobotSensor::StuckRobotSensor( int msTimeFrame , double maxDeltaPos , double maxDeltaOri )
{
	odoPos				= Vec( 0.0 , 0.0 );
	odoOri				= Angle( 0.0 );
	
	oneTimeFrameUnStucked	= true;
	stuckValue				= false;
	stucked					= false;

	firstTime				= true;

	this->msTimeFrame	= msTimeFrame;
	this->maxDeltaPos	= maxDeltaPos;
	this->maxDeltaOri	= maxDeltaOri;
}


void StuckRobotSensor::update( Vec pos , Angle ori , Vec dPos , Angle dOri )
{
	const int timeMS = timer.elapsed();
		
	Vec	posDelta; 
	Angle	oriDelta;
	
static bool oneTimeFrameUnStucked = true;

	cout << "StuckRobotSensor :: timer :: "<<timeMS<<"   > "<<msTimeFrame<<endl;
	if( timer.elapsed() > msTimeFrame )
	{
		timer.restart();	
		firstTime = true;
		cout << "StuckRobotSensor :: timer :: reset"<<endl;
		if( stucked )
			oneTimeFrameUnStucked = false;
		else
			oneTimeFrameUnStucked = true;
	}

	if( firstTime )
	{	
		odoPos		= pos;
		odoOri		= ori;
		firstTime	= false;
		cout << "StuckRobotSensor :: reset :: "<<endl;
		cout << "StuckRobotSensor :: pos "<<pos<<"  odoP "<<odoPos<<  "  delta "<<posDelta.length()<<"<"<<maxDeltaPos<<endl;
	}
	else	
	{
		odoPos += dPos;
		odoOri += dOri;

		posDelta = pos - odoPos;
		oriDelta = ori - odoOri;
		
		cout << "StuckRobotSensor :: pos "<<pos<<"  odoP "<<odoPos<<  "  delta "<<posDelta.length()<<"<"<<maxDeltaPos<<endl;
//		cout << "StuckRobotSensor :: ori "<<pos<<"  odoO "<<odoOri<<  "  delta "<<oriDelta.get_rad_pi()<<"<"<<maxDeltaOri<<endl;

		if( posDelta.length() < maxDeltaPos 
		/*||	fabs( oriDelta.get_rad_pi() ) < maxDeltaOri*/ )
			stucked = false;
		else
			stucked = true;
	}
	
//	if( oneTimeFrameUnStucked )
//		stuckValue = stucked;
//	else
//		stuckValue = false;
	stuckValue = stucked;

	cout << "StuckRobotSensor :: stucked "<<stucked<<endl;
}


bool StuckRobotSensor::isStucked()
{
	return stuckValue;
}

}}
