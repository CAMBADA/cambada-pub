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

#ifndef _STUCKROBOTSENSOR_H_
#define _STUCKROBOTSENSOR_H_

#include "Vec.h"
#include "Angle.h"
#include "Timer.h"

namespace cambada {
namespace util {

class StuckRobotSensor
{
	int		msTimeFrame;
	double	maxDeltaPos;
	double	maxDeltaOri;

	geom::Vec		odoPos;
	geom::Angle	odoOri;
	
	bool	stucked;
	bool	stuckValue;
	bool	oneTimeFrameUnStucked;
	bool	firstTime;
	
	Timer	timer;


public:
	StuckRobotSensor( int msTimeFrame , double maxDeltaPos , double maxDeltaOri );

	void update( geom::Vec pos , geom::Angle ori , geom::Vec dPos , geom::Angle dOri );
	bool isStucked();

};

}}

#endif	// _STUCKROBOTSENSOR_H_

