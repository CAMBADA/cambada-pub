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

#ifndef _LOWLEVELINFO_H_
#define _LOWLEVELINFO_H_

#include "Vec.h"
#include "WorldStateDefs.h"
#include "rtdb.h"
#include "Timer.h"
#include "common.h"

#define DISENGAGE_TIME_THRESHOLD 150
#define GRABBER_TOUCHED_THRESHOLD 15

namespace cambada {

class LowLevelInfo
{
public:
	LowLevelInfo();

	/*! Method to update the low level information (odometry, ball engaged sensor, batteries)*/
	void updateInfo();

	/*!\return the XX displacement*/
	float getDX();
	/*!\return the YY displacement*/
	float getDY();
	/*!\return the angular displacement*/
	float getDW();

	/*!\return the XX velocity*/
	float getVelX();
	/*!\return the YY velocity*/
	float getVelY();
	/*!\return the angular velocity*/
	float getVelW();

	/*!\return flag returned by the kicker control*/
	bool getJustKicked();
	
	geom::Vec		odoPosition;
	float		odoOrientation;
	bool		barrierState;
	unsigned short	batteryStatus[N_BATTERIES];
	float		dx, dy, dw;
	float		realDx, realDy, realDw;
	//Variables for adding a timer threshold when deciding ball not engaged
	util::Timer		notEngagedTimer;
	bool			ballOut;
	bool			justKicked;

	//Variables to detect touches on the grabber
	bool			rArmTouched, lArmTouched;
	unsigned char	rArmLastPos, lArmLastPos;
};

}

#endif

