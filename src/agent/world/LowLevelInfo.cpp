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

#include "LowLevelInfo.h"

using namespace cambada;
using namespace cambada::geom;

namespace cambada {

LowLevelInfo::LowLevelInfo()
{
	odoPosition = Vec(0,0);
	barrierState = 0;
	dy=dx=dw=0;
	odoOrientation = 0.0;
	for(unsigned int i = 0; i < N_BATTERIES; i++){
		this->batteryStatus[i] = 0;
	}

	ballOut = true;
	justKicked = false;
	rArmLastPos = lArmLastPos = 0;
	rArmTouched = lArmTouched = false;
}


void LowLevelInfo::updateInfo()
{
	float x, y, o, dxReal, dyReal, dwReal;

	CMD_Pos_GET( &x , &y , &o, &dxReal, &dyReal, &dwReal );
	realDx = dxReal;
	realDy = dyReal;
	realDw = dwReal;
	dx = x - odoPosition.x;
	dy = y - odoPosition.y;
	dw = ( Angle(o) - Angle(odoOrientation) ).get_rad_pi(); 
	if( fabs(dx) > 1 || fabs(dy) > 1 )
		dx = dy = dw = 0;

	// 03/12/2010 - Bug in the odometry: rotate the diference tp the new axis
	Vec rel_odo = Vec(dx,dy).rotate(-odoOrientation);
	dx = rel_odo.x;
	dy = rel_odo.y;

	dx *= 1000.0;
	dy *= 1000.0;
	
	odoPosition = Vec(x,y);
	odoOrientation = o;

	unsigned short bat[3];
	CMD_Info_GET( &bat[0], &bat[1], &bat[2], NULL, &justKicked);

	unsigned int lArm, rArm;
	CMD_Grabber_Info_GET(&lArm, &rArm, NULL, NULL);

	if (barrierState == false)
	{
		if (lArm > ENGAGED_THRESHOLD_HIGH && rArm > ENGAGED_THRESHOLD_HIGH)
		{
			barrierState = true;
			ballOut = false;
		}
	}
	else
	{
		if (lArm < ENGAGED_THRESHOLD_LOW && rArm < ENGAGED_THRESHOLD_LOW)
		{
			//On the first cycle with arms low, initialize the control variable and (re)start the timer
			if (!ballOut) {
				ballOut = true;
				notEngagedTimer.restart();
			}

			if (ballOut && notEngagedTimer.elapsed() > DISENGAGE_TIME_THRESHOLD)
				barrierState = false;
		}
	}

	for( int i = 0 ; i < N_BATTERIES ; i++ )
	{
		batteryStatus[i] = bat[i];
	}

	if ( (rArm - rArmLastPos) > GRABBER_TOUCHED_THRESHOLD )
		rArmTouched = true;
	else
		rArmTouched = false;

	if ( (lArm - lArmLastPos) > GRABBER_TOUCHED_THRESHOLD )
		lArmTouched = true;
	else
		lArmTouched = false;

	rArmLastPos = rArm;
	lArmLastPos = lArm;
}

float LowLevelInfo::getDX() {return realDx;}
float LowLevelInfo::getDY() {return realDy;}
float LowLevelInfo::getDW() {return realDw;}

/*!\return the XX velocity*/
float LowLevelInfo::getVelX() {
	return getDX() / (MOTION_TICK/1000);
}

/*!\return the YY velocity*/
float LowLevelInfo::getVelY() {
	return getDY() / (MOTION_TICK/1000);
}

/*!\return the angular velocity*/
float LowLevelInfo::getVelW() {
	return getDW() / (MOTION_TICK/1000);
}

bool LowLevelInfo::getJustKicked() {return justKicked;}

}
