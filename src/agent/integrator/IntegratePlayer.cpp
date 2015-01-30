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

#include "IntegratePlayer.h"
#include "UseCompass.h"
#include <syslog.h>


namespace cambada {

IntegratePlayer::IntegratePlayer(ConfigXML* conf) : egoMotion(PLAYER_POSITION_BUFFER_SIZE)
{
	// Todo: Substituir o USE_COMPASS por uma entrada no ConfigXML
	localization = new UseCompass(conf);

	robot 				= new Robot();
	robot->pos 			= Vec::zero_vector;
	robot->vel 			= Vec::zero_vector;
	robot->orientation 	= 0.0;
	robot->angVelocity 	= 0.0;
	robot->goalColor 	= Blue;
	robot->teamColor 	= Blue;
	robot->role	 		= rNone;
	robot->running		= false;
}

IntegratePlayer::~IntegratePlayer()
{
	delete robot;
	localization->~Localization();
}

void IntegratePlayer::integrate(vector<Vec> vision_lines, float lowLevelDx, float lowLevelDy, PlayerInfo coachInfo, bool firstTime)
{
	if(firstTime)
		egoMotion.reset();

	// Update localization
	localization->integrate(vision_lines,lowLevelDx,lowLevelDy,coachInfo.goalColor,firstTime);

	static int numberOfFails = 0;
	const double ORIENTATION_MAX_ERROR		= 80.0;
	const double MIRROR_ORIENTATION_ERROR	= 180.0 - ORIENTATION_MAX_ERROR;

	// Check conditions
	double errLoc = (firstTime)? 0 : localization->getErrorLoc();
	if( fabs( errLoc ) > MIRROR_ORIENTATION_ERROR )
	{
		syslog(LOG_DEBUG,"MIRRORED");

		// Mirror
		localization->mirror();
		egoMotion.reset();
		CMD_set_orientation(1000);
	}
	else if( fabs( errLoc ) > ORIENTATION_MAX_ERROR )
	{
		syslog(LOG_DEBUG,"nFails++");
		numberOfFails++;
		CMD_set_orientation(1000);
	}
	else
	{
		numberOfFails = 0;
		CMD_set_orientation(localization->getOrientationEarth());
	}

	// HACK numberOfFails = 0;
	if( numberOfFails > 10 )
	{
		syslog(LOG_DEBUG,"Reloc by nFails>10");

		// stop the robot during the reloc
		CMD_Vel_SET(0.0,0.0,0.0,false);

		// Update data
		localization->integrate(vision_lines,lowLevelDx,lowLevelDy,coachInfo.goalColor, true);
		numberOfFails=0;
		egoMotion.reset();
	}

	// Update player info
	robot->pos 			= localization->getPosition();
	robot->orientation 	= localization->getOrientation();
//	data.errPos 		= localization->getErrorPos();
//	data.errLoc 		= (firstTime)? 0 : localization->getErrorLoc();
	robot->goalColor	= coachInfo.goalColor;
	robot->teamColor	= coachInfo.teamColor;
	robot->roleAuto		= coachInfo.roleAuto;
	robot->running		= coachInfo.running;
	robot->role 		= coachInfo.role;

	// Update linear and angular velocity's
	egoMotion.addValue(robot->pos, robot->orientation);
	egoMotion.update();

	// Update player velocity
	robot->vel 			= (firstTime)? Vec(0.0,0.0) : egoMotion.getLinearVelocity();
	robot->angVelocity 	= ( fabs(egoMotion.getAngularVelocity())<0.05 || firstTime)? 0 : egoMotion.getAngularVelocity();
}
}/* namespace cambada */
