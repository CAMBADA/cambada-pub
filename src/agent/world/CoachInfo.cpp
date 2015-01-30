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

#include "CoachInfo.h"

namespace cambada {

PlayerInfo::PlayerInfo()
{
	role = rNone;
	teamColor = Cyan;			// Team color (should be Magenta or Cyan)
	goalColor = Blue;			// Goal color (should be Blue or Yellow)
	roleAuto = true;			//
	running = false;			//
	coaching = false;			//
	orientation = 0.0;
	pos = Vec::zero_vector;
	number = 0;
};

CoachInfo::CoachInfo()
{
	for( int i = 0 ; i < N_CAMBADAS ; i++ )
	{
		taxiRobotSN[i] = 0;
		changePositionSN[i] = 0;
	}

	ourGoals = 0;
	theirGoals = 0;
	time = 0;
	half = 0;
	gameState = SIGstop;


	taxiPos = Vec::zero_vector;
	taxiOri = false;
}

}
