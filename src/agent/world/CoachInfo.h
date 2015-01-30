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

#ifndef _COACHINFO_H_
#define _COACHINFO_H_

#include "Robot.h"
#include "WorldStateDefs.h"

using namespace cambada;
using namespace cambada::geom;

namespace cambada
{

struct FormationInfo
{
	FormationInfo()
	{
		reset();
	}
	;

	void reset()
	{
		formationIdFreePlay = 0;
		formationIdSP = 0;

		for (int i = 0; i < N_CAMBADAS; i++)
		{
			position[i] = Vec::zero_vector;
			posId[i] = -1;
		//	ourSetPiecesReceiverPriority[i] = -1;
			cover[i]=false;
		}
	}
	;

	int formationIdFreePlay;
	int formationIdSP;

	Vec position[N_CAMBADAS];
	int posId[N_CAMBADAS];
	//int ourSetPiecesReceiverPriority[N_CAMBADAS];
	bool cover[N_CAMBADAS];
};

/**
 * Player information that coach sends
 */
class PlayerInfo
{
public:
	PlayerInfo();

	RoleID role;
	WSColor teamColor; // Team color (should be Magenta or Cyan)
	WSColor goalColor; // Goal color (should be Blue or Yellow)
	bool roleAuto; //
	bool running; //
	bool coaching; //
	float orientation;
	Vec pos;
	int number;
};

class CoachInfo
{
public:
	CoachInfo();

	PlayerInfo playerInfo[N_CAMBADAS];
	int changePositionSN[N_CAMBADAS];

	int gameState;
	int ourGoals;
	int theirGoals;
	int time; // in sec
	bool manualFormation; // selected from basestation
	int half; // FIXME this is formation id  this should be in FormationInfo

	char taxiRobotSN[N_CAMBADAS]; // robot SN for taxi
	Vec taxiPos; // position for taxi
	bool taxiOri; // when true, taxi is for orientation
};

}

#endif

