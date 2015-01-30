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

#include "Decision.h"
#include <stdlib.h>
#include <math.h>
#include "log.h"

using namespace cambada::geom;

namespace cambada {

Decision::Decision(WorldState* world)
{
	this->world = world;

	roleStop = new RoleStop();
	roleStriker = new RoleStriker();

	roleActive = roleStop;
}

Decision::~Decision()
{
	roleActive = NULL;
	delete roleStop;
	delete roleStriker;
}

void Decision::decide(DriveVector* dv)
{
	static bool ballEng = false;
	static int engCount = 0;
	if(world->me->ball.engaged && engCount < 10) // max 10
		engCount++;
	else if(!world->me->ball.engaged && engCount > 0) // min 0
		engCount--;

	if(engCount > 7 && !ballEng)
	{
		world->me->coordinationVec = world->me->ball.pos;
		ballEng = true;
	}else if(engCount < 3 && ballEng){
		ballEng = false;
	}

	RoleID newRoleIdx = roleSelection(); 		// Get new role IDX from roleSelection function

	if( newRoleIdx != roleActive->getRoleRtti() )	// If the desired role differs from the active one
	{
		roleActive->options->loseControl();		// Old role calls lose control of its BDIs
		roleActive->loseControl();				// Old role loses control
		roleActive = getRoleFromID(newRoleIdx);	// Change active role to the new one
		world->me->role = roleActive->getRoleRtti();		// Update current role
		roleActive->gainControl();				// New role gains control
	}

	roleActive->run(dv);						// Call the run function for the active role
}

Role* Decision::getRoleFromID(RoleID idx) {
	Role* role = NULL;

	switch( idx )
	{
	case rStop: role = roleStop; break;
	case rStriker: role = roleStriker; break;
	default: role = roleStop; break;
	}

	return role;
}

RoleID Decision::roleSelection()
{
	// Stop decision
	if( !world->me->running )
	{
		return rStop;
	}

	// Forced role from basestation
	if( !world->me->roleAuto )
		return (RoleID)world->me->role;

	if(world->gameState == stopRobot)
		return rStop;

	
	return rStriker;
}

}
