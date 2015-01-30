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

#ifndef _DECISION_H_
#define _DECISION_H_

#include "Role.h"
#include "Timer.h"
#include "Robot.h"
#include "Formation.h"
#include "DriveVector.h"

#include "RoleStop.h"
#include "RoleStriker.h"

namespace cambada {

class Decision
{
public:
	Decision(WorldState* world);
	virtual ~Decision();

	WorldState*	world;

	void decide(DriveVector* dv);

private:
	RoleID roleSelection();
	Role* getRoleFromID(RoleID idx);

	Role*		roleActive;
	bool		keeperAlive;

	// Role List

	RoleStop* roleStop;
	RoleStriker* roleStriker;
};

}

#endif

