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

#include "Role.h"

namespace cambada {

// Init static objects
Field* 			Role::field = NULL;
WorldState*		Role::world = NULL;
ConfigXML*		Role::config = NULL;
Strategy*		Role::strategy = NULL;

Role::Role() {
	roleRtti = rNone;
	options = new CambadaArbitrator();
}

Role::Role(RoleID roleRtti) {
	this->roleRtti = roleRtti;
	options = new CambadaArbitrator();
}

Role::~Role() {
	delete options;
}

RoleID Role::getRoleRtti()
{
	return roleRtti;
}

void Role::run(DriveVector* dv)
{
	determineNextState();							// Call determineNextState virtual function
	options->calculate(dv);							// Calculate DriveVector and return it

	world->me->behaviour = options->getRtti();	// Update current behaviour
}

} /* namespace cambada */
