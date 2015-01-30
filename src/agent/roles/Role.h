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

#ifndef ROLE_H_
#define ROLE_H_

#include <iostream>
#include <string>

#include "WorldState.h"
#include "ConfigXML.h"
#include "DriveVector.h"
#include "CambadaArbitrator.h"
#include "Strategy.h"
#include "Behaviour.h"

using namespace std;

namespace cambada {

/**
 * \brief Generic Role class
 *
 * Some example of roles: RoleStriker (the attacker), RoleGoalie (the goalkeeper), RoleMidfielder (the defenders)
 * Roles use Behaviours
 */
class Role {
public:
	Role();
	Role(RoleID roleRtti);
	virtual ~Role();

	/**
	 * \brief function that runs the Role and calculates the DriveVector
	 */
	void run(DriveVector* dv);

	/**
	 * \brief Method called when the role gains control
	 */
	virtual void gainControl()=0;

	/**
	 * \brief Method called when the role loses control
	 */
	virtual void loseControl()=0;

	/**
	 * \brief Returns the RTTI of the Role
	 */
	RoleID	getRoleRtti();

	/**
	 * \brief Field object
	 */
	static Field* 			field;
	static WorldState*		world;
	static ConfigXML*		config;
	static Strategy*		strategy;

protected:

	RoleID		roleRtti;

	virtual void determineNextState()=0;

public:
	/**
	 * \brief available options (Behaviours)
	 */
	CambadaArbitrator*		options;
};

} /* namespace cambada */
#endif /* ROLE_H_ */
