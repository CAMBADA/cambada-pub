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

#ifndef _ROLESTOP_H_
#define _ROLESTOP_H_

#include "Role.h"
#include "CambadaArbitrator.h"
#include "General/BStop.h"

namespace cambada{

/**
 * \brief Role that stops the robot completely
 *
 * The robot is stopped in this Role, means no velocity, kicker and grabber disabled
 */
class RoleStop : public Role
{
public:
	RoleStop();
	virtual ~RoleStop();

	virtual void gainControl();
	virtual void loseControl();
	virtual void determineNextState();
};

}

#endif

