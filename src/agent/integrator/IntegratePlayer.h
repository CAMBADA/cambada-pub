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

#ifndef INTEGRATE_PLAYER_H_
#define INTEGRATE_PLAYER_H_

#include "Vec.h"
#include "CoachInfo.h"
#include "ConfigXML.h"
#include "Localization.h"
#include "WorldStateDefs.h"
#include "EgoMotionEstimator.h"

namespace cambada {
using namespace geom;
using namespace util;

// Generic Integrate_Player class
class IntegratePlayer {
public:
	// Constructor
	IntegratePlayer(ConfigXML* conf);

	// Distuctor
	~IntegratePlayer();

	// Integrate function
	void integrate(vector<Vec> vision_lines, float lowLevelDx, float lowLevelDy, PlayerInfo coachInfo, bool firstTime);

	// Get function that return posicion
	Vec getPosition(){ return this->robot->pos; }

	// Get function that return velocity
	Vec getVelocity(){ return this->robot->vel; }

	// Get function that return orientation
	float getOrientation(){ return this->robot->orientation; }

	// Get function that return angle velocity
	float getAngleVelocity(){ return this->robot->angVelocity; }

	// Get function that return goal color
	WSColor getGoalColor(){ return this->robot->goalColor; }

	// Get function that return team color
	WSColor getTeamColor(){ return this->robot->teamColor; }

	// Get function that return role
	RoleID getRole(){ return this->robot->role; }

	// Get function that return roleAuto
	bool getRoleAuto(){ return this->robot->roleAuto; }

	// Get function that return running
	bool getRunning(){ return this->robot->running; }

protected:
	Robot* robot;					// Struct that contains Position, Velocity, Orientation and error associated
	EgoMotionEstimator egoMotion;	// Object that calc velocity
	Localization* localization;		// Interface used by calc localization parameters

};

}/* namespace cambada */
#endif /* INTEGRATE_PLAYER_H_ */
