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

#ifndef ROBOT_H_
#define ROBOT_H_

#include "WorldStateDefs.h"
#include "Vec.h"
#include "Ball.h"
#include "Obstacle.h"
#include "geometry.h"

namespace cambada {

class Robot {
public:
	Robot();

	// Shared atributes
	int		number;				// Robot number
	BehaviourID		behaviour;	// Behaviour index
	RoleID		role;			//
	int		coordinationFlag[2];//
	WSGameState currentGameState;// The current agent gamestate
	Vec		coordinationVec;	// Shared vec used in agents coordination
	WSColor	teamColor;			// Team color (should be Magenta or Cyan)
	WSColor	goalColor;			// Goal color (should be Blue or Yellow)
	unsigned char	nObst;		// Number of valid shared obstacles
	char	stuck;				//
	bool	roleAuto;			//
	bool	running;			//
	bool	coaching;			//
	bool	justKicked;			/*!< Ball was just kicked on this cycle*/
	bool	handicappedGrabber;	/*!< This robot grabber is not working*/
	unsigned char	opponentDribbling;
	float   battery[N_BATTERIES];// batteries state
	float	orientation;		// Robot absolute orientation
	float	angVelocity;		//
	Vec 	debugPoints[4];		// debugPoints - used for debugging
	Vec 	pos;				// Robot's absolute position
	Vec 	vel;				// Robot's absolute velocity
	Ball 	ball;				// Ball used by robot (can be seen or shared)
	Line	passLine;			// Line set when there is a pass

	ObstacleInfo obstacles[MAX_SHARED_OBSTACLES];
};

} /* namespace cambada */

#endif /* ROBOT_H_ */
