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

#include "Robot.h"

namespace cambada {

Robot::Robot() {
	this->number = 0;
	this->behaviour = bBehaviour;
	this->role = rNone;
	this->coordinationFlag[0] = 0;
	this->coordinationFlag[1] = 0;
	this->currentGameState = stopRobot;
	this->coordinationVec = Vec::zero_vector;
	this->teamColor = (WSColor)Magenta;
	this->goalColor = (WSColor)Blue;
	this->nObst = 0;
	this->stuck = 0;
	this->roleAuto = true;
	this->running = false;
	this->coaching = true;
	this->opponentDribbling = 0;
	for(unsigned int i=0; i < N_BATTERIES; i++)
		this->battery[i] = 0.0f;
	this->orientation = 0.0f;
	this->angVelocity = 0.0f;
	for(unsigned int i=0; i<4; i++)
		this->debugPoints[i] = Vec::zero_vector;
	this->pos = Vec::zero_vector;
	this->vel = Vec::zero_vector;
	this->ball = Ball();
	this->passLine = Line();
}

} /* namespace cambada */
