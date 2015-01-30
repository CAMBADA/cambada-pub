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

#include "Controller.h"

namespace cambada {

WorldState* Controller::world = NULL;
util::ConfigXML* Controller::config = NULL;
Field* 		Controller::field = NULL;

Controller::Controller() {
	maxSpeed = MAX_SPEED;
	distribute = false;
}

Controller::~Controller() {
}

void Controller::cycleRestart() {
	setAvoidLevel(avoidNone, true, false);
	setDistribute(false);
}

void Controller::setAvoidLevel(AvoidLevel newAvoidLevel, bool moveFree, bool avoidBall) {
	this->avoidLevel = newAvoidLevel;
	this->moveFree = moveFree;
	this->avoidBall = avoidBall;
}

void Controller::setDistribute(bool dist) {
	this->distribute = dist;
}

} /* namespace cambada */
