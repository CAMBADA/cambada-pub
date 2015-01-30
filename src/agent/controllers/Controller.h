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

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "DriveVector.h"
#include "WorldState.h"
#include "WorldStateDefs.h"
#include "ConfigXML.h"

namespace cambada {

/**
 * \brief Generic Controller class
 */
class Controller {
public:
	Controller();
	virtual ~Controller();
	float maxSpeed;
	static WorldState* world;
	static util::ConfigXML* config;
	static Field* field;			// field object

	void cycleRestart(); // used to reinitialize variables every cycle

	void setAvoidLevel(AvoidLevel newAvoidLevel, bool moveFree, bool avoidBall);
	void setDistribute(bool dist);

protected:
	AvoidLevel avoidLevel;
	bool distribute;

private:

	bool moveFree;
	bool avoidBall;

};

} /* namespace cambada */
#endif /* CONTROLLER_H_ */
