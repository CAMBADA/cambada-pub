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

#ifndef CAMBADA_H_
#define CAMBADA_H_

#include <stdio.h>
#include <iostream>
#include <strings.h>
#include <sys/time.h>
#include <time.h>

// Libs
#include "rtdb.h"
#include "Strategy.h"

// Utils
#include "ConfigXML.h"
#include "Clock.h"

#include "WorldState.h"
#include "Integrator.h"
#include "Decision.h"
#include "Robot.h"
#include "SetPieces.hxx"

// Controllers
#include "controllers/CArc.h"
#include "controllers/CMove.h"
#include "controllers/CRotateAroundBall.h"

namespace cambada {

/**
 * \brief The top layer of the agent's Artificial Intelligence
 */
class Cambada {
public:
	Cambada();
	virtual ~Cambada();
	bool parseArguments( int argc , char* argv[] );
	void thinkAndAct();
	bool reconfigure();
	void rampVelA();

private:
	void printHelp();
	WorldState*		world;
	ConfigXML* 		config;
	Field*			field;
	Integrator*		integrator;
	Decision*		decision;
	Strategy*		strategy;
	SetPieces*		setPieces;

	DriveVector* dv; // Low level information to pass to the HW

	char*	argv;
	int		argc;

	float lastVelA;
};

} /* namespace cambada */
#endif /* CAMBADA_H_ */
