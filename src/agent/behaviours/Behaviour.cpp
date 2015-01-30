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

#include "Behaviour.h"
#include "LinRegression.h"
#include "log.h"
#include "Field.h"

using namespace std;

namespace cambada {

Field* 		Behaviour::field = NULL;
WorldState*	Behaviour::world = NULL;
ConfigXML*	Behaviour::config = NULL;
Strategy*	Behaviour::strategy = NULL;
CArc* Behaviour::cArc = NULL;
CMove* Behaviour::cMove = NULL;
CRotateAroundBall* Behaviour::cRotateAroundBall = NULL;
CRotate* Behaviour::cRotate = NULL;

Behaviour::Behaviour(): behaviourRtti(bBehaviour) {
}

Behaviour::Behaviour(BehaviourID id): behaviourRtti(id) {
}

Behaviour::~Behaviour() {}

void Behaviour::calculate(DriveVector* dv) {
	dv->velX = 0.;
	dv->velY = 0.;
	dv->velA = 0.;
	dv->kickPower = 0;
	dv->grabber = GRABBER_OFF;
}

void Behaviour::printHierarchy (unsigned int level) const {
	for (unsigned int i=0; i<level; i++)
		fprintf(stderr,"  ");
	fprintf(stderr,"%s\n",getName().c_str());
}

}
