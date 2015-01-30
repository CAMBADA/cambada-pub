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

#include "BSearchBall.h"

namespace cambada {

BSearchBall::BSearchBall() : Behaviour(bSearchBall){
	currentIdx=0;
}

BSearchBall::~BSearchBall() {}

void BSearchBall::gainControl() {

	// Populate search positions
	fieldPts.clear();
	fieldPts.push_back(Vec(field->halfWidth/2,field->halfLength/2));
	fieldPts.push_back(Vec(-field->halfWidth/2,field->halfLength/2));
	fieldPts.push_back(Vec(-field->halfWidth/2,-field->halfLength/2));
	fieldPts.push_back(Vec(field->halfWidth/2,-field->halfLength/2));

	currentIdx=0;
}

bool BSearchBall::checkInvocationCondition(){
	if(!world->me->ball.visible) {
		return true;
	}else{
		return false;
	}
}

bool BSearchBall::checkCommitmentCondition() {
	// Same as IC
	return checkInvocationCondition();
}

void BSearchBall::calculate(DriveVector* dv){

	if(fieldPts.size() > 0) {

		if((fieldPts.at(currentIdx)-world->me->pos).length() < 0.2)
			currentIdx = (currentIdx+1)%(fieldPts.size());

		unsigned int i2 = (currentIdx+1)%(fieldPts.size());

		Vec posRel = world->abs2rel(fieldPts.at(currentIdx));

		cMove->setAvoidLevel(avoidFull,true,true);
		cMove->calcVel(dv,posRel, world->abs2rel(fieldPts.at(i2)), MAX_SPEED);

	}else{
		dv->allOff();
	}
}

} /* namespace cambada */
