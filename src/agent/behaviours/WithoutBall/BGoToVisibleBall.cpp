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

#include "BGoToVisibleBall.h"

using namespace cambada::util;

namespace cambada {

BGoToVisibleBall::BGoToVisibleBall() : Behaviour(bGoToVisibleBall) {

}

BGoToVisibleBall::~BGoToVisibleBall() {

}

bool BGoToVisibleBall::checkInvocationCondition(){
	if(world->me->ball.engaged ){
		return false;
	}else{
		return true;
	}
}

bool BGoToVisibleBall::checkCommitmentCondition() {
	return checkInvocationCondition(); // Same as IC
}

void BGoToVisibleBall::calculate(DriveVector* dv){
	// Adjust positions when avoid is required
	Vec pos = world->me->ball.posRel;
	float ballVel = world->me->ball.vel.length();

	float minLength = 0.5 + ballVel;
	if(pos.length() < minLength)
		pos = pos.setLength(minLength);

	pos = world->getAvoidAdjustedPosition(pos, true, false, avoidSemi);
	cMove->calcVel(dv, pos, world->me->ball.posRel, MAX_SPEED);
}

}
