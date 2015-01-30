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

#include "BKickToTheirGoal.h"

namespace cambada {

BKickToTheirGoal::BKickToTheirGoal() : Behaviour(bKick) {

}

BKickToTheirGoal::~BKickToTheirGoal() {}

bool BKickToTheirGoal::checkInvocationCondition() {
	if( !world->me->ball.engaged )
		return false;

	return true;
}

bool BKickToTheirGoal::checkCommitmentCondition() {
	if( !world->me->ball.engaged )
		return false;

	return true;
}

void BKickToTheirGoal::calculate(DriveVector* dv) {
	dv->motorsOff();
	dv->grabber = GRABBER_ON;

	Vec kickTo = field->theirGoal;
	float distToTheirGoal = (kickTo - world->me->pos).length();

	float errorRad = world->abs2rel(kickTo).angleFromY().get_rad_pi();
	cRotateAroundBall->calcVel(dv, errorRad);
	if(shootOnGoal(kickTo))
	{
		dv->kick( distToTheirGoal );
	}
}

bool BKickToTheirGoal::shootOnGoal(Vec pointToKick)
{
	float maxAngle = 4.33545236;
	if(fabs(world->abs2rel(pointToKick).angleFromY().get_deg_180()) < maxAngle)
		return true;

	return false;
}

} /* namespace cambada */
