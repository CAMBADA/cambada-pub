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

#include "CMove.h"

using namespace cambada::geom;

namespace cambada {

CMove::CMove() : Controller(){}

void CMove::calcVel(DriveVector* dv, geom::Vec relPos, geom::Vec relOri, float maxSpeed, float oriControl){

	world->me->debugPoints[0] = world->rel2abs(relPos);

	if(avoidLevel != avoidNone)
		relPos = world->getAvoidAdjustedPosition(relPos, true, false, avoidLevel);

	float OUTDIST = 0.25;

	// Try not touch in the protection barrier
	Vec absTarget = world->rel2abs(relPos);
	if(clipPosition() && !world->getField()->isInside(absTarget, OUTDIST))
	{
		if(absTarget.x < -world->getField()->halfWidth) // left line
		{
			absTarget = Vec(-world->getField()->halfWidth - OUTDIST, absTarget.y);
		}
		else if(absTarget.x > world->getField()->halfWidth) // right line
		{
			absTarget = Vec(world->getField()->halfWidth + OUTDIST, absTarget.y);
		}

		if(absTarget.y < -world->getField()->halfLength) // end line
		{
			absTarget = Vec(absTarget.x, -world->getField()->halfLength - OUTDIST);
		}
		else if(absTarget.y > world->getField()->halfLength) // their end line
		{
			absTarget = Vec(absTarget.x, world->getField()->halfLength + OUTDIST);
		}

		relPos = world->abs2rel(absTarget);
	}

	// Clipping do maxSpeed
	if(maxSpeed > MAX_SPEED)
		maxSpeed = MAX_SPEED;

	float linearVel = config->getCtrlParam("compensateTrans").compensate(relPos.length());

	geom::Vec vel = relPos.setLength(linearVel);

	if(vel.length() > maxSpeed)
	{
		vel = vel.setLength(maxSpeed);
		dv->velX = vel.x;
		dv->velY = vel.y;
	}else{
		dv->velX = vel.x;
		dv->velY = vel.y;
	}

	if(relPos.length() < (config->getParam("movePosThreshold")))
	{
		dv->velX = 0.0;
		dv->velY = 0.0;
	}

	// Protect when orientation vec is the same as pos vec
	if(relOri == relPos) {
		relOri = relOri.setLength(relOri.length() + 1);
	}

	/* levar o robô a rodar ao longo do tempo, enquando durar a translação.*/
	Angle ang        = relOri.angleFromY();

	if(distribute)
	{
		float time2target = (relPos.length() / maxSpeed); // factor to account for accel and decel
		if(relPos.length() > 0.1)
		{
			float maxVelRot = ang.get_rad_pi() / time2target;
			dv->velA = maxVelRot;

			fprintf(stderr,"CMOVE maxVelRot %.3f %.3f - distance (%.2f,%.2f) %.3f \n", time2target, maxVelRot, relPos.x, relPos.y, relPos.length());

		}else{
			dv->velA = config->getCtrlParam("compensateR").compensate( ang.get_rad_pi() );
			fprintf(stderr,"CMOVE else %.3f\n", relPos.length());
		}
	}else{
		dv->velA = config->getCtrlParam("compensateR").compensate( ang.get_rad_pi() );
	}

	// Adjust limitations
	adjustLimits(dv, oriControl);

}

bool CMove::clipPosition() {
	if(world->gameState == preOwnKickOff ||
			world->gameState == preOwnFreeKick ||
			world->gameState == preOwnCornerKick ||
			world->gameState == preOwnGoalKick ||
			world->gameState == preOwnThrowIn)
		return false;

	if(world->gameState == postOwnKickOff ||
			world->gameState == postOwnFreeKick ||
			world->gameState == postOwnCornerKick ||
			world->gameState == postOwnGoalKick ||
			world->gameState == postOwnThrowIn)
		return false;

	if(world->gameState == parking)
		return false;

	return true;
}

void CMove::adjustLimits(DriveVector* dv, float oriControl) {

	if(dv->getLinearVel() > MAX_VL)
		dv->setLinearVel(MAX_VL);

	if(dv->velA > 0.0 && dv->velA > MAX_VA)
		dv->velA = MAX_VA;
	if(dv->velA < 0.0 && dv->velA < -MAX_VA)
		dv->velA = -MAX_VA;


	return;

	// Create the line that limits the linear velocity / angular velocity

	float m = -MAX_VL/MAX_VA;
	float b = MAX_VL;

	Vec givenPoint = Vec(fabs(dv->velA), dv->getLinearVel());
	Vec correctedPoint = Vec(
			(givenPoint.x > MAX_VA) ? MAX_VA : givenPoint.x,
			(givenPoint.y > MAX_VL) ? MAX_VL : givenPoint.y
	);

	Vec sameVelA = Vec(correctedPoint.x, m*correctedPoint.x + b);		// The point in line with the same VelA
	Vec sameVelL = Vec((correctedPoint.x - b)/m, correctedPoint.y);		// The point in line with the same VelL

	// If above line, we have to make an adjustment
	if( correctedPoint.y > sameVelA.y )
	{
		// if oriControl = 0 -> rotate first (sameVelA)
		// if oriControl = 100 -> linear movement first (sameVelL)

		Vec relOriControl = sameVelL - sameVelA; // vector from oriControl = 0 to oriControl = 100
		relOriControl = relOriControl.setLength(relOriControl.length()*oriControl/100.0); // now its length varies with oriControl variable
		correctedPoint = sameVelA + relOriControl;
	}

	// now apply the adjustment
	if(dv->velA > 0.0)
		dv->velA = correctedPoint.x;
	else
		dv->velA = -correctedPoint.x;

	dv->setLinearVel(correctedPoint.y);

}

}
