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

#include "controllers/CRotateAroundBall.h"

namespace cambada {

CRotateAroundBall::CRotateAroundBall() : Controller(){}

void CRotateAroundBall::calcVel(DriveVector* dv, float errorRad, float maxVelA){
	float Rr = 0.258;

	config->getCtrlParam("compensateR").reset();
	dv->velA = config->getCtrlParam("compensateR").compensate(errorRad);

	// Maximum angular velocity
	if(fabs(dv->velA) > maxVelA) {
		dv->velA = maxVelA * fabs(dv->velA)/dv->velA;
	}
	dv->velX = dv->velA * Rr;
	dv->velY = 0;

	if(world->me->vel.length() > 2.0)
		dv->velA = 0.0;

	dv->grabber = GRABBER_ON;
}

} /* namespace cambada */
