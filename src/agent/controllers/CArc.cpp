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

#include "controllers/CArc.h"

namespace cambada {

CArc::CArc() : Controller(){
	internalMaxSpeed = config->getParam("dribble_max_linear_vel");

	goStraight = new SlidingWindow(10);
}

void CArc::calcVel(DriveVector* dv, float error){
	calcVel(dv, error,internalMaxSpeed);
}

void CArc::calcVel(DriveVector* dv, float error, float maxSpeed){
	float ballTrajectoryRadius; //ball trajectory arc
	float k1 = config->getParam("dribble_radius_vs_error"); // relate the angle with the ball trajectory arc
	float k2 = config->getParam("dribble_overcompensate_alfa"); // alpha related to ballRadius
	//float k3 = config->getParam("dribble_compensate_velTrans");
	float MAX_LINEAR_VEL = maxSpeed;
	float velTrans;
	//NOT USED float maxAngError = (config->getParam("kick_max_deg_error") * 8) * M_PI / 180;

	// *vA = 2 * error;
	dv->velA = config->getCtrlParam("compensateR").compensate(error);
	dv->velA = dv->velA > 1.7 ? 1.7 : dv->velA;
	dv->velA = dv->velA < -1.7 ? -1.7 : dv->velA;

	velTrans =  MAX_LINEAR_VEL + TOP_SPEED_M * fabs(dv->velA);

	ballTrajectoryRadius = ((M_PI / 2) / error) / k1;

	float alfa = k2 * atan2(BALL_CENTER_TO_ROBOT_CENTER, fabs(ballTrajectoryRadius));

	dv->velX = velTrans * sin(alfa);
	dv->velX = (ballTrajectoryRadius > 0 ? (dv->velX) : -(dv->velX));

	dv->velY = velTrans * cos(alfa);

	dv->grabber = GRABBER_ON;
}
} /* namespace cambada */
