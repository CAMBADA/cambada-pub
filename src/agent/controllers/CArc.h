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

#ifndef CARC_H_
#define CARC_H_

#include "Controller.h"
#include "WorldState.h"
#include "DriveVector.h"
#include "ConfigXML.h"

#define TOP_SPEED_M    -0.21
#define BALL_CENTER_TO_ROBOT_CENTER 0.258

namespace cambada {

/**
 * \brief Controller for Arc Movements, with ball
 */
class CArc: public cambada::Controller {
public:
	CArc();
	virtual ~CArc(){}

	/**
	 * Calculates the velocities of the robot in order to perform an arc,
	 * using an angular error related to the target point.
	 * \param error angular error regarding target point
	 */
	void calcVel(DriveVector* dv, float error);

	/**
	 * Calculates the velocities of the robot in order to perform an arc,
	 * using an angular error related to the target point.
	 * \param error angular error regarding target point
	 * \param maxSpeed the maximum allowed linear speed
	 */
	void calcVel(DriveVector* dv, float error, float maxSpeed);

private:
	float internalMaxSpeed;
	SlidingWindow* goStraight;
};

} /* namespace cambada */
#endif /* CARC_H_ */
