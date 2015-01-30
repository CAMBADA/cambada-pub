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

#ifndef CMOVE_H_
#define CMOVE_H_

#include "Controller.h"
#include "WorldState.h"
#include "DriveVector.h"
#include "ConfigXML.h"
#include "common.h"
#include <math.h>

namespace cambada {

/**
 * \brief Controller for straight movements, without ball
 */
class CMove : public cambada::Controller{
public:
	CMove();
	virtual ~CMove(){}

	/*!
		Calculates the linear and angular velocities using the error related
		to the target point. A maximum linear velocity can be specified...
		\param relPos Vector to the relative point to move
		\param relOri Vector to the relative point to look to
		\param maxSpeed maximum linear velocity for the movement
	 */
	void calcVel(DriveVector* dv, geom::Vec relPos, geom::Vec relOri, float maxSpeed = MAX_SPEED, float oriControl = 100);

private:
	bool clipPosition();

	/**
	 * \brief Adjusts the velocities when we are above the allowed limits
	 */
	void adjustLimits(DriveVector* dv, float oriControl);
};

} /* namespace cambada */
#endif /* CMOVE_H_ */
