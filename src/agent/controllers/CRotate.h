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

#ifndef CROTATE_H_
#define CROTATE_H_

#include "Controller.h"

namespace cambada {

class CRotate: public cambada::Controller {
public:
	CRotate();
	virtual ~CRotate();

	/**
	 * Calculates the velocities of the robot in order to
	 * rotate around the ball
	 * \param errorRad angular error regarding target point
	 */
	void calcVel(DriveVector* dv, float errorRad);
};

} /* namespace cambada */
#endif /* CROTATE_H_ */
