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

#ifndef BALL_H_
#define BALL_H_

#include "Vec.h"

using namespace cambada::geom;

namespace cambada {

/**
 * \brief Model of a ball
 */
class Ball {
public:
	Ball();
	void clear();
	float distance();

	Vec pos;		// Ball absolute position
	Vec posRel;		// Ball relative position
	Vec vel;		// Ball velocity

	float height;	// Ball height
	bool own;		// TRUE if ball is visible by me, FALSE if ball is shared
	bool engaged;	// If the ball is engaged in robot
	bool visible;	// Is ball visible
	bool airborne;	// Ball in the air

	bool hasMoved;	// TODO: Ball just moved (needed?)
};

} /* namespace cambada */
#endif /* BALL_H_ */
