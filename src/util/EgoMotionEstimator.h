/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA UTILITIES
 *
 * CAMBADA UTILITIES is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA UTILITIES is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef EGOMOTIONESTIMATOR_H_
#define EGOMOTIONESTIMATOR_H_

#include "SlidingWindow.h"
#include "LinRegression.h"
#include "Vec.h"
#include "Timer.h"

namespace cambada {
namespace util {
using namespace geom;

class EgoMotionEstimator
{
public:
	EgoMotionEstimator(int nSamples);
	virtual ~EgoMotionEstimator();
	void update();
	void addValue(float x, float y, float theta);
	void addValue(Vec pos, float theta);
	geom::Vec getLinearVelocity(void);
	float getAngularVelocity(void);
	void reset();

private:
	cambada::util::LinRegression xLinReg,yLinReg,thetaLinReg;
	cambada::util::SlidingWindow xWindow, yWindow, thetaWindow, timeWindow;
	Timer timer;
	geom::Vec linVel;
	float angVel;

};

}}

#endif /* EGOMOTIONESTIMATOR_H_ */
