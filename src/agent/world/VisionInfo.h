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

#ifndef _VISIONINFO_H_
#define _VISIONINFO_H_

#include "Vec.h"
#include "WorldStateDefs.h"
#include "limits.h"

using namespace cambada::geom;

struct BallSensor
{
	BallSensor() { position = Vec(0.0,0.0); };
	Vec position;
};

//ball data for front vision information
struct BallFrontSensor
{
	BallFrontSensor() { position = Vec(0.0,0.0); height = 0.0; cyclesNotVisible = INT_MAX-100; }
	Vec		position;
	float	height;
	int cyclesNotVisible;
};

/* Not needed anymore
struct GoalSensor : public BallSensor
{
	Vec	leftPost;
	Vec rightPost;

	int nPoints;
	Vec point[MAX_GOAL];
};
*/

struct RadialSensor
{
	int nPoints;
	Vec point[MAX_POINTS];
};

class VisionInfo
{
public:
	VisionInfo();
	
	RadialSensor	obstacles;
	RadialSensor	lines;
	BallSensor		ball[MAX_BALLS];
	int nBalls;
};

class FrontVisionInfo
{
public:
	FrontVisionInfo();
	void clear();

	BallFrontSensor		ball[MAX_BALLS];
//	int nBalls;
};

#endif

