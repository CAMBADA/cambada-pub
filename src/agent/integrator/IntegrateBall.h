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

#ifndef INTEGRATE_BALL_H_
#define INTEGRATE_BALL_H_

//#include "WorldStateDefs.h"
#include "VisionInfo.h"
#include "Filter.h"
#include "KalmanFilter.h"
#include "ParticleFilter.h"
#include "Vec.h"
#include "Ball.h"
#include "Field.h"
#include <vector>

namespace cambada {
using namespace util;
using namespace geom;

// Generic Integrate_Ball class
class IntegrateBall
{
public:
	// Construtor
	IntegrateBall(Field* world_field, double deviation, struct timeval instant);

	// Virtual Distuctor
	~IntegrateBall();

	// Virtual function to be implemented in each mode
	void integrate(vector<Ball> ballsVision, vector<BallFrontSensor> ballsFrontVision, Ball* shareBall, struct timeval instant, bool ballHitFront);

	// Function that return position
	Vec getPosition(){ return this->ball->pos; }

	// Function that return velocity
	Vec getVelocity(){ return this->ball->vel; }

	// Function that return visible situation
	bool getVisible(){ return this->ball->visible; }

	// Function that return airborne situation
	bool getAirborne(){ return this->ball->airborne; }

	// Function that return height
	float getHeight(){ return this->ball->height; }

	// Function that return own situation
	bool getOwn(){ return this->ball->own; }

private:
	int omniCyclesNotVisible;
	int frontCyclesVisible;
	Filter *ballFilter;
	Field* field;

	Ball* ball;
	bool selectMostProbableVisionBall(vector<Ball> ballsVision, struct timeval instant, bool ballHitFront);
	bool selectMostProbableFrontVisionBall(vector<BallFrontSensor> ballsFrontVision, struct timeval instant);
	bool selectShareBall(Ball* shareBall, struct timeval instant);
	void setBallNotVisible();
};

}/* namespace cambada */
#endif /* INTEGRATE_BALL_H_ */
