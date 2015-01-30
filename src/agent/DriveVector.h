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

#ifndef DRIVEVECTOR_H_
#define DRIVEVECTOR_H_

#include "Vec.h"
#include "WorldState.h"
#include "SlidingWindow.h"
#include "LinRegression.h"
#include "KickerConf.h"

namespace cambada {

enum GrabberType {
	GRABBER_OFF=0,
	GRABBER_ON,
	GRABBER_DEFAULT
};

class DriveVector {
public:
	DriveVector(WorldState* world);
	DriveVector(WorldState* world, double vX, double vY, double vA, unsigned char k=0, GrabberType g=GRABBER_OFF);
	void allOff();
	void motorsOff();
	virtual ~DriveVector();

	float velX, velY, velA;		// X,Y and Angular velocities
	unsigned char kickPower;	// Kicker device power
	GrabberType grabber;		// Grabber device enable
	bool smallAdjustment;		// Indicates a small adjustment to the lowlevel => constant accel will be applied
	int passedCounter;			// robot passed the ball # cycles ago
	int kickedCounter;			// robot kicked the ball # cycles ago

	void pass( double distance );
	void kickOld( double distance );
	void kick( double distance, double height = -2014.0 );
	void grabberControl();

	float getLinearVel();
	void setLinearVel(float absLinearVel);
	void limitVel(float maxLinVel, float maxAngVel);

private:
	WorldState* world;			// Worldstate pointer
	KickerConf* kickConf;		// Kicker configuration
	Timer ballNotVisibleTimer;		// ball not visible timer

};

} /* namespace cambada */
#endif /* DRIVEVECTOR_H_ */
