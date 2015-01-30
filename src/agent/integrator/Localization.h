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

#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_

#include "Vec.h"
#include "ConfigXML.h"
#include "WorldStateDefs.h"

namespace cambada {
using namespace geom;

// Struct used on result
struct DATA_LOCALIZATION {
	Vec		position;		// Absolute position
	float	orientation;	// Absolute orientation
	double 	errPos;			// Error associated by position
	double 	errLoc;			// Error associated by localization
	int 	oriEarth;		// Orientation regarding earth
};

class Localization {
public:

	// Default Construtor
	Localization()
	{
		this->data.position = Vec::zero_vector;
		this->data.orientation = 0.0;
		this->data.errPos = 0.0;
		this->data.errLoc = 0.0;
		this->data.oriEarth = 0;
	}

	// Virtual destrutor
	virtual ~Localization(){}

	// Virtual mirror
	virtual void mirror() =0;

	// Virtual integrate
	virtual void integrate(vector<Vec> vision_lines, float lowLevelDx, float lowLevelDy, WSColor goalColor, bool firstTime) =0;

	// Get function that return stuct
	DATA_LOCALIZATION getResult(){ return this->data; }

	// Get function that return posicion
	Vec getPosition(){ return this->data.position; }

	// Get function that return orientation
	float getOrientation(){ return this->data.orientation; }

	// Get function that return error position
	double getErrorPos(){ return this->data.errPos; }

	// Get function that return error localization
	double getErrorLoc(){ return this->data.errLoc; }

	// Get function that return error localization
	int getOrientationEarth(){ return this->data.oriEarth; }

protected:
	DATA_LOCALIZATION data;
};

}/* namespace cambada */
#endif /* LOCALIZATION_H_ */
