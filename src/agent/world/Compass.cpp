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

#include "Compass.h"
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cambada::geom;

namespace cambada {

Compass::Compass(Angle theNorth, WSColor color)
{
	setTheNorth(theNorth);
	setGoalColor(color);
}

Compass::~Compass()
{
	this->theNorth = Angle::zero;
	this->theCompass = Angle::zero;
	this->trueCompass = Angle::zero;
}

void Compass::setTheNorth(Angle theNorth)
{
	this->theNorth = theNorth;
}

Angle Compass::getTheNorth()
{
	return ((this->goalColor==Blue)? (theNorth+Angle(M_PI)) : theNorth);
}

Angle Compass::getTrueTheNorth()
{
	return theNorth;
}


void Compass::setGoalColor(WSColor color)
{
	this->goalColor = color;
}


WSColor Compass::getGoalColor()
{
	return goalColor;
}

void Compass::update(double compassReadDeg180)
{
	Angle readCompass;
	readCompass.set_deg(compassReadDeg180);

	//theCompass = theNorth - readCompass; // upside down without hd correction
	theCompass = readCompass - theNorth; // hd correction: 29.12.2010
}

void Compass::updateTrue(double compassReadDeg180)
{
	Angle readCompass;
	readCompass.set_deg(compassReadDeg180);

	//theCompass = theNorth - readCompass; // upside down without hd correction
	trueCompass = readCompass - theNorth; // hd correction: 29.12.2010
}


Angle Compass::getCompass()
{
	return ((this->goalColor==Blue)? (theCompass+Angle(M_PI)) : theCompass);
}

Angle Compass::getTrueCompass()
{
	return trueCompass;
}

}
