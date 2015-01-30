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

#ifndef _COMPASS_H_
#define _COMPASS_H_

#include "Angle.h"
#include "WorldStateDefs.h"

namespace cambada {

/**
 * CONVENTION: 
 *		- theNorth is the "orientation" of the left goal
 *		- attack blueGoal is the same to attack theNorth
 **/

class Compass
{
public:
	Compass( geom::Angle theNorth = geom::Angle(0.0) , WSColor color = Blue);
	~Compass();

	void setTheNorth(geom::Angle theNorth);
	geom::Angle getTheNorth();
	geom::Angle getTrueTheNorth();

	void setGoalColor(WSColor color);
	WSColor getGoalColor();

	void update(double compassReadDeg180);
	void updateTrue(double compassReadDeg180);

	geom::Angle getCompass();
	geom::Angle getTrueCompass();

private:

	WSColor goalColor;
	geom::Angle theNorth;
	geom::Angle theCompass;
    	geom::Angle trueCompass;
};
}
#endif // _COMPASS_H_

