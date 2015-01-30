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

#ifndef FIELD_H_
#define FIELD_H_

#include "ConfigXML.h"
#include "Vec.h"
#include "geometry.h"

using namespace cambada::util;

namespace cambada{

class Field {
public:
	Field();
	Field( ConfigXML* config );

	// Field dimensions
	float width,length;
	float halfWidth,halfLength;

	// Goal Area
	float goalAreaLength, goalAreaWidth;
	float goalAreaHalfLength, goalAreaHalfWidth;

	// Penalty Area
	float penaltyAreaWidth, penaltyAreaLength;
	float penaltyAreaHalfWidth, penaltyAreaHalfLength;

	float penaltyMarkerDistance;

	// Ball dimensions
	float ballDiameter, ballRadius;

	// Line properties
	float borderLineThickness, lineThickness, linesetType;

	// Center circle
	float centerCircleRadius,centerCircleDiameter;

	// Corner arcs
	float cornerArcRadius;

	// Goal Dimensions
	float goalBandWidth, goalHeight, goalLength, goalWidth;
	float goalHalfWidth;

	// Zone Slider
	float zoneSlider;

	// The north
	int theNorth;

	// Our goal
	geom::Vec ourGoal;

	// Their goal
	geom::Vec theirGoal;

	/**
	 * Checks if the given vector is inside field
	 * \param testPost the vector of the position to test
	 * \param outerMargin the margin of error (positive value grows the field)
	 */
	bool isInside(geom::Vec testPos, float outerMargin = 0.0);

	/**
	 * Checks if the vector is inside the opponent goal area
	 * \param testPost the vector of the position to test
	 * \param outerMargin the margin of error (positive value grows the area)
	 */
	bool isInsideTheirGoalArea(geom::Vec testPos, float outerMargin = 0.0);

	/**
	 * Checks if the vector is inside our goal area
	 * \param testPos the vector of the position to test
	 * \param outerMargin the margin of error (positive value grows the area)
	 */
	bool isInsideOurGoalArea(geom::Vec testPos, float outerMargin = 0.0);

	/**
	 * \brief Filters the absolute position to be outside our penalty area
	 */
	void ourPenaltyAreaFilter( geom::Vec& pos , float offset );

	/**
	 * \brief Checks if vec is near our penalty area
	 */
	bool isNearOurPenaltyArea(geom::Vec pointAbs, double distance);

	/**
	 * \brief Filters the absolute position to be outside their goal area
	 */
	void theirGoalAreaFilter( geom::Vec& pos , float offset );

	/**
	 * \brief Filters the absolute position to inside field
	 */
	void filter( geom::Vec& pos , float offset );


private:
	ConfigXML* config;

};

} /* namespace cambada */
#endif /* FIELD_H_ */
