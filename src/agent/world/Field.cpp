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

#include "Field.h"

using namespace cambada::geom;

namespace cambada{

Field::Field() {}

Field::Field( ConfigXML* config )
{
	this->length = config->getField("field_length")/1000.;
	this->halfLength = this->length/2.;
	this->width = config->getField("field_width")/1000.;
	this->halfWidth = this->width/2.;
	this->goalAreaWidth = config->getField("goal_area_width")/1000.;
	this->goalAreaLength = config->getField("goal_area_length")/1000.;
	this->goalAreaHalfWidth = this->goalAreaWidth/2.;
	this->goalAreaHalfLength = this->goalAreaLength/2.;
	this->penaltyAreaWidth = config->getField("penalty_area_width")/1000.;
	this->penaltyAreaLength = config->getField("penalty_area_length")/1000.;
	this->penaltyAreaHalfWidth = this->penaltyAreaWidth/2.;
	this->penaltyAreaHalfLength = this->penaltyAreaLength/2.;
	this->penaltyMarkerDistance = config->getField("penalty_marker_distance")/1000.0;
	this->ballDiameter = config->getField("ball_diameter")/1000.;
	this->ballRadius = this->ballDiameter/2.;
	this->borderLineThickness = config->getField("border_line_thickness")/1000.;
	this->lineThickness = config->getField("line_thickness")/1000.;
	this->linesetType = config->getField("lineset_type");
	this->centerCircleRadius = config->getField("center_circle_radius")/1000.;
	this->centerCircleDiameter = this->centerCircleRadius*2.;
	this->cornerArcRadius = config->getField("corner_arc_radius")/1000.;
	this->goalBandWidth = config->getField("goal_band_width")/1000.;
	this->goalHeight = config->getField("goal_height")/1000.;
	this->goalLength = config->getField("goal_length")/1000.;
	this->goalWidth = config->getField("goal_width")/1000.;
	this->goalHalfWidth = this->goalWidth/2;
	this->zoneSlider = config->getField("zone_slider")/1000.;
	this->theNorth = config->getField("theNorth");

	this->theirGoal = geom::Vec(0.0 , this->halfLength);
	this->ourGoal = geom::Vec(0.0 , -this->halfLength);

	this->config = config;
}

bool Field::isInside(geom::Vec testPos, float outerMargin){
	if( (fabs(testPos.x) > (halfWidth + outerMargin)) || (fabs(testPos.y) > (halfLength + outerMargin) ) )
		return false;

	return true;
}

bool Field::isInsideTheirGoalArea(geom::Vec testPos, float outerMargin) {

	if ( (fabs(testPos.x) < (goalAreaHalfWidth + 0.25 + outerMargin) )
			&& ( testPos.y > ((halfLength - goalAreaLength) - 0.25 - outerMargin) ) )
		return true;

	return false;
}

bool Field::isInsideOurGoalArea(geom::Vec testPos, float outerMargin) {

	if ( (fabs(testPos.x) < (goalAreaHalfWidth + 0.25 + outerMargin) )
			&& ( testPos.y < ((-halfLength + goalAreaLength) + 0.25 + outerMargin) ) )
		return true;

	return false;
}

void Field::ourPenaltyAreaFilter( Vec& pos , float offset )
{
        const Vec areaP1 =
                        Vec( - (config->getField("penalty_area_width"))/2000.0 - offset,
                                   - (config->getField("field_length"))/2000.0 + (config->getField("penalty_area_length"))/1000.0 + offset);
        const Vec areaP2 =
                       Vec( + (config->getField("penalty_area_width"))/2000.0 + offset,
                                   - (config->getField("field_length"))/2000.0 );

        XYRectangle area(areaP1,areaP2);

        if( area.is_inside(pos) )
                pos = area.adjust(pos);
}

bool Field::isNearOurPenaltyArea(Vec pointAbs, double distance)
{
	const Vec position = pointAbs;
	if ( (fabs(position.x) < (penaltyAreaWidth/2.0 + config->getField("robot_radius")/1000.0 + distance) )
	&& ( position.y < -((halfLength - penaltyAreaLength) - config->getField("robot_radius")/1000.0 - distance) ) )
		return true;

	return false;
}

void Field::theirGoalAreaFilter( geom::Vec& pos , float offset ) {
	const Vec areaP1 =
			Vec( -(config->getField("goal_area_width"))/2000.0 - offset ,
					(config->getField("field_length"))/2000.0 );

	const Vec areaP2 =
			Vec( + (config->getField("goal_area_width"))/2000.0 + offset,
					(config->getField("field_length"))/2000.0 - (config->getField("goal_area_length"))/1000.0 - offset);


	XYRectangle area(areaP1,areaP2);

	if( area.is_inside(pos) )
		pos = area.adjust(pos);
}

void Field::filter( geom::Vec& pos , float offset ) {
	const Vec areaP1 = Vec( -halfWidth -offset , -halfLength -offset );
	const Vec areaP2 = Vec( +halfWidth +offset , +halfLength +offset);

	XYRectangle area(areaP1,areaP2);

	if( area.is_inside(pos) )
		pos = area.adjust(pos);
}

} /* namespace cambada */
