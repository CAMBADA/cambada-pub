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

#include "Zones.h"
#include <cassert>
#include <cmath>

using namespace cambada::geom;

namespace cambada {

Zones::Zones(Field* field)
{
    // Make sure config is not a null pointer
    assert( field != NULL );

    double field_length, field_width;
    double penalty_area_lenght, penalty_area_width;

    double zone_slider;

    field_length = field->length;
    field_width  = field->width;

    penalty_area_lenght = field->penaltyAreaLength;
    penalty_area_width  = field->penaltyAreaWidth;

    zone_slider = field->zoneSlider;

    this->zoneLength[ Zone1 ] = penalty_area_lenght;
    this->zoneWidth [ Zone1 ] = field_width * 0.5;

    this->zoneLength[ Zone2 ] = field_length * 0.5 - penalty_area_lenght + zone_slider;
    this->zoneWidth [ Zone2 ] = (field_width - (penalty_area_width + 1.0) ) * 0.5;

    this->zoneLength[ Zone3 ] = field_length * 0.5 - zone_slider;
    this->zoneWidth [ Zone3 ] = this->zoneWidth[ Zone2 ];

    this->zoneLength[ Zone4 ] = this->zoneLength[ Zone2 ];
    this->zoneWidth [ Zone4 ] = field_width * 0.5 -  this->zoneWidth[ Zone2 ];

    this->zoneLength[ Zone5 ] = this->zoneLength[ Zone3 ];
    this->zoneWidth [ Zone5 ] = this->zoneWidth [ Zone4 ];

    this->zoneLength[ Zone6 ] = this->zoneLength[ Zone4 ];
    this->zoneWidth [ Zone6 ] = this->zoneWidth [ Zone4 ];

    this->zoneLength[ Zone7 ] = this->zoneLength[ Zone5 ];
    this->zoneWidth [ Zone7 ] = this->zoneWidth [ Zone5 ];

    this->zoneLength[ Zone8 ] = this->zoneLength[ Zone1 ];
    this->zoneWidth [ Zone8 ] = this->zoneWidth [ Zone1 ];

    this->zoneLength[ Zone9 ] = this->zoneLength[ Zone2 ];
    this->zoneWidth [ Zone9 ] = this->zoneWidth [ Zone2 ];

    this->zoneLength[ Zone10 ] = this->zoneLength[ Zone3 ];
    this->zoneWidth [ Zone10 ] = this->zoneWidth [ Zone3 ];


    /// Buid Zone boxes
	//Area points
	Vec topLeft =  Vec( - field_width * 0.5 , field_length * 0.5 );
	double offset = 1.0;

	Vec Area1TopLeft  = topLeft;
	Vec Area1BotRight = Area1TopLeft - Vec( - getZoneWidth(Zone1) , getZoneLength(Zone1) );

	Vec Area2TopLeft  = topLeft 	 - Vec( 0.0 , getZoneLength(Zone1) );
	Vec Area2BotRight = Area2TopLeft - Vec( - getZoneWidth(Zone2) , getZoneLength(Zone2) );

	Vec Area3TopLeft  = Area2TopLeft - Vec( 0.0 , getZoneLength(Zone2) );
	Vec Area3BotRight = Area3TopLeft - Vec( -getZoneWidth(Zone3) , getZoneLength(Zone3) );

	Vec Area4TopLeft  = topLeft 	 - Vec( - getZoneWidth(Zone2), getZoneLength(Zone1) );
	Vec Area4BotRight = Area4TopLeft - Vec( -getZoneWidth(Zone4), getZoneLength(Zone4) );

	Vec Area5TopLeft = Area4TopLeft  - Vec( 0.0 , getZoneLength(Zone4) );
	Vec Area5BotRight = Area5TopLeft - Vec( -getZoneWidth(Zone5) , getZoneLength(Zone5) );

	Vec Area6TopLeft = Area4TopLeft  + Vec( getZoneWidth(Zone4), 0.0 );
	Vec Area6BotRight = Area6TopLeft - Vec( -getZoneWidth(Zone6) , getZoneLength(Zone6) );

	Vec Area7TopLeft = Area6TopLeft  - Vec( 0.0 , getZoneLength(Zone6) );
	Vec Area7BotRight = Area7TopLeft - Vec( - getZoneWidth(Zone7) , getZoneLength(Zone7) );

	Vec topRight = Vec( field_width * 0.5 , field_length * 0.5 );

	Vec Area8TopLeft = topRight - Vec( getZoneWidth(Zone8) , 0.0);
	Vec Area8BotRight = Area8TopLeft - Vec( -getZoneWidth(Zone8) , getZoneLength(Zone8) );

	Vec Area9TopLeft = topRight 	 - Vec( getZoneWidth(Zone9) , getZoneLength(Zone8) );
	Vec Area9BotRight = Area9TopLeft - Vec( -getZoneWidth(Zone9) , getZoneLength(Zone9) );

	Vec Area10TopLeft  = Area9TopLeft  - Vec( 0.0 , getZoneLength(Zone9) );
	Vec Area10BotRight = Area10TopLeft - Vec( - getZoneWidth(Zone10) , getZoneLength(Zone10) );

	zoneBoxes.push_back( XYRectangle( Area1TopLeft + Vec(-offset,offset) , Area1BotRight ) );
	zoneBoxes.push_back( XYRectangle( Area2TopLeft + Vec(-offset,0.0), Area2BotRight ) );
	zoneBoxes.push_back( XYRectangle( Area3TopLeft + Vec(-offset,0.0), Area3BotRight + Vec( 0.0 , -offset ) ) );
	zoneBoxes.push_back( XYRectangle( Area4TopLeft , Area4BotRight ) );
	zoneBoxes.push_back( XYRectangle( Area5TopLeft , Area5BotRight  + Vec(0.0 , -offset) ) );
	zoneBoxes.push_back( XYRectangle( Area6TopLeft , Area6BotRight ) );
	zoneBoxes.push_back( XYRectangle( Area7TopLeft , Area7BotRight + Vec(0.0 , -offset) ) );
	zoneBoxes.push_back( XYRectangle( Area8TopLeft + Vec( 0.0, offset ) , Area8BotRight + Vec(offset, 0.0) )  );
	zoneBoxes.push_back( XYRectangle( Area9TopLeft , Area9BotRight + Vec(offset, 0.0) ) );
	zoneBoxes.push_back( XYRectangle( Area10TopLeft , Area10BotRight + Vec(offset,-offset) ) );

}

double Zones::getZoneLength(Zone zone){
    return this->zoneLength[zone];
}

double Zones::getZoneWidth(Zone zone){
    return this->zoneWidth[zone];
}

} /* namespace cambada */
