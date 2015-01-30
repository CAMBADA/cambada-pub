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

#include "Formation.h"
#include "Strategy.h"
#include "geometry.h"

#include <fstream>
#include <istream>

using namespace cambada;

namespace cambada {

Formation::Formation( ConfigXML* config, WorldState* world ) 
{
	this->world = world;
	this->config = config;
	//actualPos = world->whoami() ;

}

FormationSBSP::FormationSBSP(void) : Formation()
{
} 

FormationSBSP::FormationSBSP( ConfigXML* config, WorldState* world ) : Formation(config,world) 
{} ;

void FormationSBSP::update()
{
	Vec ball;
	ball = world->getBestBallAbs();

	const float OFFSET = 0.5;

	if(  world->gameState == preOwnKickOff)
	{
		if(world->isBallVisible() == false || ball.length() > 0.3 )
			ball = Vec(0.0,0.0);		
	}

	for( int i = 0 ; i < N_CAMBADAS ; i++ )
	{
		//cerr << " SP " << i << " fid " << strategy->finfo.formationID << " pos "  << pos[i] << "\n";
		strategy->SPosition[i] = pos[i];
		strategy->SPosition[i].x += ball.x * att[i].x;	
		strategy->SPosition[i].y += ball.y * att[i].y;	

		if( world->gameState == postOwnCornerKick || world->gameState == preOwnCornerKick )
		{
			if( strategy->SPosition[i].x < min[i].x+1 ) strategy->SPosition[i].x = min[i].x+1;
			if( strategy->SPosition[i].x > max[i].x-1 ) strategy->SPosition[i].x = max[i].x-1;
		}
		else
		{
			if( strategy->SPosition[i].x < min[i].x ) strategy->SPosition[i].x = min[i].x;
			if( strategy->SPosition[i].x > max[i].x ) strategy->SPosition[i].x = max[i].x;
		}

		if( strategy->SPosition[i].y < min[i].y ) strategy->SPosition[i].y = min[i].y;
		if( strategy->SPosition[i].y > max[i].y ) strategy->SPosition[i].y = max[i].y;

		// not enter in my penaltyArea
		const Vec myPenaltyAreaP1 = 
				Vec( - (config->getField("penalty_area_width"))/2000.0 - OFFSET, 
						- (config->getField("field_length"))/2000.0 + (config->getField("penalty_area_length"))/1000.0 + OFFSET);

		const Vec myPenaltyAreaP2 = 
				Vec( + (config->getField("penalty_area_width"))/2000.0 + OFFSET, 
						- (config->getField("field_length"))/2000.0 );

		XYRectangle myPenaltyArea(myPenaltyAreaP1,myPenaltyAreaP2);

		if( myPenaltyArea.is_inside(strategy->SPosition[i]) )
			strategy->SPosition[i] = myPenaltyArea.adjust(strategy->SPosition[i]);

		// not enter in their penaltyArea
		const Vec theirPenaltyAreaP1 = 
				Vec( -(config->getField("penalty_area_width"))/2000.0 - OFFSET , 
						(config->getField("field_length"))/2000.0 );

		const Vec theirPenaltyAreaP2 = 
				Vec( + (config->getField("penalty_area_width"))/2000.0 + OFFSET, 
						(config->getField("field_length"))/2000.0 - (config->getField("penalty_area_length"))/1000.0 - OFFSET);


		XYRectangle theirPenaltyArea(theirPenaltyAreaP1,theirPenaltyAreaP2);

		if( theirPenaltyArea.is_inside(strategy->SPosition[i]) )
			strategy->SPosition[i] = theirPenaltyArea.adjust(strategy->SPosition[i]);
	}
}	

}
