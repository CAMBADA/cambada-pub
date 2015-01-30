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

#include "IntegrateBall.h"

//definitions for ball filter and integration
#define DIST_A 			0.01 //0.04
#define DIST_B 			-0.01 //-0.02
#define DIST_C 			0.039 //0.016
#define PARTICLE 		0

//definitions for debug prints
#define DEBUG_FILTER 	0
#define DEBUG_FRONT 	0

namespace cambada {

IntegrateBall::IntegrateBall(Field* world_field, double deviation, struct timeval instant)
{
	this->field = world_field;
	omniCyclesNotVisible = 0;
	frontCyclesVisible = 0;

	ball		= new Ball();
	ballFilter = new KalmanFilter(deviation, instant);
}

IntegrateBall::~IntegrateBall()
{
	delete ball;
	delete ballFilter;
	this->field = NULL;
}

// TODO JLS: added YET ANOTHER parameter for letting this class now that the ball touched the grabber so the filter can be reset because the ball has bounced
// TODO \todo Does it really make sense to have this so much separated from the world state??? I am more and more convinced that it is not worth it this way...
void IntegrateBall::integrate(vector<Ball> ballsVision, vector<BallFrontSensor> ballsFrontVision, Ball* shareBall, struct timeval instant, bool ballHitFront)
{
	// Select most probable ball by vision if exist
	if(selectMostProbableVisionBall(ballsVision, instant, ballHitFront))
		omniCyclesNotVisible = 0;

	// Mantein the last values if omniCyclesNotVisible have a valid value
	else if( omniCyclesNotVisible < ((!ballsFrontVision.empty())? BALL_CYCLES_NOT_VISIBLE_LIMIT_FRONTVISION : BALL_CYCLES_NOT_VISIBLE_LIMIT) )
		omniCyclesNotVisible++;

	// if can not select ball by front vision, select share ball (if exist)
	else if(!selectMostProbableFrontVisionBall(ballsFrontVision, instant))
		if(!selectShareBall(shareBall, instant))
			setBallNotVisible();
}

// JLS: lets propagate the grabber hit flag to finally get it to the function that needs the flag :s
bool IntegrateBall::selectMostProbableVisionBall(vector<Ball> ballsVision, struct timeval instant, bool ballHitFront)
{
	if(ballsVision.empty())
	{
		float persistencePercentage = 0.4;
		float persistenceCycles = BALL_CYCLES_NOT_VISIBLE_LIMIT*persistencePercentage;
		float reductionFactor = ( (persistenceCycles-omniCyclesNotVisible) < 0 )? 0.0 : (persistenceCycles-omniCyclesNotVisible)/(persistenceCycles);
		ball->vel *= reductionFactor;
		return false;
	}

	// Make sure the filter is reset (specially critical for velocity) if the ball source changes (either from shared to mine or from not visible to my ball)
	if ( !ball->visible || !ball->own )
		ballFilter->resetFilter( ballsVision[0].pos, instant );

	/*Sets the noise for the current cycle*/
	double ballDist = ballsVision.at(0).posRel.length();
	ballFilter->setNoise(DIST_A*ballDist*ballDist + DIST_B*ballDist + DIST_C);
	ballFilter->updateFilter( ballsVision[0].pos, instant );

	// Check if new result is valid
	if( !field->isInside(ballFilter->getPosition(), 0.5) )
		ballFilter->resetFilter( ballsVision[0].pos, instant );

	if( ballHitFront )
		ballFilter->resetFilter( ballsVision[0].pos, instant );

	// Update data_ball values
	ball->pos 		= ballFilter->getPosition();
	ball->vel	 	= (ballFilter->getVelocity(omniCyclesNotVisible).length()>=0.2)? ballFilter->getVelocity(omniCyclesNotVisible) : Vec::zero_vector;
	ball->visible 	= true;
	ball->own 	 	= true;

	// Mantein data_ball.airborne
	// data_ball.airborne = false;
	if(!ball->airborne )
		ball->height = 0.0;

	return true;
}

bool IntegrateBall::selectMostProbableFrontVisionBall(vector<BallFrontSensor> ballsFrontVision, struct timeval instant)
{
	int	closerFrontID = -1;
	float shorterFront = 1000.0;		//Used to keep the shorter value of two measures between cycles. USED BOTH FOR ANGLE AND FOR POSITION.

	// Choose the most probable ball visible by the front camera
	if(!ballsFrontVision.empty())
	{
		for (unsigned int i = 0; i < ballsFrontVision.size(); i++ )
		{
			//If the ball was seen by the frontal camera and is inside the field (unknown error in frontal position evaluation, ball can be evaluated farther. Short 0.5m margin helps reduce false positives outside the field)...
			if ( ballsFrontVision.at(i).cyclesNotVisible < 5 )
			{
				// Validate the ball by angular difference over the last ball seen on the omni vision (CURRENTLY ONLY WORKS FOR INCOMING BALLS)
				float diffAngle = fabs( (ballsFrontVision.at(i).position.angle()- ball->pos.angle()).get_deg_180() );
				if( (diffAngle < 10) && (diffAngle < shorterFront) )
				{
					shorterFront = diffAngle;
					closerFrontID = i;
				}
			}
		}
	}

	if(((closerFrontID != -1) && (frontCyclesVisible < 40)))
	{
		frontCyclesVisible++;

		ballFilter->updateFilter(ballsFrontVision.at(closerFrontID).position, instant);

		ball->pos 	 	= ballFilter->getPosition();
		ball->vel 	 	= (ballFilter->getVelocity(omniCyclesNotVisible).length()>=0.2)? ballFilter->getVelocity(omniCyclesNotVisible) : Vec::zero_vector;
		ball->visible  	= true;
		ball->airborne 	= true;
		ball->height 	= ballsFrontVision.at(closerFrontID).height;
		ball->own 	 	= true;

		cout << "[IntegrateBall] Select ball by front vision " << endl;

		return true;
	}
	else
	{
		frontCyclesVisible = 0;
		return false;
	}
}

bool IntegrateBall::selectShareBall(Ball* shareBall, struct timeval instant)
{
	if(!shareBall->visible)
		return false;

	ballFilter->resetFilter(shareBall->pos, instant);

	ball->pos 		= shareBall->pos;
	ball->vel 		= shareBall->vel;
	ball->airborne 	= shareBall->airborne;
	ball->height 	= shareBall->height;
	ball->visible 	= true;
	ball->own 		= false;

	return true;
}

void IntegrateBall::setBallNotVisible()
{
	// Maintain old ball values, but set not visible
	ballFilter->setNotVisible();
	ball->visible 	= false;
	ball->own = false;
}
}/* namespace cambada */
