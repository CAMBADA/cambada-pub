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

#include "ObstacleHandler.h"

using namespace cambada::geom;

namespace cambada {

ObstacleHandler::ObstacleHandler()
{}

ObstacleHandler::ObstacleHandler(WorldState* world)
{
	this->world = world;
}

ObstacleHandler::~ObstacleHandler()
{
	fprintf(stderr,"HANDLER DESCTRUCTOR!!!!!!!!!");
}

void ObstacleHandler::defineRtdbTime(unsigned int infoAge[], int length)
{
	for (int i=0; i<length; i++)
		rtdbInfoAge[i] = infoAge[i];
}

vector<Obstacle> ObstacleHandler::getObstacles()
{
	return obstacles;
}

vector<Obstacle> ObstacleHandler::getSharedObstacles()
{
	return sharedObstacles;
}

vector<Obstacle> ObstacleHandler::getTrackedObstacles()
{
	vector<Obstacle> returnVector;
	Obstacle temp;

	for (unsigned int i=0; i<identifiedMates.size(); i++)
	{
		returnVector.push_back( *(identifiedMates.at(i)) );
	}

	for (unsigned int i=0; i<tracksList.size(); i++)
	{
		temp.clear();
		temp.obstacleInfo.absCenter = tracksList.at(i).getFilterPosition();
		temp.obstacleInfo.id = tracksList.at(i).getID();
		temp.obstacleWidth=0.5;
		Vec limitCenter=world->abs2rel(temp.obstacleInfo.absCenter);
		temp.limitCenter=limitCenter.setLength(limitCenter.length() - 0.25);
		temp.rightPoint=limitCenter.rotate_three_quarters(); //-90 degrees
		temp.leftPoint=limitCenter.rotate_quarter(); //90 degrees
		returnVector.push_back(temp);
	}

	return returnVector;
}



/** ///////////////////////////////////////////////////////////////////////////
// This function is responsible for identifying the obstacles, starting by   //
// dividing possible obstacle cluster into singular ones to afterwards make  //
// a geometrically circle analysis for matching of obstacles with team mates //
/////////////////////////////////////////////////////////////////////////////*/
void ObstacleHandler::identifyObstacles()
{
//	struct timeval initTime;
//  gettimeofday( &initTime , NULL );

	vector<Obstacle*> singleObstacles;
	vector<Obstacle*>::iterator obstIterator;
	double obstDist, obstCoord;
	
	//Clear the vector of ordered obstacles from last cycle
	orderedObstacles.clear();
	identifiedMates.clear();

	/* Keep original obstacles size, so the new ones created don't change the counting.*/
	unsigned int originalSize = obstacles.size();

//	fprintf(stderr,"\nOBST TOTAL: %d, inside field margin: %f, max (x,y) positions: %f, %f\n",originalSize,(*world->config->getField("side_band_width"))/1000.0 - OBSTACLE_RADIUS,world->getFieldHalfWidth()+((*world->config->getField("side_band_width"))/1000.0 - OBSTACLE_RADIUS), world->getFieldHalfLength()+((*world->config->getField("side_band_width"))/1000.0 - OBSTACLE_RADIUS));
	/*Select obstacles that are candidates for being robots and separate multiple obstacles*/
	for ( unsigned int i = 0; i < originalSize; i++ )
	{
		/*If the obstacle is within the maximum defined distance, has the minimum defined size and is inside the surrounding field protection...*/
		if (	((obstDist = obstacles[i].limitCenter.length()) <= OBSTACLE_MAX_DISTANCE) &&
				(obstacles[i].obstacleWidth > MIN_OBST_SIZE) &&
				(world->getField()->isInside(world->rel2abs(obstacles[i].limitCenter), (world->config->getField("side_band_width"))/1000.0)) )
		{
			/* If the obstacle is smaller than the defined size for a robot, put it directly in the list to identify*/
			if ( obstacles[i].obstacleWidth < (OBSTACLE_RADIUS*2.0 + 1.0/*+ getErrorMargin(obstDist)*/) )
			{
				if ( singleObstacles.empty() )
				{
					singleObstacles.push_back( &(obstacles[i]) );
				}
				else
				{
					for ( obstIterator = singleObstacles.begin(); obstIterator < singleObstacles.end(); obstIterator++ )
					{
						if ( obstDist < (*obstIterator)->limitCenter.length() )
						{
							singleObstacles.insert(obstIterator, &(obstacles[i]) );
							break;
						}
					}
					if ( obstIterator == singleObstacles.end() )
						singleObstacles.push_back( &(obstacles[i]) );
				}
				
				/* Insert obstacle on the coordinate ordered obstacle list */
				obstCoord = obstacles[i].obstacleInfo.absCenter.x + obstacles[i].obstacleInfo.absCenter.y;
				if ( orderedObstacles.empty() )
				{
					orderedObstacles.push_back( &(obstacles[i]) );
				}
				else
				{
					for ( obstIterator = orderedObstacles.begin(); obstIterator < orderedObstacles.end(); obstIterator++ )
					{
						if ( obstCoord < ( (*obstIterator)->obstacleInfo.absCenter.x + (*obstIterator)->obstacleInfo.absCenter.y ) )
						{
							orderedObstacles.insert(obstIterator, &(obstacles[i]) );
							break;
						}
					}
					if ( obstIterator == orderedObstacles.end() )
						orderedObstacles.push_back( &(obstacles[i]) );
				}
			}
			else
			{	/*if the obstacle is bigger, analyze it's size and separate it in the several single obstacles to add to the list to identify*/

				/* This block tests if there is an obstacle whose limits are at different distances from this robot, to avoid obstacles seen in perspective to become too wide (obstacle convoy bug) */
				//TODO ???? MAYBE NOW WITH THE OBSTACLES BUILT HERE, THERE IS NO NEED FOR THIS VERIFICATION AS THE MERGE OBSTACLES WILL NOT CREATE LARGE DISPLACEMENTS ALONG THE SCANLINE
				if ( fabs(obstacles[i].leftPoint.length()-obstacles[i].rightPoint.length()) > 1.5*(OBSTACLE_RADIUS*2.0) ) {
					if ( obstacles[i].leftPoint.length() < obstacles[i].rightPoint.length() ){
						obstacles[i].limitCenter = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( (OBSTACLE_RADIUS*2.0) * 0.5 );
						obstacles[i].rightPoint = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( OBSTACLE_RADIUS*2.0 );
						obstacles[i].obstacleInfo.absCenter = world->rel2abs( obstacles[i].limitCenter.setLength(obstacles[i].limitCenter.length() + OBSTACLE_RADIUS));
						obstacles[i].obstacleWidth = OBSTACLE_RADIUS*2.0;
					} else {
						obstacles[i].limitCenter = obstacles[i].rightPoint + (obstacles[i].leftPoint -obstacles[i].rightPoint).setLength( (OBSTACLE_RADIUS*2.0) * 0.5 );
						obstacles[i].leftPoint = obstacles[i].rightPoint + (obstacles[i].leftPoint -obstacles[i].rightPoint).setLength( OBSTACLE_RADIUS*2.0 );
						obstacles[i].obstacleInfo.absCenter = world->rel2abs( obstacles[i].limitCenter.setLength(obstacles[i].limitCenter.length() + OBSTACLE_RADIUS));
						obstacles[i].obstacleWidth = OBSTACLE_RADIUS*2.0;
					}

					/* Insert the resized obstacle on the single obstacles list */
					obstDist = obstacles[i].limitCenter.length();
					if ( singleObstacles.empty() )
					{
						singleObstacles.push_back( &(obstacles[i]) );
					}
					else
					{
						for ( obstIterator = singleObstacles.begin(); obstIterator < singleObstacles.end(); obstIterator++ )
						{
							if ( obstDist < (*obstIterator)->limitCenter.length() )
							{
								singleObstacles.insert(obstIterator, &(obstacles[i]) );
								break;
							}
						}
						if ( obstIterator == singleObstacles.end() )
								singleObstacles.push_back( &(obstacles[i]) );
					}
					
					/* Insert obstacle on the coordinate ordered obstacle list */
					obstCoord = obstacles[i].obstacleInfo.absCenter.x + obstacles[i].obstacleInfo.absCenter.y;
					if ( orderedObstacles.empty() )
					{
						orderedObstacles.push_back( &(obstacles[i]) );
					}
					else
					{
						for ( obstIterator = orderedObstacles.begin(); obstIterator < orderedObstacles.end(); obstIterator++ )
						{
							if ( obstCoord < ( (*obstIterator)->obstacleInfo.absCenter.x + (*obstIterator)->obstacleInfo.absCenter.y ) )
							{
								orderedObstacles.insert(obstIterator, &(obstacles[i]) );
								break;
							}
						}
						if ( obstIterator == orderedObstacles.end() )
							orderedObstacles.push_back( &(obstacles[i]) );
					}
				} else {
					/* Estimate how many obstacles */
					int nObst = round(obstacles[i].obstacleWidth / (OBSTACLE_RADIUS*2.0) );
					double separationOffset = obstacles[i].obstacleWidth / nObst;

					/* Create new single obstacles, based on the pivot left position*/
					for ( int a = 0;  a < nObst-1; a++ )
					{
						Obstacle newObstacle;
						newObstacle.limitCenter = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( separationOffset * 1.5 + a * separationOffset );
						newObstacle.leftPoint = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( separationOffset + a * separationOffset );
						newObstacle.rightPoint = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( separationOffset * 2 + a * separationOffset );
						newObstacle.obstacleInfo.absCenter = world->rel2abs( newObstacle.limitCenter.setLength(newObstacle.limitCenter.length() + separationOffset/2.0));
						newObstacle.obstacleWidth = separationOffset;
						obstacles.push_back( newObstacle );

						/* Insert new created obstacle on the single obstacles list */
						obstDist = newObstacle.limitCenter.length();
						if ( singleObstacles.empty() )
						{
							singleObstacles.push_back( &(obstacles.back()) );
						}
						else
						{
							for ( obstIterator = singleObstacles.begin(); obstIterator < singleObstacles.end(); obstIterator++ )
							{
								if ( obstDist < (*obstIterator)->limitCenter.length() )
								{
									singleObstacles.insert(obstIterator, &(obstacles.back()) );
									break;
								}
							}
							if ( obstIterator == singleObstacles.end() )
									singleObstacles.push_back( &(obstacles.back()) );
						}

						/* Insert new created obstacle on the coordinate ordered obstacle list */
						obstCoord = obstacles[i].obstacleInfo.absCenter.x + obstacles[i].obstacleInfo.absCenter.y;
						if ( orderedObstacles.empty() )
						{
							orderedObstacles.push_back( &(obstacles.back()) );
						}
						else
						{
							for ( obstIterator = orderedObstacles.begin(); obstIterator < orderedObstacles.end(); obstIterator++ )
							{
								if ( obstCoord < ( (*obstIterator)->obstacleInfo.absCenter.x + (*obstIterator)->obstacleInfo.absCenter.y ) )
								{
									orderedObstacles.insert(obstIterator, &(obstacles.back()) );
									break;
								}
							}
							if ( obstIterator == orderedObstacles.end() )
								orderedObstacles.push_back( &(obstacles.back()) );
						}

//						fprintf(stderr,"OBST new (INFOR): left: %f, %f right: %f, %f center: %f, %f width: %f\n", newObstacle.leftPoint.x, newObstacle.leftPoint.y, newObstacle.rightPoint.x, newObstacle.rightPoint.y, newObstacle.limitCenter.x, newObstacle.limitCenter.y, newObstacle.obstacleWidth);
					}

					//resize the leftier obstacle as single obstacle
					obstacles[i].limitCenter = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( separationOffset * 0.5 );
					obstacles[i].rightPoint = obstacles[i].leftPoint + (obstacles[i].rightPoint -obstacles[i].leftPoint).setLength( separationOffset );
					obstacles[i].obstacleInfo.absCenter = world->rel2abs( obstacles[i].limitCenter.setLength(obstacles[i].limitCenter.length() + separationOffset/2.0));
					obstacles[i].obstacleWidth = separationOffset;

					/* Insert the resized obstacle on the single obstacles list */
					obstDist = obstacles[i].limitCenter.length();
					if ( singleObstacles.empty() )
					{
						singleObstacles.push_back( &(obstacles[i]) );
					}
					else
					{
						for ( obstIterator = singleObstacles.begin(); obstIterator < singleObstacles.end(); obstIterator++ )
						{
							if ( obstDist < (*obstIterator)->limitCenter.length() )
							{
								singleObstacles.insert(obstIterator, &(obstacles[i]) );
								break;
							}
						}
						if ( obstIterator == singleObstacles.end() )
								singleObstacles.push_back( &(obstacles[i]) );
					}
					
					/* Insert the resized obstacle on the coordinate ordered obstacle list */
					obstCoord = obstacles[i].obstacleInfo.absCenter.x + obstacles[i].obstacleInfo.absCenter.y;
					if ( orderedObstacles.empty() )
					{
						orderedObstacles.push_back( &(obstacles[i]) );
					}
					else
					{
						for ( obstIterator = orderedObstacles.begin(); obstIterator < orderedObstacles.end(); obstIterator++ )
						{
							if ( obstCoord < ( (*obstIterator)->obstacleInfo.absCenter.x + (*obstIterator)->obstacleInfo.absCenter.y ) )
							{
								orderedObstacles.insert(obstIterator, &(obstacles[i]) );
								break;
							}
						}
						if ( obstIterator == orderedObstacles.end() )
							orderedObstacles.push_back( &(obstacles[i]) );
					}
				} //close else within multiple obstacle division which separates big obstacles "vertically" or "horizontally"
			} //close else of multiple obstacle division
		} //close if for minimum size, inside field and maximum distance
	} //close cycle of original obstacles size

//fprintf(stderr,"OBST Single candidates: %d, Ignored as too small: %d\n", singleObstacles.size(), obstacles2.size() - singleObstacles.size() );


/*Identify the obstacles*/
/*Check for all cambadas except myself...*/

				
	double maxStd = getErrorMargin(OBSTACLE_MAX_DISTANCE);
	for( int i = 0 ; i < N_CAMBADAS ; i++ )
	{
		if( ( i != world->getMyIdx() ) && ( world->robot[i].running ) )
		{
			vector<unsigned int> obstsAsTeamIdxs;	/*!< Vector to hold the full list of indexes of singleObstacles that are classified as the current team mate*/
			obstsAsTeamIdxs.clear();

			/*Make circle around cambada with a given margin, according to the distance*/
			double std = getErrorMargin( (world->me->pos - world->robot[i].pos).length() );
			double margin = (std/maxStd) * (OBSTACLE_RADIUS/2.0);
//			fprintf(stderr,"OBST margin for agent %d is %f (maxStd: %f, std: %f)\n",i,margin, maxStd, std);
			Circle teamMate = Circle(world->robot[i].pos, OBSTACLE_RADIUS + margin);
			Circle teamMateProj = Circle(world->robot[i].pos + world->robot[i].vel * (rtdbInfoAge[i] / 1000.0), OBSTACLE_RADIUS);

			/*Test obstacles to be inside the mate circle*/
			for ( unsigned int j = 0; j < singleObstacles.size(); j++ )
			{
				if ( singleObstacles[j]->obstacleInfo.id == 0 )
				{
//					double obstDist = world->abs2rel(singleObstacles[j]->obstacleInfo.absCenter).length();
//					double std = getErrorMargin(obstDist);

					/*In first versions, compare for: (1) intersection points between mate and obstacle circle, (2) if obstacle center is inside mate or (3) intersection between obstacle circle and mate velocity extended circle*/
					//if ( (intersect(teamMate, obstCircle).size() > 0) || (teamMate.is_inside(singleObstacles[j]->obstacleInfo.absCenter)) || (intersect(teamMateProj, obstCircle).size() > 0) )

					/*2014_03_28: Test obstacle center or limit center to be inside either the team mate or the team mate projection.*/
					if ( (teamMate.is_inside(singleObstacles[j]->obstacleInfo.absCenter)) || (teamMateProj.is_inside(singleObstacles[j]->obstacleInfo.absCenter)) || (teamMate.is_inside(world->rel2abs(singleObstacles[j]->limitCenter))) || (teamMateProj.is_inside(world->rel2abs(singleObstacles[j]->limitCenter))) )
					{
						singleObstacles[j]->obstacleInfo.id = i + 1;
						obstsAsTeamIdxs.push_back(j);	/*!< Current obstacle is inside team mate, keep its index on the list*/
					}
				}
			}

			// Merge all obstacles identified as team mates into the first one and delete all the others
			if (obstsAsTeamIdxs.size() > 0)
			{
				Vec meanLimitLeft, meanLimitCenter, meanLimitRight, meanAbsCenter;
				for ( unsigned int identified=0; identified < obstsAsTeamIdxs.size(); identified++ )
				{
					meanLimitLeft += singleObstacles.at( obstsAsTeamIdxs.at(identified) )->leftPoint;
					meanLimitCenter += singleObstacles.at( obstsAsTeamIdxs.at(identified) )->limitCenter;
					meanLimitRight += singleObstacles.at( obstsAsTeamIdxs.at(identified) )->rightPoint;
					meanAbsCenter += singleObstacles.at( obstsAsTeamIdxs.at(identified) )->obstacleInfo.absCenter;
				}
				meanLimitLeft /= obstsAsTeamIdxs.size();
				meanLimitCenter /= obstsAsTeamIdxs.size();
				meanLimitRight /= obstsAsTeamIdxs.size();
				meanAbsCenter /= obstsAsTeamIdxs.size();

				for ( unsigned int n = obstsAsTeamIdxs.size()-1; n > 0; n-- )
				{
					singleObstacles.erase( singleObstacles.begin()+obstsAsTeamIdxs.at(n) );
				}

				singleObstacles.at( obstsAsTeamIdxs.at(0) )->leftPoint = meanLimitLeft;
				singleObstacles.at( obstsAsTeamIdxs.at(0) )->limitCenter = meanLimitCenter;
				singleObstacles.at( obstsAsTeamIdxs.at(0) )->rightPoint = meanLimitRight;
				singleObstacles.at( obstsAsTeamIdxs.at(0) )->obstacleInfo.absCenter = meanAbsCenter;
				identifiedMates.push_back( singleObstacles.at( obstsAsTeamIdxs.at(0) ) );
			}
		}
	}

	/*Merge the team mates obstacles*/
	mergeObstacles();

	trackObstacles();

	/*Check how many obstacles fullfilled the requisites and fill the obstacles rtdb*/
	vector<Obstacle> trackedObstacles = getTrackedObstacles();
	unsigned int num_shared = trackedObstacles.size();
	if ( num_shared > MAX_SHARED_OBSTACLES )
		num_shared = MAX_SHARED_OBSTACLES;

	for ( unsigned int a = 0; a < num_shared; a++ )
	{
		world->me->obstacles[a] = trackedObstacles.at(a).obstacleInfo;
//		globalObstacles.push_back(*orderedObstacles.at(a));
	}

	world->me->nObst = num_shared;
}




/** ///////////////////////////////////////////////////////////////////////////
// This function is responsible for getting the obstacles shared by the team //
//  mates and accept the ones that are to be accepted                        //
// Current criteria is accept obstacles that are shared by at least 2 others //
/////////////////////////////////////////////////////////////////////////////*/
void ObstacleHandler::mergeObstacles()
{
	vector<Obstacle> unconfirmedObstacles;
	Obstacle temp;
	double maxStd = getErrorMargin(OBSTACLE_MAX_DISTANCE);

	for ( int i = 0; i < N_CAMBADAS; i++ )
	{
		if( ( i != (Whoami()-1) ) && (rtdbInfoAge[i] < 1000) )
		{
			
			for ( unsigned int a = 0; a < world->robot[i].nObst; a++ )
			{	/*Check all the obstacles shared by the team mate (obstacles on the cambada array are ObstacleInfo)*/
				/*Make circle around obstacle*/
				double obstDist = world->abs2rel(world->robot[i].obstacles[a].absCenter).length();
				
				//Ignore if team mates report an obstacle very close to me, they can be seeing me; also ignore if the team mate obstacle is identified as a team mate
				if ( obstDist < 1.5 || world->robot[i].obstacles[a].isTeamMate() )
					continue;

				double std = getErrorMargin( obstDist );
				double margin = (std/maxStd) * (OBSTACLE_RADIUS/2.0);
				Circle mateObst = Circle(world->robot[i].obstacles[a].absCenter, OBSTACLE_RADIUS + margin );
				bool mateObstExists = false;
				bool mateObstConfirmed = false;

				/*Compare my obstacles with the current team mate obstacles*/
				for ( unsigned int b = 0; b < obstacles.size(); b++ )
				{	/*If my obstacle is inside the team mate obstacle area, mark the mate obstacle as existing so it is not added*/
					if ( mateObst.is_inside(obstacles[b].obstacleInfo.absCenter) )
					{
						mateObstExists = true;
						break;
					}
				}

				if ( !mateObstExists )
				{	/*If my own obstacles did not match the mate obstacle, test the already shared ones, maybe it exists already*/
					for ( unsigned int c = 0; c < unconfirmedObstacles.size(); c++ )
					{
						if ( mateObst.is_inside(unconfirmedObstacles[c].obstacleInfo.absCenter) )
						{
							mateObstExists = true;
							mateObstConfirmed = true;
						}
					}
				}

				if ( !mateObstExists )
				{	/*If the obstacle doesn't exist on the unconfirmed list already, then add it to the list*/
					temp.clear(); temp.obstacleInfo = world->robot[i].obstacles[a];
					unconfirmedObstacles.push_back(temp);
				}

				if ( mateObstConfirmed )
				{
					unsigned int alreadyShared = sharedObstacles.size();
					if ( alreadyShared > 0 )
					{
						for ( unsigned int d = 0; d < alreadyShared; d++ )
						{
							if ( !mateObst.is_inside(sharedObstacles[d].obstacleInfo.absCenter) )
							{
								/*If the obstacle already existed only on the unconfirmed list and current team mate confirmated its existence, add it to shared obstacles*/
								temp.clear(); temp.obstacleInfo = world->robot[i].obstacles[a];
								temp.obstacleWidth=0.5;
								Vec limitCenter=world->abs2rel(temp.obstacleInfo.absCenter);
								temp.limitCenter=limitCenter.setLength(limitCenter.length() - 0.25);
								temp.rightPoint=limitCenter.rotate_three_quarters(); //-90 degrees
								temp.leftPoint=limitCenter.rotate_quarter(); //90 degrees
								sharedObstacles.push_back(temp);
								break;
							}
						}
					}
					else
					{
						temp.clear(); temp.obstacleInfo = world->robot[i].obstacles[a];
						temp.obstacleWidth=0.5;
						Vec limitCenter=world->abs2rel(temp.obstacleInfo.absCenter);
						temp.limitCenter=limitCenter.setLength(limitCenter.length() - 0.25);
						temp.rightPoint=limitCenter.rotate_three_quarters(); //-90 degrees
						temp.leftPoint=limitCenter.rotate_quarter(); //90 degrees
						sharedObstacles.push_back(temp);
					}
				}
			}	//close cycle of current mate obstacle list
		}	//close condition of running time and not myself
	}	//close cycle of mates
}

double ObstacleHandler::getErrorMargin(double distance)
{
	return DIST_A*distance*distance + DIST_B*distance + DIST_C;
}

/** ///////////////////////////////////////////////////////////////////////////
// This function is responsible for building creating the obstacles from the //
// collection of black visual points, through analysis of distance thresholds//
/////////////////////////////////////////////////////////////////////////////*/
void ObstacleHandler::buildAndUpdateObstacles(Vec points[], int nPoints)
{
	//Register the current time
	gettimeofday( &currentTime , NULL );
	world->pointsRighOfBall = 0;
	world->pointsLeftOfBall = 0;
	
	#define MERGE_CENTERS 0
	obstacles.clear();
	sharedObstacles.clear();
//	globalObstacles.clear();

	if (nPoints<=1)		//if there's only one point (or none), there are no obstacles to consider: return
		return;

	Obstacle tempObst;
	bool noCurrentObstacle = false;
	int firstPoint = 0;
	float meanPointDist = -1.0;

	//start by introducing the 1st valid point as the beggining of the first obstacle
	while ( !world->getField()->isInside(world->rel2abs(points[firstPoint])) )
	{
		firstPoint++;
		if (firstPoint >= nPoints)		//if the firstPoint overflows (or if it is the last one, meaning there is only one point): return, no obstacles should be considered
			return;
	}
	tempObst.rightPoint = points[firstPoint];
	tempObst.leftPoint = points[firstPoint];

	int obstNpoints=1;//,out=0;
	int ignoredLastIndexes = 0;
	//Iteratively build each obstacle "blob"
	for (int i=firstPoint+1; i < nPoints; i++)
	{
		bool obstacleOnBall = false;
		// Activate or deactivate the ignoring of obstacle points in front of the ball
		if ( world->me->ball.own )
		{
			double sliceRadius = world->me->ball.posRel.length();
			Angle ballAngle = world->me->ball.posRel.angle();

			Angle alpha = Angle(atan( 0.4 / sliceRadius ));
			Arc rightSide = Arc( Vec::zero_vector, sliceRadius+0.7, ballAngle - alpha, ballAngle );
			Arc leftSide = Arc( Vec::zero_vector, sliceRadius+0.7, ballAngle, ballAngle + alpha );

			float ball_pointDist = points[i].length() - world->me->ball.posRel.length();
			if ( (rightSide.is_inside( points[i] )) && (ball_pointDist < 1.0) )
			{
				world->pointsRighOfBall++;
			}
			else if ( (leftSide.is_inside( points[i] )) && (ball_pointDist < 1.0) )
			{
				world->pointsLeftOfBall++;
			}
		}

		if ( !world->getField()->isInside(world->rel2abs(points[i])) || obstacleOnBall )
		{
			if ( !noCurrentObstacle && (obstNpoints > 1) ) //When a point is ignored for being out, the next point will not be part of the current obstacle, so finish the current obstacle
			{
				if ( ignoredLastIndexes > ALLOWED_N_IGNORED )
				{
					tempObst.limitCenter = tempObst.leftPoint + (tempObst.rightPoint-tempObst.leftPoint)/2.0;	//estimate relative visual center
					tempObst.obstacleWidth = (tempObst.leftPoint-tempObst.rightPoint).length();	//estimate width
					tempObst.obstacleInfo.absCenter = world->rel2abs( tempObst.limitCenter.setLength(tempObst.limitCenter.length()+tempObst.obstacleWidth/2.0));	//estimate absolute geometric center

					obstacles.push_back( tempObst );
					tempObst.clear();
					meanPointDist = -1.0;
					noCurrentObstacle = true;
				}
			}
			else
			{
				if ( ignoredLastIndexes > ALLOWED_N_IGNORED )
				{
					tempObst.clear();
					meanPointDist = -1.0;
					noCurrentObstacle = true;
				}
			}

			ignoredLastIndexes++;
			continue;
		}

		if ( !noCurrentObstacle )	//if there is an obstacle currently being built...
		{
			double dist = (points[i]-points[i-1-ignoredLastIndexes]).length();
			if ( ((meanPointDist == -1.0) && (dist < BLACK_BLOB_THRESHOLD)) ||
				( (meanPointDist > 0.0) && (dist < (meanPointDist*MEAN_POINT_DISTANCE_FACTOR)) )
				)
			{	//current point is the left point
				if (meanPointDist == -1.0)
				{
					meanPointDist = dist;
				}
				else
				{	//obstNpoints-1 because the number of distances for the mean is actually one less than the number of points
					meanPointDist = ( (obstNpoints-1)*meanPointDist+dist)/obstNpoints;
				}
				ignoredLastIndexes = 0;
				obstNpoints++;
				tempObst.leftPoint = points[i];
				if ( i==nPoints-1 )	//current point is the last on the list: immediately finish the obstacle being built
				{
					tempObst.limitCenter = tempObst.leftPoint + (tempObst.rightPoint-tempObst.leftPoint)/2.0;	//estimate relative visual center
					tempObst.obstacleWidth = (tempObst.leftPoint-tempObst.rightPoint).length();	//estimate width
					tempObst.obstacleInfo.absCenter = world->rel2abs( tempObst.limitCenter.setLength(tempObst.limitCenter.length()+tempObst.obstacleWidth/2.0));	//estimate absolute geometric center
					
					obstacles.push_back( tempObst );
					tempObst.clear();

				}
			} else
			{	//current point is a new obstacle, finish the previous and start the new one
				if (obstNpoints > 1)	//if the previous obstacle is one single point, ignore it
				{
					//finish current obstacle
					tempObst.limitCenter = tempObst.leftPoint + (tempObst.rightPoint-tempObst.leftPoint)/2.0;	//estimate relative visual center
					tempObst.obstacleWidth = (tempObst.leftPoint-tempObst.rightPoint).length();	//estimate width
					tempObst.obstacleInfo.absCenter = world->rel2abs( tempObst.limitCenter.setLength(tempObst.limitCenter.length()+tempObst.obstacleWidth/2.0));	//estimate absolute geometric center

					obstacles.push_back( tempObst );

				}
				tempObst.clear();

				//begin next obstacle
				tempObst.rightPoint = points[i];
				tempObst.leftPoint = points[i];
				obstNpoints = 1;
				meanPointDist = -1.0;
				noCurrentObstacle = false;
				ignoredLastIndexes = 0;
			}
		}
		else	//if currently there is no obstacle being built, initialize one
		{
			tempObst.rightPoint = points[i];
			tempObst.leftPoint = points[i];
			ignoredLastIndexes = 0;
			obstNpoints = 1;
			meanPointDist = -1.0;
			noCurrentObstacle = false;
			ignoredLastIndexes = 0;
		}
	}

	if ( obstacles.size() > 1 )
	{
		//consider the angular discontinuity and check if a merge of first and last obstacle is needed
		if ( (obstacles.at(0).rightPoint - obstacles.at(obstacles.size()-1).leftPoint).length() < BLACK_BLOB_THRESHOLD )
		{
			obstacles.at(0).rightPoint = obstacles.at(obstacles.size()-1).rightPoint;	//merge the obstacles by expanding the first
			obstacles.at(0).limitCenter = obstacles.at(0).leftPoint + (obstacles.at(0).rightPoint - obstacles.at(0).leftPoint)/2.0;	//estimate relative visual center
			obstacles.at(0).obstacleWidth = (obstacles.at(0).leftPoint-obstacles.at(0).rightPoint).length();	//estimate width
			obstacles.at(0).obstacleInfo.absCenter = world->rel2abs( obstacles.at(0).limitCenter.setLength( obstacles.at(0).limitCenter.length()+obstacles.at(0).obstacleWidth/2.0));	//estimate absolute geometric center
			obstacles.pop_back();
		}
	}

	identifyObstacles();
}





/** ///////////////////////////////////////////////////////////////////////////
// This function is responsible for tracking the obstacles. It feeds the     //
// existent tracks, kills them or create new ones                            //
/////////////////////////////////////////////////////////////////////////////*/
void ObstacleHandler::trackObstacles()
{
	bool observationUsed;

	vector<ObstaclePositionKalman>::iterator tracksIterator;
	
	unsigned int currentTracks = tracksList.size();
	
//	ObstaclePositionKalman tempK;
	
	if ( currentTracks > 0 )
	{
		for(tracksIterator = tracksList.begin(); tracksIterator < tracksList.end(); tracksIterator++)
		{

			//Check if the track is running only prediction for too much cycles and eliminate it if true
			if ( tracksIterator->getOnlyPredictionCount() > (int)(3*33/MOTION_TICK + 0.5) )
			{
				tracksList.erase(tracksIterator);
			}
			//Execute prediction phase of the current track
			else
			{
				tracksIterator->update_PredictPhase(currentTime.tv_sec*1000 + currentTime.tv_usec/1000);
			}
		}
	}


	for (unsigned int ordObst = 0; ordObst < orderedObstacles.size(); ordObst++)
	{
		if ( orderedObstacles.at(ordObst)->obstacleInfo.id == 0)
		{
			observationUsed = false;
			if ( currentTracks > 0 )
			{
				for ( tracksIterator = tracksList.begin(); tracksIterator < tracksList.end(); tracksIterator++ )
				{
					Vec trackPosition = tracksIterator->getFilterPosition();
					Vec currObstPos = orderedObstacles.at(ordObst)->obstacleInfo.absCenter;

					if ( (currObstPos - trackPosition).length() < 0.5)
					{
					
						tracksIterator->setNoise( getErrorMargin( world->abs2rel(currObstPos).length() ) );
						tracksIterator->update_ObservationPhase(currObstPos);
						observationUsed = true;
						break;
					}
				}

				//Current obstacle was not fit to any existing track, create a new one
				if  ( !observationUsed )
				{
					ObstaclePositionKalman tempK;
					tempK.update_PredictPhase(currentTime.tv_sec*1000 + currentTime.tv_usec/1000);
					tempK.update_ObservationPhase(orderedObstacles.at(ordObst)->obstacleInfo.absCenter);
					tempK.setID();
					tracksList.push_back(tempK);
				}
			}
			else
			{
				ObstaclePositionKalman tempK;
				tempK.update_PredictPhase(currentTime.tv_sec*1000 + currentTime.tv_usec/1000);
				tempK.update_ObservationPhase(orderedObstacles.at(ordObst)->obstacleInfo.absCenter);
				tempK.setID();
				tracksList.push_back(tempK);
			}
		}
	}
}

}//Close namespace
