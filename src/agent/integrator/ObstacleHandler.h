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

#ifndef OBSTACLE_HANDLER_H
#define OBSTACLE_HANDLER_H

#include "WorldState.h"
#include "WorldStateDefs.h"
#include "Vec.h"
#include "ObstaclePositionKalman.h"

//definitions for obstacle integration
#define MIN_OBST_SIZE 0.10				/*!<Minimum size of an obstacle to be considered for identification.*/
#define OBSTACLE_MAX_DISTANCE 6.0		/*!<Maximum distance at which the obstacles will be considered for identification.*/
#define BLACK_BLOB_THRESHOLD 0.40		/*!<Default threshold used to decide when a point should be part of the next obstacle (and to decide if the first and last obstacles must be merged.*/
#define MEAN_POINT_DISTANCE_FACTOR 1.5	/*!<Used to decide whether a black point is added to the current obstacle or not, using this factor * the mean distance between the points of the current obstacle.*/
#define MERGE_CENTER_PERCENTAGE 0.9		/*!<Percentage of the obstacle width used to evaluate if the center is too close to the last obstacle, for merging purposes.*/
#define ALLOWED_N_IGNORED -1 			/*!<Number of allowed ignored points in the middle of an obstacle construction*/

#define DIST_A 0.01 //0.04
#define DIST_B -0.01 //-0.02
#define DIST_C 0.039 //0.016

namespace cambada {

class ObstacleHandler
{
	public:
		ObstacleHandler();
		ObstacleHandler(WorldState* world);
		~ObstacleHandler();

		void defineRtdbTime(unsigned int infoAge[], int length = N_CAMBADAS);
		void buildAndUpdateObstacles(geom::Vec points[], int nPoints);
		vector<Obstacle> getObstacles();
		vector<Obstacle> getTrackedObstacles();
		vector<Obstacle> getSharedObstacles();

	private:
		struct timeval currentTime;
		
		WorldState* world;
		vector<Obstacle> obstacles;
		vector<Obstacle> sharedObstacles;
		
		vector<Obstacle*> orderedObstacles;
		vector<Obstacle*> identifiedMates;

//		vector<Obstacle> globalObstacles;
		vector<ObstaclePositionKalman> tracksList;

		unsigned int rtdbInfoAge[N_CAMBADAS];

		void identifyObstacles();
		void mergeObstacles();
		double getErrorMargin(double distance);
		
		void trackObstacles();
		
//#if CREATE_M_POINTS
		FILE*	logPoints;
		FILE*	logBall;
		int		frameCount;
//#endif
};

}//Close namespace

#endif
