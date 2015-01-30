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

#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

#include "IntegratePlayer.h"
#include "ObstacleHandler.h"
#include "IntegrateBall.h"
#include "WorldStateDefs.h"
#include "HWcomm_rtdb.h"
#include "WorldState.h"
#include "VisionInfo.h"
#include "CoachInfo.h"
#include "Strategy.h"
#include "Field.h"
#include "Clock.h"
#include "Vec.h"
#include <deque>
#include <iostream>

//definitions for prediction calculus
#define ROBOT_RADIUS 0.24
#define BALL_RADIUS 0.11
#define COLLISION_ERROR 5

//definitions for robot mouth prediction
#define MOUTH_WIDTH           	0.19
#define MOUTH_WIDENESS       	(2 * asin(MOUTH_WIDTH/(2*ROBOT_RADIUS)))
#define ROBOT_BALL_DISTANCE   	0.17
#define APOTHEM               	(ROBOT_RADIUS*cos(MOUTH_WIDENESS))
#define SAGITTA               	(APOTHEM-ROBOT_BALL_DISTANCE)
#define MOUTH_RADIUS          	(((MOUTH_WIDTH*MOUTH_WIDTH)+4*(SAGITTA*SAGITTA))/(8*SAGITTA))
#define MOUTH_ANGLE           	(asin(MOUTH_WIDTH/(2*MOUTH_RADIUS)))

#define LEFT_ARC_END			(atan2(APOTHEM,-MOUTH_WIDTH/2 ))
#define LEFT_ARC_START      	(atan2(MOUTH_RADIUS*cos(MOUTH_ANGLE),-MOUTH_WIDTH/2))

#define RIGHT_ARC_END       	( atan2(MOUTH_RADIUS*cos(MOUTH_ANGLE),-MOUTH_WIDTH/2) )
#define RIGHT_ARC_START         ( atan2(APOTHEM,MOUTH_WIDTH/2) )

//definitions for stuck detection
#define ACCELERATION_THRESHOLD 2.5
#define SPEED_THRESHOLD 0.650	//em percentagem
#define TIME_THRESHOLD 500

//definitions for ball filter and integration
#define USE_FRONT_VISION false
#define BALL_MAX_DISTANCE 8.0

//definitions for debug prints
#define DEBUG_FILTER 0
#define DEBUG_FRONT 0
#define DEBUG_FILE 0
#define DEBUG_ENGAGED 0

#define LOAD_DUMMY_FILES 0
#define CAPTURE_COMPASS 0

#include <iostream>

using namespace std;

namespace cambada {
class Integrator
{
public:
	Integrator( ConfigXML* config, WorldState* world , Strategy* strategy );
	~Integrator();

	void integrate();

private:
	Clock* 				clock;
	WorldState*			world;
	ConfigXML*			config;
	Strategy*			strategy;
	CoachInfo			coach;
	Field*				field;
	VisionInfo			vision;
	FrontVisionInfo		frontVision;
	IntegratePlayer*	integrate_player;
	IntegrateBall*		integrate_ball;
	ObstacleHandler		handleObstacle;
	unsigned int 		cambadaInfoTTL[N_CAMBADAS];
	deque<CMD_Vel> 		buffer;
	int 				receiverIdxForCorridor;

	void loadVision(bool use_front_vision);
	void loadCoach(int coachRtdbID);
	void GetMultiRobotBall(Ball* shareBall);
	void updateGameState();
	void predictNoCollision();

	bool setPieceBallInCorridor();
	bool isStuck();

};

}/* namespace cambada */
#endif /* MYINTEGRATOR_H_ */
