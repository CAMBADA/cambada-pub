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

#include "Integrator.h"
#include "log.h"
#include <syslog.h>

namespace cambada{

Integrator::Integrator( ConfigXML* config, WorldState* world , Strategy* strategy )
{
	this->world = world;
	this->config = config;
	this->strategy = strategy;

	this->clock = new Clock();
	this->field = world->getField();
	this->handleObstacle = ObstacleHandler(world);

	Field* field = world->getField();
	struct timeval start_instant;
	gettimeofday( &start_instant , NULL );
	this->integrate_ball = new IntegrateBall(field, config->getParam("measure_deviation"), start_instant);
	this->integrate_player = new IntegratePlayer(config); //, lines, coach.playerInfo[myID].goalColor)

	// Initialize buffer
	CMD_Vel v;
	v.vx = (v.vy = (v.va = 0.0));
	while( buffer.size() < ceil((config->getParam("delay")/1000.0)/(config->getParam("cycle_time")/1000.0)))
		buffer.push_back(v);

}

Integrator::~Integrator()
{
	delete clock;

	this->field = NULL;
	this->world = NULL;
	this->config = NULL;
	this->strategy = NULL;

	//this->handleObstacle.~ObstacleHandler();
	this->integrate_ball->~IntegrateBall();
	this->integrate_player->~IntegratePlayer();
}

void Integrator::integrate()
{
	// cerr << "[Integrator] : integrate() " << endl;
	syslog(LOG_DEBUG,"INTEGRATOR NEW CYCLE");

	static bool firstTime = true;
	static bool lastLowLevelRunningInfo = false;
	static struct timeval instant;

	// Get time struct
	gettimeofday( &instant , NULL );

//////////////////////////////////////////////////////////////////////////////////////////////////// Update debug points
	for ( int i=0; i< 4; i++ ) {
		world->me->debugPoints[i]=Vec(100.0,100.0);
	}
	world->me->passLine = Line::def;

//////////////////////////////////////////////////////////////////////////////////////////////////// Update LowLevelInfo
	world->lowlevel.updateInfo();
	for( int i = 0 ; i < N_BATTERIES ; i++ )
		world->me->battery[i] = world->lowlevel.batteryStatus[i]/10.0; // TODO RC2013

	// when we have a off->on power switch transition try to relocate
	bool lowLevelRunningInfo = (world->lowlevel.batteryStatus[1] > 0);
	if( lowLevelRunningInfo == true && lastLowLevelRunningInfo == false )
	{
		firstTime = true;
		syslog(LOG_DEBUG,"Reloc by power switch: !running is %.2d, batValue is %.2d",NOT_RUNNING_VOLTAGE,world->lowlevel.batteryStatus[1]);
	}
	lastLowLevelRunningInfo = lowLevelRunningInfo;
	world->setgrabberTouched(world->lowlevel.rArmTouched || world->lowlevel.lArmTouched);
	world->me->justKicked = world->lowlevel.getJustKicked();

////////////////////////////////////////////////////////////////////////////////////////////////////// Update PlayerInfo
	const int myID = Whoami()-1;
	Robot& player = world->robot[myID];

	// unsigned int  fw,fl;
	// loc->GetFieldDimensions(fw,fl);
	double maxXY = 7000; //(fw/2); //* 0.8;
	double minXY = 10;

	// Filter lines to vision
	vector<Vec> lines;
	loadVision(USE_FRONT_VISION);
	for(int i = 0 ; i < vision.lines.nPoints ; i++)
		if( fabs(vision.lines.point[i].x) <= maxXY && fabs(vision.lines.point[i].y) <= maxXY )
			if( fabs(vision.lines.point[i].x) >= minXY && fabs(vision.lines.point[i].y) >= minXY )
				lines.push_back(vision.lines.point[i]);

	// GET CoachInfo if we want coaching
	int changePositionSNOld = coach.changePositionSN[myID];

	loadCoach((player.coaching)? BASE_STATION : Whoami());

	if( changePositionSNOld != coach.changePositionSN[myID] )
	{
		firstTime = true;
		syslog(LOG_DEBUG,"Reloc by baseReloc: oldSN is %d, new SN is %d",changePositionSNOld,coach.changePositionSN[myID]);
	}

	// Integrate Player
	integrate_player->integrate(lines, world->lowlevel.getDX(), world->lowlevel.getDY(), coach.playerInfo[myID], firstTime);
	if(firstTime) firstTime = false;

	// Update Player Information
	player.pos 			= integrate_player->getPosition();
	player.vel			= integrate_player->getVelocity();
	player.orientation	= integrate_player->getOrientation();
	player.angVelocity	= integrate_player->getAngleVelocity();
	player.goalColor	= integrate_player->getGoalColor();
	player.teamColor	= integrate_player->getTeamColor();
	player.roleAuto		= integrate_player->getRoleAuto();
	player.running		= (integrate_player->getRunning() && lowLevelRunningInfo);
	if(!player.roleAuto) player.role = integrate_player->getRole();
	// TODO this must be set by coach
	//player.number		= coach.playerInfo[myID].number;

	// Clear lines
	lines.clear();


/////////////////////////////////////////////////////////////////////////////////////////////// UPDATE OTHER ROBOTS INFO
	for( int i = 0 ; i < N_CAMBADAS ; i++ )
	{
		if( myID != i )
		{
			int ltime = DB_get( i+1 , ROBOT_WS , &world->robot[i] );
			if( ltime == -1 )
			{
				cerr << "[Integrator] : integrate - db_get ROBOT_WS error" << endl;
				world->robot[i].running = false;
			}

			cambadaInfoTTL[i] = ltime;

			if( cambadaInfoTTL[i] > NOT_RUNNING_TIMEOUT )
				world->robot[i].running = false;

			if( player.teamColor != world->robot[i].teamColor )
				world->robot[i].running = false;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////// UPDATE BALL DATA

	// Filter valid balls by Vision
	Ball visionBall;
	vector<Ball> visionBalls;
	for (int i = 0; i < vision.nBalls; i++ )
	{
		visionBall.posRel = vision.ball[i].position;
		visionBall.pos = world->rel2abs(vision.ball[i].position);
		if ( (visionBall.posRel.length() < BALL_MAX_DISTANCE) && (field->isInside(visionBall.pos, 0.75)) )
			visionBalls.push_back(visionBall);
	}

	// Filter valid balls by Vision
	vector<BallFrontSensor> frontVisionBalls;

	// Get share ball (if exist)
	Ball* shareBall = new Ball();
	GetMultiRobotBall(shareBall);

	/*Keep the original relative position of the used vision ball (Only true while inside ball_integrate candidate 0 is always selected)*/
	if (!visionBalls.empty())
	{
		world->origBallPos = visionBalls.at(0).posRel;
	}

	// Integrate info
	integrate_ball->integrate(visionBalls,frontVisionBalls, shareBall, instant, world->grabberTouched(true));

	world->me->ball.pos 		= integrate_ball->getPosition();
	world->me->ball.vel 		= integrate_ball->getVelocity();
	world->me->ball.visible 	= integrate_ball->getVisible();
	world->me->ball.airborne 	= integrate_ball->getAirborne();
	world->me->ball.height 		= integrate_ball->getHeight();
	world->me->ball.own 		= integrate_ball->getOwn();
	world->me->ball.posRel 		=  world->abs2rel(world->me->ball.pos);
	world->me->ball.hasMoved 	= (world->me->ball.vel.length() >= 0.2);
// 	world->me->ball.distance 	= world->me->ball.posRel.length();
	world->me->ball.engaged 	= (world->me->ball.visible && world->me->ball.own
				&& world->lowlevel.barrierState
				&& world->me->ball.posRel.length() < BALL_ENGAGED_DISTANCE
				&& fabs( world->me->ball.posRel.angleFromY().get_deg_180() ) < BALL_ENGAGED_DEG);


	// Clear aux data
	delete shareBall;
	visionBalls.clear();
	frontVisionBalls.clear();


/////////////////////////////////////////////////////////////////////////////////////////////////////// UPDATE OBSTACLES
 	world->obstacles.clear();
 	world->sharedObstacles.clear();

 	handleObstacle.defineRtdbTime(cambadaInfoTTL);
	handleObstacle.buildAndUpdateObstacles(vision.obstacles.point, vision.obstacles.nPoints);

//	world->obstacles = handleObstacle.getObstacles();
	world->obstacles = handleObstacle.getTrackedObstacles();
	world->sharedObstacles = handleObstacle.getSharedObstacles();


////////////////////////////////////////////////////////////////////////////////////////////////////// UPDATE GAME_STATE
	updateGameState();

////////////////////////////////////////////////////////////////////////////////////////////////////// Predict Avaliable
	// GET last velocity command (used latter for WorldState Prediction)
	CMD_Vel v;
	DB_get( Whoami(),LAST_CMD_VEL,&v);
	buffer.pop_front();
	buffer.push_back(v);

//////////////////////////////////////////////////////////////////////////////////////////////////////////// STUCK TESTS

	world->me->stuck=isStuck();

	world->update();
	if( world->me->role != rGoalie )
		predictNoCollision();

//////////////////////////////////////////////////////////////////////////////////////////////////////////// Close Cycle
	gettimeofday( &instant , NULL );
	world->timeStamp = instant.tv_sec*1000 + instant.tv_usec/1000;
}

void Integrator::loadVision(bool use_front_vision)
{
	// GET VisionInfo
	if( DB_get( Whoami() , VISION_INFO , &vision ) == -1 )
		if( DB_get( Whoami() , VISION_INFO , &vision ) == -1 )
			cerr << "[Integrator] : integrate - db_get VISION_INFO error" << endl;

	if(use_front_vision)
	{
		// GET FrontVisionInfo
		int frontVisionLifeTime = 1000;
		if( (frontVisionLifeTime = DB_get( Whoami() , FRONT_VISION_INFO , &frontVision )) == -1 )
			if( (frontVisionLifeTime = DB_get( Whoami() , FRONT_VISION_INFO , &frontVision )) == -1 )
				cerr << "[Integrator] : integrate - db_get FRONT_VISION_INFO error" << endl;
		if( frontVisionLifeTime >= 0 && frontVisionLifeTime <= 100 ) frontVision.clear();
	}

}

void Integrator::loadCoach(int coachRtdbID)
{
	// Load coach
	int coachLt;
	if( (coachLt=DB_get( coachRtdbID , COACH_INFO , &coach )) == -1 )
		if( (coachLt=DB_get( coachRtdbID , COACH_INFO , &coach )) == -1 )
			cerr << "[Integrator] : integrate - db_get COACH_INFO error" << endl;

	world->coach = coach;  //needed for setplays

	// TODO if there no coach clear finfo information
	// Load formation
	int formationLt;
	if( (formationLt=DB_get( coachRtdbID , FORMATION_INFO , &strategy->finfo )) == -1 )
		if( (formationLt=DB_get( coachRtdbID , FORMATION_INFO , &strategy->finfo )) == -1 )
			cerr << "[Integrator] : integrate - db_get FORMATION_INFO error" << endl;

	world->isFormationCoachAvailable = ( formationLt <= NOT_RUNNING_TIMEOUT );

	if( !world->isFormationCoachAvailable )
	{
		//cerr << "[Integrator] : POS coach not running " << coachLt << endl;
		strategy->finfo.reset();
	}
}

void Integrator::GetMultiRobotBall(Ball* shareBall)
{
	/*
	 * Create list of teammates indexes seeing the ball.
	 * Goalkeeper is excluded, it is the most suscetible to see false balls.
	 */
	deque<int> runningPlayersBallIdx;
	for( int i = 1 ; i < N_CAMBADAS ; i++ )
		if( world->robot[i].running && (i != (Whoami()-1)) )
			if( world->robot[i].ball.visible && (world->robot[i].ball.own))
				runningPlayersBallIdx.push_back( i );

	/*
	 * If there are teammates seeing the ball, choose the one with higher confidence (with the ball closer to itself).
	 * Commented is the possibility to use a mean of the seen balls.
	 */

	if( runningPlayersBallIdx.size() > 0 )
	{
		//calculate the ball seen by the other players
		int chosenAgent=-1;
		double minDistBall = 1000.0;
		for( unsigned int i = 0 ; i < runningPlayersBallIdx.size() ; i++ )
		{
			double agentBallDist = world->robot[runningPlayersBallIdx[i]].ball.distance();
			if ( agentBallDist < minDistBall )
			{
				minDistBall = agentBallDist;
				chosenAgent = runningPlayersBallIdx[i];
			}
		}

		shareBall->pos 		= world->robot[chosenAgent].ball.pos;
		shareBall->vel 		= world->robot[chosenAgent].ball.vel;
		shareBall->visible	= world->robot[chosenAgent].ball.visible;
		shareBall->airborne = world->robot[chosenAgent].ball.airborne;
		shareBall->height 	= world->robot[chosenAgent].ball.height;
		shareBall->posRel 	= world->abs2rel(shareBall->pos);// world->robot[chosenAgent].ball.posRel;
		shareBall->engaged	= false;
		shareBall->hasMoved	= world->robot[chosenAgent].ball.hasMoved;
		shareBall->own 		= false;
	}
	else	//if no one sees the ball, no choice, it is not visible
		shareBall->clear();
}

void Integrator::updateGameState()
{
	static int countBallMoved = 0;
	static int grabberTouched = false;

	if( coach.gameState != SIGourKickOff && coach.gameState != SIGtheirKickOff )
		world->resetParkingTimer();

	if( coach.gameState == SIGdropBall )
		world->gameState = dropBall;
	else
	if( coach.gameState == SIGparking )
		world->gameState = parking;
	else
	if( coach.gameState == SIGourKickOff )
	{
		double timeToWait = Whoami() * (config->getParam("parkingTimeIntervalMS"));
		if( world->parkingTimeMS() < timeToWait )
			world->gameState = stopRobot;
		else
			world->gameState = preOwnKickOff;
	}
	else
	if( coach.gameState == SIGtheirKickOff )
	{
		double timeToWait = Whoami() * (config->getParam("parkingTimeIntervalMS"));
		if( world->parkingTimeMS() < timeToWait )
			world->gameState = stopRobot;
		else
			world->gameState = preOpponentKickOff;
	}
	else
	if( coach.gameState == SIGourFreeKick )
		world->gameState = preOwnFreeKick;
	else
	if( coach.gameState == SIGtheirFreeKick )
		world->gameState = preOpponentFreeKick;
	else
	if( coach.gameState == SIGourGoalKick )
		world->gameState = preOwnGoalKick;
	else
	if( coach.gameState == SIGtheirGoalKick )
		world->gameState = preOpponentGoalKick;
	else
	if( coach.gameState == SIGourCornerKick )
		world->gameState = preOwnCornerKick;
	else
	if( coach.gameState == SIGtheirCornerKick )
		world->gameState = preOpponentCornerKick;
	else
	if( coach.gameState == SIGourThrowIn )
		world->gameState = preOwnThrowIn;
	else
	if( coach.gameState == SIGtheirThrowIn )
		world->gameState = preOpponentThrowIn;
	else
	if( coach.gameState == SIGourPenalty )
		world->gameState = preOwnPenalty;
	else
	if( coach.gameState == SIGtheirPenalty )
		world->gameState = preOpponentPenalty;
	else
	if( coach.gameState == SIGstop || coach.gameState == SIGhalt )
	{
		world->resetParkingTimer();
		world->gameState = stopRobot;
	}
	else
	if( coach.gameState == SIGstart )
	{
		static Vec ballPos;
		static bool ballSeen;
		static bool playStarted;
		static bool playTimer;
		static struct timeval tv1;
		static struct timeval tv2;
		const int gs = (WSGameState)(((int)(world->gameState))+1); // TODO FIXME

		if( world->gameState == dropBall )
		   world->gameState = freePlay;
		else
		if( world->gameState == stopRobot || world->gameState == parking )
		   world->gameState = freePlay;
		else
		if( world->gameState == preOwnKickOff
		|| world->gameState ==  preOwnThrowIn
		|| world->gameState ==  preOwnGoalKick
		|| world->gameState ==  preOwnCornerKick
		|| world->gameState ==  preOwnPenalty
		|| world->gameState ==  preOwnFreeKick )
		{
			//cout << "--- own reset timer"<< endl;
			gettimeofday(&tv1,NULL);
			grabberTouched = false;

			ballPos = Vec(0.0,0.0);
			ballSeen = false;
			if( world->me->ball.visible )
			{
				ballPos = world->getBestBallAbs();
				//ballpos = world->me->ball.posRel;
				ballSeen = true;
				playStarted = false;
			}

			//Reset the receiverIdxForCorridor when a new own setpiece is started
			receiverIdxForCorridor = -1;

			world->gameState = (WSGameState)gs;
		}
		else
		if(  world->gameState == preOpponentKickOff
		|| world->gameState ==  preOpponentThrowIn
		|| world->gameState ==  preOpponentGoalKick
		|| world->gameState ==  preOpponentCornerKick
		|| world->gameState ==  preOpponentPenalty
		|| world->gameState ==  preOpponentFreeKick )
		{
			gettimeofday(&tv1,NULL);

			ballSeen = false;
			ballPos = Vec(0.0,0.0);
			if( world->me->ball.visible )
			{
				ballPos = world->getBestBallAbs();
				//ballpos = world->me->ball.posRel;
				ballSeen = true;
			}

			world->gameState = (WSGameState)gs;
		}
		else
		if(  world->gameState == postOpponentKickOff
		|| world->gameState == postOpponentThrowIn
		|| world->gameState == postOpponentGoalKick
		|| world->gameState == postOpponentCornerKick
		|| world->gameState == postOpponentPenalty
		|| world->gameState == postOpponentFreeKick )
		{
			gettimeofday(&tv2,NULL);
			const int waitTime = (int)( (tv2.tv_sec+tv2.tv_usec/1e6) - (tv1.tv_sec+tv1.tv_usec/1e6) );

			// myprintf("BARRIER post int. %d\n", waitTime);

			if( waitTime >= WAIT_TIME_SEC-1 || world->isTeamEngaged())//  || countBallMoved > 5)
			{
				world->gameState = freePlay;
				//countBallMoved=0;
			}
		}
		else
		if(  world->gameState == postOwnKickOff
		|| world->gameState == postOwnThrowIn
		|| world->gameState == postOwnGoalKick
		|| world->gameState == postOwnCornerKick
		|| world->gameState == postOwnFreeKick )
		{
			gettimeofday(&tv2,NULL);
			unsigned long currentTime = (tv2.tv_sec+tv2.tv_usec/1e6);
			const int toPlayTime = (int)( (tv2.tv_sec+tv2.tv_usec/1e6) - (tv1.tv_sec+tv1.tv_usec/1e6) );

			if( ballSeen && world->me->ball.visible )
			{
			   float ballMoved = (ballPos - world->getBestBallAbs()).length();
			   //ballMoved = (ballpos - world->me->ball.pos).length();

				if( ballMoved>0.20 )
					countBallMoved++;
				else
					countBallMoved = 0;
			}
			else if ( world->me->ball.visible )
			{
				ballSeen = true;
				ballPos = world->getBestBallAbs();
			}

			if( countBallMoved > (int)(5*33/MOTION_TICK) )
				playStarted  = true;
			else
				playTimer = toPlayTime;

			const int playTime = toPlayTime - playTimer;

			// TODO Review this part of code, receiver probably does not set the flag
			bool receiverTimeout = false;
			vector<int> receiverList = world->getRoleIndex(rReceiver,true);
			for(unsigned int i=0; i < receiverList.size(); i++)
			{
				if(world->robot[receiverList[i]].coordinationFlag[0] == Ready)
				{
					receiverTimeout = true;
				}
			}

			vector<int> strikerList = world->getRoleIndex(rStriker,true);

			static unsigned long grabberTouchedLastTime;
			if(world->grabberTouched(true))
			{
				grabberTouchedLastTime = (tv2.tv_sec+tv2.tv_usec/1e6);
				grabberTouched = true;
			}

			if( toPlayTime >= PLAY_TIME_SEC-1 ||
				playTime >= PASS_TIME_SEC ||
				receiverTimeout ||
				strikerList.size() ||
				!setPieceBallInCorridor() ||
				(world->me->role != rReplacer && grabberTouched && !world->me->ball.engaged && (currentTime - grabberTouchedLastTime) > 0) ||
				(world->me->role == rReceiver && world->me->ball.engaged))
			{
				world->gameState = freePlay;
			}

			vector<int> replacers = world->getRoleIndex(rReplacer, true);
			bool abortPass = false;
			int currentReplacerIdx = -1;
			Line passLine = Line::def;
			bool validLine = false;
			if( replacers.size() == 1)
			{
				currentReplacerIdx = replacers.at(0);
				passLine = world->robot[currentReplacerIdx].passLine;
				validLine = (passLine.p1 != Line::def.p1 && passLine.p2 != Line::def.p2);
				if( world->robot[currentReplacerIdx].coordinationFlag[0] >= BallPassed0
						&& world->robot[currentReplacerIdx].coordinationFlag[0] <= BallPassed5)
				{
					if(validLine && world->robot[currentReplacerIdx].passLine.distance(world->me->ball.pos) > 0.5)
					{
						abortPass = true;
					}
				}
			}

			if (currentReplacerIdx != -1 && validLine)
			{

				if ( world->isBallPassedToMe() )
				{
					float meP1Dist = (world->me->pos - world->robot[currentReplacerIdx].passLine.p1).length();
					float ballP1Dist = (world->me->ball.pos - world->robot[currentReplacerIdx].passLine.p1).length();
					if (ballP1Dist > meP1Dist)
						abortPass = true;
				}
			}

			if (world->me->role != rReplacer
					&& world->me->ball.visible
					&& world->me->ball.own
					&& world->me->ball.vel.length() < 0.5 && world->me->ball.posRel.length() < 0.7)
			{
				abortPass = true;
			}

			if(abortPass)
				world->gameState = freePlay;
		}
	}
	else if( world->gameState == postOwnPenalty)
	{
		//Do Nothing, Keep gstate = postOwnPenalty
	}
	else
		world->gameState = errorState;

	world->me->currentGameState = world->gameState;

}

bool Integrator::setPieceBallInCorridor()
{
	//get the id of the replacer
	vector<int> replacerList = world->getRoleIndex( rReplacer , true );
	static Vec corridorStart;
	static Vec corridorEnd;

	int passerIdx;

	if (replacerList.size() != 0)
	{
		passerIdx = replacerList.at(0);

		switch(world->robot[passerIdx].coordinationFlag[0])
		{
			case TryingToPass0:
			{
				receiverIdxForCorridor = 0;
				corridorStart = world->robot[passerIdx].pos;
				corridorEnd = world->robot[receiverIdxForCorridor].pos;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 0\n");
				break;
			}
			case TryingToPass1:
			{
				receiverIdxForCorridor = 1;
				corridorStart = world->robot[passerIdx].pos;
				corridorEnd = world->robot[receiverIdxForCorridor].pos;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 1\n");
				break;
			}
			case TryingToPass2:
			{
				receiverIdxForCorridor = 2;
				corridorStart = world->robot[passerIdx].pos;
				corridorEnd = world->robot[receiverIdxForCorridor].pos;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 2\n");
				break;
			}
			case TryingToPass3:
			{
				receiverIdxForCorridor = 3;
				corridorStart = world->robot[passerIdx].pos;
				corridorEnd = world->robot[receiverIdxForCorridor].pos;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 3\n");
				break;
			}
			case TryingToPass4:
			{
				receiverIdxForCorridor = 4;
				corridorStart = world->robot[passerIdx].pos;
				corridorEnd = world->robot[receiverIdxForCorridor].pos;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 4\n");
				break;
			}
			case TryingToPass5:
			{
				receiverIdxForCorridor = 5;
				corridorStart = world->robot[passerIdx].pos;
				corridorEnd = world->robot[receiverIdxForCorridor].pos;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 5\n");
				break;
			}
			case TryingToPass:
			{
				receiverIdxForCorridor = 10;
//				fprintf(stderr,"FREEPLAY_FORCE, receiver 10\n");
				break;
			}
			case BallPassed:
//				fprintf(stderr,"FREEPLAY_FORCE, EVALUATING\n");
				if (receiverIdxForCorridor != 10)
				{
//					fprintf(stderr,"FREEPLAY_FORCE, line between: (%.2f, %.2f) - (%.2f,%.2f)\n",corridorStart.x,corridorStart.y, corridorEnd.x, corridorEnd.y);
					LineSegment lineSegment(corridorStart, corridorEnd);

					Vec closestPoint = lineSegment.closestPoint(world->me->ball.pos);
//					fprintf(stderr,"FREEPLAY_FORCE, dist: %.2f\n", (closestPoint - world->me->ball.pos).length());

					if( (closestPoint - world->me->ball.pos).length() > 1.0 )
					{
//						fprintf(stderr,"FREEPLAY_FORCE, Ball left corridor, return false\n");
						return false;
					}
				}
				else	//If receiverIdx was set to 10, then cross was made, go to freePlay imediately
				{
//					fprintf(stderr,"FREEPLAY_FORCE, CROSS was made, immdediately false\n");
					return true;
				}
				break;
			default:
//				fprintf(stderr,"FREEPLAY_FORCE, default, just break\n");
				break;
		}

	}

//	fprintf(stderr,"FREEPLAY_FORCE, end, return default true\n");
	//If there is no replacer, assume the ball is on, to keep everything as before
	return true;
}

void Integrator::predictNoCollision()
{
	int delay = config->getParam("delay");
	int cycle_time = config->getParam("cycle_time");
	Vec predPos(world->me->pos.x,world->me->pos.y);
	Angle predDir(world->me->orientation);
	Vec absVel;

	for(unsigned int i=0; i < buffer.size();i++)
	{
		Vec relVel(buffer[i].vx,buffer[i].vy);
		absVel = relVel.s_rotate(predDir);

		if(i==0)
		{
			predPos.x += absVel.x * ((delay%cycle_time)/1000.0);
			predPos.y += absVel.y * ((delay%cycle_time)/1000.0);
			predDir += buffer[i].va * ((delay%cycle_time)/1000.0);
		}else{
			predPos.x += absVel.x * ((cycle_time)/1000.0);
			predPos.y += absVel.y * ((cycle_time)/1000.0);
			predDir += buffer[i].va * ((cycle_time)/1000.0);
		}
	}

	//predicted robot pose
	world->me->pos = predPos;
	world->me->orientation = predDir.get_rad();
	//	world->me->vel = absVel;

	if(world->me->ball.engaged)
	{
		world->me->ball.pos = world->rel2abs(world->me->ball.posRel);
	}else{
		world->me->ball.pos.x = world->me->ball.pos.x + world->me->ball.vel.x * delay/1000.0;
		world->me->ball.pos.y = world->me->ball.pos.y + world->me->ball.vel.y * delay/1000.0;
	}

	world->me->ball.posRel = world->abs2rel(world->me->ball.pos);

}

bool Integrator::isStuck()
{

	static Vec lastVel(0,0);
	static Timer deltaTime;

	Vec desiredVelRel(buffer[1].vx,buffer[1].vy);
	Vec desiredVel = desiredVelRel.s_rotate(world->me->orientation);
	Vec estimatedVel = world->me->vel;

	if(desiredVel.length() == 0.0)
	{
		deltaTime.restart();
		return false;
	}

//	const float cycleTime = config->getParam("cycle_time");
	float difference = fabs(desiredVel.length() - estimatedVel.length());
	float threshold = desiredVel.length() * SPEED_THRESHOLD;
	float time = deltaTime.elapsed();

	lastVel = estimatedVel;

	//validação: se o robot está a acelerar ou desacelerar, retorna falso
	//comparar a primeira velocidade do buffer com a velocidade estimada
	if( difference > threshold )
	{
		if(time > TIME_THRESHOLD)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	deltaTime.restart();
	return false;
}

}/* namespace cambada */
