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

#include "WorldState.h"
#include "ConfigXML.h"

using namespace cambada::geom;

namespace cambada {

Robot* WorldState::me = NULL;

WorldState::WorldState( ConfigXML* config) {

	this->config = config; 							// Set 'config' object
	field = new Field( config ); 					// Initialize 'field' object

	gameState = stopRobot;							// Init gameState as STOP

	// Init various objects
	this->isFormationCoachAvailable = false;
	this->timeStamp = 0;
	this->parkingTimer = 0.0;
	this->SetPiecesZones = new Zones(field);
	this->lastCycleEngaged = false;

	// Init kicking conditions variables
	ok2kick = false;

	/**
	 * param1 = distance to consider obstacles (above "param1" -> ignore)
	 * param2 = width of the sonar
	 * param3 = distance to clip width (until "param3", the sonar is a cone)
	 * param4 = number of sensors
	 */
	freeMoveSonar = Sonar(config->getParam("avoid_distance"), 0.9, 1.5, (int)(config->getParam("avoid_nSensors")) );	//moving free, the corridor may be thinner, so use only 1 meter for sonar opening.
	dribbleSonar = Sonar(4.0, 1.5, 1.5, (int)(config->getParam("avoid_nSensors")));

	mapObstacles = new HeightMap();
	mapDribble = new HeightMap();
	mapReceiveBallFP = new HeightMap();
	mapTheirGoalFOV = new HeightMap();
	mapKick2Goal = new HeightMap();
	receiverSPMap = new HeightMap();

	grabberWasTouched = false;

	FILE *fp = fopen("../config/handicapGrabber", "r");                                // Open config file for reading
	int handicappedValue;
	if(fp == NULL)
	{
		fprintf(stderr,"ERROR OPENING HANDICAP FILE!!!\n");
		exit(1);
	}
	if ( fscanf(fp, "%d", &handicappedValue) != 1)
	{
		fprintf(stderr,"ERROR READING HANDICAP FILE!!!\n");
		exit(1);
	}
	fprintf(stderr,"HANDICAP_TEST handicapped: %d\n", handicappedValue);
	if(Whoami() > 0 && Whoami() <= N_CAMBADAS)
		robot[Whoami()-1].handicappedGrabber = (handicappedValue == 0)? false: true;

	pointsRighOfBall = 0;
	pointsLeftOfBall = 0;
}

WorldState::~WorldState() {
	delete field;

	delete mapKick2Goal;
	delete mapTheirGoalFOV;
	delete mapReceiveBallFP;
	delete mapDribble;
	delete mapObstacles;
	delete receiverSPMap;
}

Vec WorldState::rel2abs(const Vec& rel, int robotIdx)
{
	//override of world me with local declaration
	Robot *me;
	me = &robot[robotIdx];
	Vec abs;
	abs.x = me->pos.x + cos(me->orientation)*rel.x-sin(me->orientation)*rel.y;
	abs.y = me->pos.y + sin(me->orientation)*rel.x+cos(me->orientation)*rel.y;
	return abs;
}

Vec WorldState::rel2absDelta(const Vec& rel)
{
	Vec abs = rel;
	abs.s_rotate ( Angle( me->orientation ) );

	return abs;
}

Vec WorldState::abs2rel(const Vec& abs, int robotIdx)
{
	//override of world me with local declaration
	Robot *me;
	me = &robot[robotIdx];
	Vec rel = abs - me->pos;
	rel.s_rotate( Angle(-me->orientation) );
	return rel;
}

Vec WorldState::abs2rel(const Vec& position , const double& orientation, const Vec& abs)
{
	Vec rel = abs - position;
	rel.s_rotate( Angle(-orientation) );

	return rel;

}

Vec WorldState::abs2relDelta(const Vec& abs)
{
	Vec rel = abs;
	rel.s_rotate ( Angle( -me->orientation ) );

	return rel;
}

Vec WorldState::getAvoidAdjustedPosition(Vec targetRel, bool moveFree, bool avBall, AvoidLevel avLevel)
{
	/**Create an exception for when the robot is very close to its target. It does not make sense to try to avoid the obstacles around when we already are at the point.*/
	if (targetRel.length() < NO_AVOID_DISTANCE)
	{
		#if DEBUG_SONAR
		fprintf(stderr,"SONAR Very close, not avoiding!\n");
		#endif
		return targetRel;
	}

	//WARNING WORKS WITH RELATIVE TARGET
	vector<Vec> obstaclesToAvoid;
	double avObstBallDist = 1.0;
	double maxSonarDist, robotCenterOffset;

	if (moveFree)
	{
		maxSonarDist = freeMoveSonar.getMaxSonarDistance();
		robotCenterOffset = freeMoveSonar.getRobotCenterOffset();
	}
	else
	{
		maxSonarDist = dribbleSonar.getMaxSonarDistance();
		robotCenterOffset = dribbleSonar.getRobotCenterOffset();
	}

///////////// PUT OBSTACLES POINTS ON THE AVOIDANCE LIST
	if( avLevel != avoidNone  )
	{
		for (unsigned int i = 0; i < obstacles.size(); i++)
		{
			//Escape case created for stopped balls, when we want the robot to go around it to outside of the field. Especially critical in corners, where the obstacles outside field would prevent the robot to go to the small corner
			if( avBall && me->ball.visible &&  !field->isInside(obstacles[i].obstacleInfo.absCenter, OBSTACLE_RADIUS) )
				continue;

			#if DEBUG_SONAR
			fprintf(stderr,"SONAR (%.1f, %.1f) obstDist: %f, distToOutile: %f, maxSonarDist: %f\n", obstacles.at(i).obstacleInfo.absCenter.x,
					obstacles.at(i).obstacleInfo.absCenter.y,
					obstacles.at(i).limitCenter.length(), robotCenterOffset, maxSonarDist);
			#endif
			if( ( ( (avLevel == avoidFull) &&
				    ((obstacles.at(i).limitCenter.length() - robotCenterOffset) < maxSonarDist) )
				) ||
				( ( (avLevel == avoidSemi) &&
					  ((obstacles[i].limitCenter - targetRel).length() > avObstBallDist) &&
					  ((obstacles.at(i).limitCenter.length() - robotCenterOffset) < maxSonarDist) )
				)
			  )
			{
				#if DEBUG_SONAR
				fprintf(stderr,"SONAR Obstacle was pushed to the list!!\n");
				#endif

				Vec limitCenter = abs2rel(obstacles.at(i).obstacleInfo.absCenter);
				limitCenter = limitCenter.setLength(limitCenter.length() - 0.25);

				Vec aux = limitCenter.setLength(0.25).rotate_quarter();
				Vec rightPoint = limitCenter - aux;
				Vec leftPoint = limitCenter + aux;

				obstaclesToAvoid.push_back( limitCenter );
				obstaclesToAvoid.push_back( rightPoint );
				obstaclesToAvoid.push_back( leftPoint );
			}

		}

		vector<int> runningFieldRobots = getRunningFieldRobotsIdx();
		for( unsigned int i = 0 ; i < runningFieldRobots.size() ; i++ )
		{
			int tmpIdx = runningFieldRobots[i];
			if(tmpIdx == getMyIdx())
				continue;

			Vec pos = robot[tmpIdx].pos;
			Vec relPos = abs2rel(pos);
			float distRelPos = relPos.length();
			relPos.setLength(distRelPos - 0.30);
			obstaclesToAvoid.push_back(relPos);
		}
	}

///////////// CREATE AN OBSTACLE FOR THE BALL AND PUT THEM ON THE AVOIDANCE LIST
	#if DEBUG_SONAR
	if (avBall)
	{
		fprintf(stderr,"SONAR ballDist: %f, distToOutile: %f, maxSonarDist: %f\n", me->ball.posRel.length(), robotCenterOffset, maxSonarDist);
	}
	#endif
	if( avBall && me->ball.visible && ( (me->ball.posRel.length() - robotCenterOffset) < maxSonarDist) )
	{
		//Push one single point as obstacle for ball avoidance. This point is the surface of the ball in the direction of the robot (closer ball point)
		Vec ballRel = me->ball.posRel;
		obstaclesToAvoid.push_back( ballRel.setLength(ballRel.length()-0.11) );
		#if DEBUG_SONAR
		fprintf(stderr,"SONAR Ball was pushed to the list!!\n");
		fprintf(stderr,"SONAR Ball is (%f,%f), ballObstacle is (%f,%f)\n", ballRel.x, ballRel.y, ballRel.setLength(ballRel.length()-0.11).x, ballRel.setLength(ballRel.length()-0.11).y);
		#endif
	}

	if (!moveFree)
	{
		if ( field->isInside(me->pos) )
		{
			Circle sideLineTester = Circle( me->pos, 2.0 );
			Line sideLine, endLine;

			if ( me->pos.x > 0.0 )
				sideLine = Line( Vec(field->halfWidth,-field->halfLength), Vec(field->halfWidth,field->halfLength) );
			else
				sideLine = Line( Vec(-field->halfWidth,-field->halfLength), Vec(-field->halfWidth,field->halfLength) );

			if ( me->pos.y > 0.0 )
				endLine = Line( Vec(-field->halfWidth,field->halfLength), Vec(field->halfWidth,field->halfLength) );
			else
				endLine = Line( Vec(-field->halfWidth,-field->halfLength), Vec(field->halfWidth,-field->halfLength) );

			vector<Vec> sidePoints;
			if ( (sidePoints = intersect(sideLine, sideLineTester)).size() == 2 )	//create obstacles between the 2 points on the sideline
			{
				Vec tempObst = sidePoints[0];
				if ( sidePoints[1].y > tempObst.y )
				{
					while ( tempObst.y < sidePoints[1].y )
					{
						obstaclesToAvoid.push_back( abs2rel(tempObst) );
						tempObst.y += OBSTACLE_RADIUS;
					}
				}
				else
				{
					while ( tempObst.y > sidePoints[1].y )
					{
						obstaclesToAvoid.push_back( abs2rel(tempObst) );
						tempObst.y -= OBSTACLE_RADIUS;
					}
				}
			}

			vector<Vec> endPoints;
			if ( (endPoints = intersect(endLine, sideLineTester)).size() == 2 )
			{
				Vec tempObst = endPoints[0];
				if ( endPoints[1].x > tempObst.x )
				{
					while ( tempObst.x < endPoints[1].x )
					{
						obstaclesToAvoid.push_back( abs2rel(tempObst) );
						tempObst.x += OBSTACLE_RADIUS;
					}
				}
				else
				{
					while ( tempObst.x > endPoints[1].x )
					{
						obstaclesToAvoid.push_back( abs2rel(tempObst) );
						tempObst.x -= OBSTACLE_RADIUS;
					}
				}
			}
		}
	}

///////////// FINALY GET THE NEW TARGET
	Angle newDirection;
	if (moveFree)
	{
		newDirection = freeMoveSonar.getFreeDirection(targetRel, obstaclesToAvoid, abs2relDelta(me->vel) );
	}
	else
	{
		newDirection = dribbleSonar.getFreeDirection(targetRel, obstaclesToAvoid, abs2relDelta(me->vel) );
	}
	Vec newTarget = targetRel.rotate(newDirection - targetRel.angle());

	// DEFINE DEBUG POINTS AS THE ORIGINAL TARGET AND THE NEW AVOIDANCE TARGET DIRECTION
	me->debugPoints[0]=rel2abs(targetRel);
	#if DEBUG_SONAR
	fprintf(stderr,"SONAR maxSonarDist: %f, NewTarget: %f,%f, length: %f\n", maxSonarDist, newTarget.setLength(maxSonarDist).x, newTarget.setLength(maxSonarDist).y, newTarget.setLength(maxSonarDist).length());
	#endif
	me->debugPoints[1]=rel2abs(newTarget.setLength(maxSonarDist));

	return newTarget;
}

Vec WorldState::getBestBallAbs(bool restrictToVisible) const
{
	Vec ballAbs(0.0,0.0);
	double ballDist = 2008;
	for( int i = 0 ; i<N_CAMBADAS ; i++ )
		if ((robot[i].ball.visible || !restrictToVisible)
		&& robot[i].running
		&& robot[i].ball.posRel.length() < ballDist
		&& field->isInside(robot[i].ball.pos,1.0))
		//&& world->cambada[i].ball.position.length() > 0.0 )
		{
			ballAbs = robot[i].ball.pos;
			ballDist = robot[i].ball.posRel.length();
		}

	return ballAbs;
}

bool WorldState::isBallVisible() const
{
	for (int i = 0; i < N_CAMBADAS; i++)
		if (robot[i].ball.visible && robot[i].running)
			return true;
	return false;
}

vector<int> WorldState::getRunningFieldRobotsIdx()
{
	vector<int> runningRobots;
	for( int i = 0 ; i < N_CAMBADAS ; i++ )
		if( robot[i].running  && robot[i].role != rGoalie )
			runningRobots.push_back(i);

	return runningRobots;
}

int WorldState::getNumberOfRunningFieldRobots()
{
	int numberOfRunningFieldRobots = 0;

	for( int i = 0 ; i < N_CAMBADAS ; i++ )
		if( robot[i].running  && robot[i].role != rGoalie )
			numberOfRunningFieldRobots++;

	return numberOfRunningFieldRobots;
}

vector<int> WorldState::getRoleIndex( RoleID role , bool meIncluded )
{
	vector<int> idxRole;

	for( int i = 0 ; i < N_CAMBADAS ; i++ ){
		if( meIncluded  || (i+1 != me->number) ){
			if( robot[i].running && robot[i].role == role )
			{
				idxRole.push_back(i);
			}
		}
	}

	return idxRole;
}

int WorldState::getMyIdx()
{
	return Whoami() -1;
}

Vec WorldState::getReplacerSearchPos()
{
	const double xOffset = 3.0;
	const double yOffset = 3.0;
	Vec searchPos;
	Vec searchPosLeft( field->halfWidth - xOffset, field->halfLength - yOffset);
	Vec searchPosRight( -field->halfWidth + xOffset, field->halfLength - yOffset);

	static Vec searchAt = searchPosLeft;

	if( ( me->pos - searchPosLeft ).length() < 0.5  )
		searchAt = searchPosRight;
	else
	if( ( me->pos - searchPosRight ).length() < 0.5  )
		searchAt = searchPosLeft;

	searchPos = searchAt;
	return searchPos;
}

double WorldState::parkingTimeMS()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);

	return ( (tv.tv_sec*1e3+tv.tv_usec/1e3) - parkingTimer );
}


void WorldState::resetParkingTimer()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	parkingTimer = tv.tv_sec*1e3+tv.tv_usec/1e3;
}

void WorldState::update()
{
	// Update Opponent Dribbling
	if (me->ball.visible)
		updateOpponentDribbling();

	ok2kick_update();

	// Update timer for setpieces
	switch(gameState)
	{
	case preOwnKickOff:
	case preOwnGoalKick:
	case preOwnCornerKick:
	case preOwnThrowIn:
	case preOwnFreeKick:
	case postOwnKickOff:
	case postOwnGoalKick:
	case postOwnCornerKick:
	case postOwnThrowIn:
	case postOwnFreeKick:
		timeAfterOurSetPiece.restart();
		break;
	case preOpponentKickOff:
	case preOpponentGoalKick:
	case preOpponentCornerKick:
	case preOpponentThrowIn:
	case preOpponentFreeKick:
	case postOpponentKickOff:
	case postOpponentGoalKick:
	case postOpponentCornerKick:
	case postOpponentThrowIn:
	case postOpponentFreeKick:
		timeAfterTheirSetPiece.restart();
		break;
	default:
		break;
	}
}

void WorldState::updateEndCycle() {
	lastCycleEngaged = me->ball.engaged;
	lastCycleVisible = me->ball.visible;
}

bool WorldState::isTeamEngaged()
{
	for( int i = 1 ; i < N_CAMBADAS ; i++ )
		if( robot[i].running && robot[i].ball.engaged )
			return true;

	return false;
}

float WorldState::lineClear( Vec origin, Vec destination, int indexToIgnore, float obsIgnoreDist, int robotIdx)
{
	Vec intersect;
	destination = (destination - origin).setLength((destination - origin).length() + 0.4) + origin;
	Line linha1 = Line(origin, destination);

	float minDist = 2014;
	//override of world me with local declaration
	Robot *me;
	me = &robot[robotIdx];

	bool local = (robotIdx == Whoami() - 1);
	for (unsigned int i = 0;
			(local && i < obstacles.size()) || (!local && i < me->nObst); i++)
	{
		bool teamMate;
		Vec currentObs;
		if (local)
		{
			teamMate = obstacles[i].obstacleInfo.isTeamMate();
			if (indexToIgnore != -1
					&& obstacles[i].obstacleInfo.id == (indexToIgnore + 1))
				continue;

			currentObs = obstacles[i].obstacleInfo.absCenter;
		}
		else
		{
			teamMate = me->obstacles[i].isTeamMate();
			if (indexToIgnore != -1
					&& me->obstacles[i].id == (indexToIgnore + 1))
				continue;

			currentObs = me->obstacles[i].absCenter;
		}

		if ((currentObs - origin).length() < obsIgnoreDist)
			continue;

		Vec temp = (destination - origin).normalize().rotate(Angle::deg_angle(90));
		Line limit1 = Line(origin, origin + temp);
		Line limit2 = Line(destination, destination + temp);

		if (limit1.side(currentObs) != 1 || limit2.side(currentObs) != -1)
		{
			continue;
		}

		// Point on the line, closest (perpendicular) to the obstacle position
		intersect = linha1.perpendicular_point(currentObs);

		// distance between obstacle and line
		float nowDist = (currentObs - intersect).length();

		if (nowDist < minDist)
		{
			if (teamMate && me->currentGameState != freePlay && nowDist > 0.5)
			{
				continue;
			}
			minDist = nowDist;
		}
	}

	return minDist - 0.37; //the value subtracted is half of the max diagonal of a robot,
						   //so the returned value is minDist to the periphery of the closest obstacle
						   //and not the minDist to the center of the closest obstacle
}

bool WorldState::isLineClear( Vec absPosition, double conf, int indexToIgnore, Vec ownPos, float obsIgnoreDist, int robotIdx)
{
	return lineClear(ownPos, absPosition, indexToIgnore, obsIgnoreDist, robotIdx) >= conf;
}

Angle WorldState::getVectorAlignment(Vec v1, Vec v2)
{
	return v1.angle() - v2.angle();
}

double WorldState::getBallPathAlignment()
{
	if(me->ball.vel.length()<0.2)
		return 0;

	Vec ballVelAngle = me->ball.vel;
	Vec ballRobotAngle = me->pos - me->ball.pos;

	return getVectorAlignment(ballVelAngle,ballRobotAngle).get_rad_pi();
}

bool WorldState::isMovingOutside(Vec movePos,Vec& clippedPos)
{

	vector<LineSegment> borderLines;

	LineSegment leftLine = LineSegment(Vec(-field->halfWidth,-field->halfLength),Vec(-field->halfWidth,field->halfLength));
	LineSegment rightLine = LineSegment(Vec(field->halfWidth,-field->halfLength),Vec(field->halfWidth,field->halfLength));
	LineSegment ourLine = LineSegment(Vec(-field->halfWidth,-field->halfLength),Vec(field->halfWidth,-field->halfLength));
	LineSegment theirLine = LineSegment(Vec(-field->halfWidth,field->halfLength),Vec(field->halfWidth,field->halfLength));

	borderLines.push_back(leftLine);
	borderLines.push_back(rightLine);
	borderLines.push_back(ourLine);
	borderLines.push_back(theirLine);

	Vec ballPos = me->ball.pos;

	LineSegment movingPath = LineSegment(ballPos,movePos);

	for(unsigned int i=0;i<borderLines.size();i++)
	{
		vector<Vec> intersections = intersect(movingPath,borderLines[i]);
		if(intersections.size())
		{
			clippedPos = intersections[0];
			return true;
		}
	}

	return false;
}

bool WorldState::isBallRollingAwayOrStopped()
{
	if((!me->ball.visible) || !me->ball.own ) {
		return false;
	}

	Vec ballToMe = me->pos - me->ball.pos;
	Vec ballVelocity = me->ball.vel;
	if(ballVelocity.length() < 0.2) {
		return true;
	}

	float onBallAngle = fabs( (ballVelocity.angle(ballToMe)).get_deg_180() );

	return onBallAngle > 45;
}

Vec WorldState::getScalingFactor()
{
	float oficial_length = 18.0;
	float oficial_width  = 12.0;

	// Scaling vector
	return Vec(field->width / oficial_width,  field->length / oficial_length);
}

double WorldState::getGoalSideOffset(double dist)
{
	float offset;

	if(dist > field->halfLength)
	{
		offset = 0.0;
	}
	else
	{
		offset = (config->getParam("goal_side_offset_factor") / dist) ;
		if(offset > 0.5)
		{
			offset = 0.5;
		}
	}

	return offset;
}

double WorldState::getGoalSideOffset(double dist, float x)
{
	double linearFactor1 = 0.0073;
	double linearFactor2 = -0.1458;
	double linearFactor3 = 0.7545;

	double xFactor = -0.1;

	double goalSideOffset = linearFactor1*dist*dist + linearFactor2*dist + linearFactor3 + xFactor*fabs(x);

	if(goalSideOffset<0)
		return 0;

	return goalSideOffset;
}

Vec WorldState::getOpponentGoalie()
{

	Vec openGoal(0,0);

	//area between posts
	XYRectangle keeperArea(Vec(-field->goalAreaHalfWidth,field->goalLength)+field->theirGoal,Vec(field->goalHalfWidth,-field->goalAreaLength) + field->theirGoal);

	cerr << "KICK KeeperArea " << keeperArea.p1 << " " << keeperArea.p2 << endl;

	for ( unsigned int i = 0; i < sharedObstacles.size(); i++ )
	{
		if ( keeperArea.is_inside( sharedObstacles[i].obstacleInfo.absCenter ) )
			return sharedObstacles[i].obstacleInfo.absCenter;
	}

	for ( unsigned int i = 0; i < obstacles.size(); i++ )
	{
		if ( keeperArea.is_inside( obstacles[i].obstacleInfo.absCenter ) )
		{
			return obstacles[i].obstacleInfo.absCenter;
		}
	}

	return openGoal;

}

bool WorldState::isBallGoingInMyDirection(double angle)
{
	if(me->ball.vel.length() < 0.4)
		return false;

	Vec ballToMe = (me->pos - me->ball.pos).setLength(1.0);
	Vec ballVel = (me->ball.vel).setLength(1.0);

	float dotProduct = ballToMe * ballVel; // dotProd = |a| |b| cos(theta)
	if(dotProduct > cos(angle))
		return true;
	else
		return false;
}

bool WorldState::obstaclesInFront(float distance) {

	float robotCenter2grabber = 0.15; // not less than 10 cm

	if(distance < 1.0)									// if less than 1 m
		distance = 1.0;									// limit to 1 m

	Vec p1 = Vec(0,robotCenter2grabber);
	Vec p2 = Vec(0,distance);
	LineSegment me2front = LineSegment(p1, p2);

	for(unsigned int i = 0; i < obstacles.size(); i++) {
		Vec obstCenter = abs2rel(obstacles.at(i).obstacleInfo.absCenter);

		Vec perpPoint = me2front.closestPoint(obstCenter); // perpendicular point
		if(perpPoint != p1 && perpPoint != p2)
		{
			if((perpPoint - obstCenter).length() < 0.45) // check distance
				return true;
		}
	}

	return false;
}

bool WorldState::obstaclesToTheirGoal(float distance, Vec position) {

	float robotCenter2grabber = 0.15; // not less than 10 cm

	if(distance < 1.0)									// if less than 1 m
		distance = 1.0;									// limit to 1 m

	Vec pos2goal = field->theirGoal - position;
	Vec p1 = position + pos2goal.setLength(robotCenter2grabber);
	Vec p2 = position + pos2goal.setLength(distance);
	LineSegment p2goal = LineSegment(p1, p2);

	for(unsigned int i = 0; i < obstacles.size(); i++) {
		Vec obstCenter = obstacles.at(i).obstacleInfo.absCenter;

		Vec perpPoint = p2goal.closestPoint(obstCenter); // perpendicular point
		if(perpPoint != p1 && perpPoint != p2)
		{
			if((perpPoint - obstCenter).length() < 0.45) // check distance
				return true;
		}
	}

	return false;
}

void WorldState::calcMaps()
{
	// --- All Obstacles Map --

	float persistence_obstacles = 0.8;
	mapObstacles->map->scale(persistence_obstacles);
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		mapObstacles->addHill(obstacles.at(i).obstacleInfo.absCenter, 1.0, 1.0 - persistence_obstacles);
	}
	mapObstacles->map->clamp(0.0, 1.0);

	// -- Dribble Map --
	mapDribble->clear();

	mapDribble->addOffset(Circle(me->coordinationVec, 3.0), 2.0, false); // avoid getting out of the 3m-radius circle
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		mapDribble->addHill(obstacles.at(i).obstacleInfo.absCenter, 2.0, 0.5); // grow in obstacle positions
	}

	XYRectangle recTheirPenaltyArea = XYRectangle(Vec(-field->penaltyAreaHalfWidth, field->halfLength + 4.0) , Vec(field->penaltyAreaHalfWidth, field->halfLength- field->penaltyAreaLength));
	mapDribble->addOffset(recTheirPenaltyArea, 2.0, true);

	XYRectangle recOurPenaltyArea = XYRectangle(Vec(-field->penaltyAreaHalfWidth - 0.7, -field->halfLength + field->penaltyAreaLength + 1.0) , Vec(field->penaltyAreaHalfWidth + 0.7, - field->halfLength - 4.0));
	mapDribble->addOffset(recOurPenaltyArea, 2.0, true);

	XYRectangle fieldRect = XYRectangle(Vec(-field->halfWidth + 0.5, field->halfLength - 0.5) , Vec(field->halfWidth - 0.5, - field->halfLength + 0.5));
	mapDribble->addOffset(fieldRect, 2.0, false);

	mapDribble->map->clamp(0.0, 2.0);

	mapDribble->addHill(me->pos, 2.0, -0.001); // dig in my position

	if(me->role == rStriker)
	{
		vector<int> receivers = getRoleIndex(rMidfielder,false);		// Get midfielder list
		for(unsigned int i = 0; i < receivers.size(); i++)
		{
			if( robot[receivers.at(i)].coordinationFlag[0] == LineClear
					|| robot[receivers.at(0)].coordinationFlag[i] == NotClear )	// If there's one of these flags
			{
				//mapDribble->addHill(robot[receivers.at(i)].pos, 3.0, -0.1); // dig for the receiver
			}
		}


	}

	// -- Map to Receive Ball in FreePlay --

	TCODMap fov = TCODMap(SAMPLE_SCREEN_WIDTH,SAMPLE_SCREEN_LENGTH);
	fov.clear(true,true);
	int ballX, ballY;
	mapReceiveBallFP->world2grid(me->ball.pos, ballX, ballY);
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		int x,y;
		mapReceiveBallFP->world2grid(obstacles.at(i).obstacleInfo.absCenter, x, y);
		//Ignore obstacles close to the ball for FOV effects; assume that the passer will be able to get around them
		bool closeObstacle = (obstacles.at(i).obstacleInfo.absCenter - me->ball.pos).length() < 1.5;
		if(!obstacles.at(i).obstacleInfo.isTeamMate() && !closeObstacle)
		{
			for(int ix = -2; ix <= 2; ix++)
				for(int iy = -2; iy <= 2; iy++)
				{
					fov.setProperties(x + ix,y + iy,false,false);
				}
		}
	}

	fov.computeFov(ballX, ballY, 0);

	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			float val = (fov.isInFov(x,y)) ? 0 : 2.0; // up where there is no Field of Vision
			mapReceiveBallFP->map->setValue(x,y,val);
		}
	}

	// 3x3 kernel for smoothing operations
//	static const int smoothKernelSize=9;
//	static const int smoothKernelDx[9]={-1,0,1,-1,0,1,-1,0,1};
//	static const int smoothKernelDy[9]={-1,-1,-1,0,0,0,1,1,1};
//	static const float smoothKernelWeight[9]={2,8,2,8,20,8,2,8,2};
	//mapReceiveBallFP->map->kernelTransform(smoothKernelSize, smoothKernelDx, smoothKernelDy, smoothKernelWeight, -1000, 1000);

	mapReceiveBallFP->addOffset(recOurPenaltyArea, 2.0, true);
	mapReceiveBallFP->addOffset(fieldRect, 2.0, false);

	mapReceiveBallFP->map->clamp(0.0, 2.0);

	// -- TheirGoal FOV --

	TCODMap fovTheirGoal = TCODMap(SAMPLE_SCREEN_WIDTH,SAMPLE_SCREEN_LENGTH);
	fovTheirGoal.clear(true,true);
	int theirGoalX, theirGoalY;
	mapTheirGoalFOV->world2grid(field->theirGoal, theirGoalX, theirGoalY);
	for(unsigned int i = 0; i < obstacles.size(); i++)
	{
		int x,y;
		mapTheirGoalFOV->world2grid(obstacles.at(i).obstacleInfo.absCenter, x, y);
		for(int ix = -1; ix <= 1; ix++)
			for(int iy = -1; iy <= 1; iy++)
			{
				fovTheirGoal.setProperties(x + ix,y + iy,false,false);
			}
	}

	fovTheirGoal.computeFov(theirGoalX, theirGoalY, 0);

	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			Vec pos = mapTheirGoalFOV->grid2world(x,y);
			float val = 0.0;
			if(!fovTheirGoal.isInFov(x,y))
			{
				val = 1000.0;
				if(!obstaclesToTheirGoal(1.5, pos))
					val = 0.0;
			}
			mapTheirGoalFOV->map->setValue(x,y,val);
		}
	}

	// Smoothing
	//mapTheirGoalFOV->map->kernelTransform(smoothKernelSize, smoothKernelDx, smoothKernelDy, smoothKernelWeight, -1000, 1000);


	mapKick2Goal->clear();

	// Add "boobs" map
	bool addBoobs = config->getParam("dribble_boobs_map") > 0.0;
	if(addBoobs)
	{
		mapKick2Goal->digHill( Vec(2*field->halfWidth/3 , field->halfLength - field->penaltyAreaLength*2.0) , 6, -3.0 );
		mapKick2Goal->digHill( Vec(-2*field->halfWidth/3 , field->halfLength - field->penaltyAreaLength*2.0) , 6, -3.0 );
		mapKick2Goal->add(mapObstacles);
	}

	mapKick2Goal->add(mapTheirGoalFOV);
	mapKick2Goal->add(mapDribble);

	mapKick2Goal->map->clamp(-1000, 2.0);

	// Create dead angle
	float deadAngle = 20;
	Line deadAngle1 = Line(field->theirGoal,field->theirGoal + Vec(0,-1).rotate(Angle(-M_PI/2 + deadAngle*M_PI/180)));
	Line deadAngle2 = Line(field->theirGoal,field->theirGoal + Vec(0,-1).rotate(Angle( M_PI/2 - deadAngle*M_PI/180)));
	mapKick2Goal->addOffset(deadAngle1, 2.0, true);
	mapKick2Goal->addOffset(deadAngle2, 2.0, false);

	mapKick2Goal->map->clamp(-1000, 2.0);


	// Go down to their goal
	float gainGoDownTheirGoal = 0.1;
	//Vec me2theirGoal = field->theirGoal - me->pos;
	Vec coordVec2theirGoal = field->theirGoal - me->coordinationVec;
	float distCoordVec2TheirGoal = coordVec2theirGoal.length();

	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			Vec pos = mapKick2Goal->grid2world(x,y);
			if((me->coordinationVec - pos).length() > 3.0)
				continue;

			float dist2theirgoal = (field->theirGoal - pos).length();
			float relDistance = (dist2theirgoal - distCoordVec2TheirGoal)/3.0; // 3m is the allowed dribble circle
			mapKick2Goal->map->setValue(x,y,mapKick2Goal->map->getValue(x,y) + gainGoDownTheirGoal*relDistance);
		}
	}



	//mapKick2Goal->map->clamp(0, 2.0);

	mapKick2Goal->normalize();

	// This block draws a "Color scale" for a normalized grid view
	/*for (float y = -field->halfLength; y <= field->halfLength; y+=SCALE )
	{
		Vec pos = Vec(field->halfWidth,y);
		int gX, gY;
		mapKick2Goal->world2grid(pos, gX, gY);
		float val = (y/field->halfLength + 1) / 2;
		mapKick2Goal->map->setValue(gX,gY,val);
	}*/

	mapKick2Goal->fillRtdb();

	//mapObstacles->fillRtdb();
}

HeightMap* WorldState::calcReceiverSPMap(Vec testPoint, float maxDistance, int idReplacer, int robotIdx ){
	float MIN_LINE_CLEAR = 1.0;
	Robot *me;
	me = &robot[robotIdx];
	Vec ball = getBestBallAbs();
	Vec ballRelPosition = abs2rel(ball, robotIdx); // Get relative ball
	ballRelPosition = ballRelPosition.setLength(ballRelPosition.length() * 0.8); // 2/3 of the way

	receiverSPMap->clear();
	TCODMap fovBall = TCODMap(SAMPLE_SCREEN_WIDTH, SAMPLE_SCREEN_LENGTH);
	TCODMap fovMe = TCODMap(SAMPLE_SCREEN_WIDTH, SAMPLE_SCREEN_LENGTH);
	fovBall.clear(true, true);
	fovMe.clear(true, true);
	int ballX, ballY;
	receiverSPMap->world2grid(ball, ballX, ballY);
	for (unsigned int i = 0; i < me->nObst; i++)
	{
		int x, y;
		receiverSPMap->world2grid(me->obstacles[i].absCenter, x, y);
		//Ignore obstacles close to the ball for FOV effects; assume that the passer will be able to get around them
		bool closeToBall =
				(me->obstacles[i].absCenter - ball).length() < 1.5;
		bool closeToMe = (me->obstacles[i].absCenter - me->pos).length() < 0.75;
		if (/*!me->obstacles[i].isTeamMate() &&*/!closeToBall)
		{ //considerar os teammates como obstaculos ou nao?
			for (int ix = -2; ix <= 2; ix++)
				for (int iy = -2; iy <= 2; iy++)
				{
					fovBall.setProperties(x + ix, y + iy, false, false);
					if(!closeToMe)
						fovMe.setProperties(x + ix, y + iy, false, false);
				}
		}
		if(closeToMe)
		for (int ix = -1; ix <= 1; ix++)
			for (int iy = -1; iy <= 1; iy++)
			{
				fovMe.setProperties(x + ix, y + iy, false, false);
			}
	}
	int px, py;
	receiverSPMap->world2grid(me->pos, px, py);
	fovBall.computeFov(ballX, ballY, 0);
	fovMe.computeFov(px, py, 0); // para optimizar pode-se limitar o raio de calculo do FOV

	Circle circle(testPoint, maxDistance);
	Circle distToBallRestrition(ball, 2.2);
	float minDistToBall = ((ball - testPoint).length() - maxDistance) < 0 ? 0.0 : (ball - testPoint).length() - maxDistance;
	float maxDistToBall = (ball - testPoint).length() + maxDistance;
	float minDistToGoal = ((field->theirGoal - testPoint).length() - maxDistance) < 0 ? 0.0 : (field->theirGoal - testPoint).length() - maxDistance;
	float maxDistToGoal = (field->theirGoal - testPoint).length() + maxDistance;
	for (int x = 0; x < SAMPLE_SCREEN_WIDTH; x++)
	{
		for (int y = 0; y < SAMPLE_SCREEN_LENGTH; y++)
		{
			Vec realPt = receiverSPMap->grid2world(x, y);
			if (circle.is_inside(realPt) && !distToBallRestrition.is_inside(realPt))
			{
				if (realPt.y <= 0.0)//our side of the field values range [1.0,2.0]
				{
					receiverSPMap->map->setValue(x, y,
							fabs(realPt.y / field->halfLength) + 1.0);
				}
				else // their side
				{
					if (fabs((realPt - field->theirGoal).angle().get_deg_180())
							> DEAD_ANGLE
							&& fabs(
									(realPt - field->theirGoal).angle().get_deg_180())
									< 180 - DEAD_ANGLE)
					{
//						fprintf(stderr,"%.2f\n",lineClear(realPt, rel2abs(ballRelPosition, robotIdx), idReplacer, 0.0, robotIdx));
						if (lineClear(rel2abs(ballRelPosition, robotIdx),realPt, idReplacer, 0.0, robotIdx)
								> MIN_LINE_CLEAR)
						{// more than MIN_LINE_CLEAR values range [0,0.5]
							receiverSPMap->map->setValue(x, y,
									( ( (fabs((ball - realPt).angle(field->theirGoal - realPt).get_deg_180()) / (180 * 2)) * config->getParam("set_play_receiver_angle"))+
									((((ball-realPt).length()-minDistToBall)/(maxDistToBall/0.5))*config->getParam("set_play_receiver_ball_distance"))+
									((((field->theirGoal-realPt).length()-minDistToGoal)/(maxDistToGoal/0.5))*config->getParam("set_play_receiver_goal_distance"))+
									(((testPoint-realPt).length()/(maxDistance/0.5))*config->getParam("set_play_receiver_move_distance"))
									)/(config->getParam("set_play_receiver_angle")
											+ config->getParam("set_play_receiver_ball_distance")
											+ config->getParam("set_play_receiver_goal_distance")
											+ config->getParam("set_play_receiver_move_distance")) );
						}//less than MIN_LINE_CLEAR values range [0.5, 1]
						else
						{
							receiverSPMap->map->setValue(x, y,
									((MIN_LINE_CLEAR
											- lineClear(rel2abs(ballRelPosition, robotIdx),
													realPt, idReplacer, 0.0, robotIdx))
											/ (MIN_LINE_CLEAR * 2))
											+ 0.5);
						}
					}
					else //Dead angle values range [1.0,2.0]
					{
						receiverSPMap->map->setValue(x, y,
								2.0	- (min(
											fabs((realPt - field->theirGoal).angle().get_deg_180()),
											180	- fabs(	(realPt	- field->theirGoal).angle().get_deg_180()))
											/ DEAD_ANGLE));

					}
				}
			}// outside of search zone
			else
				receiverSPMap->map->setValue(x, y, 2.0);
		}
	}

	for (int x = 0; x < SAMPLE_SCREEN_WIDTH; x++)
	{
		for (int y = 0; y < SAMPLE_SCREEN_LENGTH; y++)
		{
			if (!fovBall.isInFov(x, y) || !fovMe.isInFov(x, y))
				receiverSPMap->map->setValue(x, y, 2.0);
		}
	}
	//area fora do campo
	for (int i = 0; i < ((OFFSETX - field->halfWidth) / SCALE) + 1; i++)
	{
		for (int j = 0; j < SAMPLE_SCREEN_LENGTH; j++)
		{
			receiverSPMap->map->setValue(i, j, 2.0);
			receiverSPMap->map->setValue(SAMPLE_SCREEN_WIDTH - 1 - i, j, 2.0);
		}
	}
	for (int j = 0; j < ((OFFSETY - field->halfLength) / SCALE) + 1; j++)
	{
		for (int i = 0; i < SAMPLE_SCREEN_WIDTH; i++)
		{
			receiverSPMap->map->setValue(i, j, 2.0);
			receiverSPMap->map->setValue(i, SAMPLE_SCREEN_LENGTH - 1 - j, 2.0);
		}
	}
	//goal area
	for (int i = ((OFFSETX - field->goalAreaHalfWidth) / SCALE) - 1;
			i < OFFSETX / SCALE; i++)
	{
		for (int j = 0;
				j
						< ((OFFSETY
								- (field->halfLength - field->goalAreaLength))
								/ SCALE) + 1; j++)
		{
			receiverSPMap->map->setValue(i, j, 2.0);
			receiverSPMap->map->setValue(SAMPLE_SCREEN_WIDTH - 1 - i, j, 2.0);
			receiverSPMap->map->setValue(i, SAMPLE_SCREEN_LENGTH - 1 - j, 2.0);
			receiverSPMap->map->setValue(SAMPLE_SCREEN_WIDTH - 1 - i,
					SAMPLE_SCREEN_LENGTH - 1 - j, 2.0);
		}
	}
	return receiverSPMap;
}

bool WorldState::updateOpponentDribbling()
{
	static int cyclesWithOpponent;
	static int cyclesFree;
	static bool withOpponent = false;
	unsigned char opponentID = 0;
	unsigned int i;
	int threshold = 5;		//should be around 1 second

	if ( me->ball.visible )
	{
		bool teamEngaged = isTeamEngaged();

		for (i=0; i<obstacles.size(); i++)
		{
			if ( !(obstacles.at(i).obstacleInfo.isTeamMate()) &&
				((obstacles.at(i).obstacleInfo.absCenter - me->ball.pos).length() < 0.7) &&
				!teamEngaged )
			{
				opponentID = obstacles.at(i).obstacleInfo.id;
				cyclesWithOpponent++;
				break;
			}
		}

		if ( i>=obstacles.size() )
		{
			cyclesFree++;
		}

		if ( (cyclesWithOpponent > threshold) && (!withOpponent) )
		{
			withOpponent = true;
			cyclesFree = 0;
		}
		else if ( (cyclesFree > threshold) && (withOpponent) )
		{
			withOpponent = false;
			cyclesWithOpponent = 0;
		}
	}

	//Fill the RTDB with the flag to sinalize if I see an opponent dribbling
	if (withOpponent)
		me->opponentDribbling = opponentID;
	else
		me->opponentDribbling = 0;

	return me->opponentDribbling;
}

bool WorldState::isOpponentDribbling()
{
	//If I don't see an opponent dribbling, check if some of my mates sees
	for( int i = 0 ; i < N_CAMBADAS ; i++ )
	{
		if( robot[i].running && robot[i].opponentDribbling )
		{
			return true;
			break;
		}
	}

	return false;
}

bool WorldState::isBallPassedToMe()
{
	coordinationType search = (coordinationType)(BallPassed0 + getMyIdx());

	vector<int> passers;
	if(me->role == rMidfielder)
		passers = getRoleIndex(rStriker, false);
	else
		passers = getRoleIndex(rReplacer, false);

	if(passers.empty())
		return false;
	else
	{
		if(robot[passers.at(0)].coordinationFlag[0] == search)
			return true;
		else
			return false;
	}
}

bool WorldState::isOk2Kick() {
	return ok2kick;
}

void WorldState::ok2kick_update() {
	ok2kick = true;
}

bool WorldState::isBallEngagedChanged(bool engagedNow) {
	if(engagedNow)
		return (!lastCycleEngaged && me->ball.engaged);
	else
		return (lastCycleEngaged && !me->ball.engaged);
}

bool WorldState::isBallVisibleChanged(bool visibleNow) {
	if(visibleNow)
			return (!lastCycleVisible && me->ball.visible);
		else
			return (lastCycleVisible && !me->ball.visible);
}

void WorldState::setgrabberTouched(bool touched) {
	grabberWasTouched = touched;
}

bool WorldState::grabberTouched(bool byBall) {
	bool returnValue = false;
	if (byBall) {
		Circle aroundGrabPoint = Circle(Vec(0,0.25), 0.25);
		if( grabberWasTouched && me->ball.visible && me->ball.own && aroundGrabPoint.is_inside(me->ball.posRel)) {  // grabber has been touched by the ball
			returnValue = true;
		}
	}
	else
	{
		returnValue = grabberWasTouched;
	}
	return returnValue;
}

bool WorldState::isNearTeammate(Vec pos, double distance, int idxToIgnore=-1){
	for(int i=0; i< N_CAMBADAS;i++){
		if(!robot[i].running || i == idxToIgnore)
			continue;
		if((robot[i].pos - pos).length()< distance)
			return true;
	}
	return false;
}


} /* namespace cambada */
