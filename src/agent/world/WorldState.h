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

#ifndef WORLDSTATE_H_
#define WORLDSTATE_H_

#include <time.h>
#include <sys/time.h>

#include "WorldStateDefs.h"
#include "CoachInfo.h"
#include "ConfigXML.h"
#include "geometry.h"
#include "Zones.h"
#include "HeightMap.h"
#include "Timer.h"
#include "LowLevelInfo.h"

#include "Field.h"
#include "Robot.h"
#include "Sonar.h"

namespace cambada {

/**
 * Class to maintain information on the state of the world,
 * either robot information (own and others), game state, obstacles,
 * free area mapping, ... . Has methods to query for various conditions,
 * utilities to transform Vec's, and other utilities to manipulate the
 * world state.
 * \brief Info and tools on the world state.
 */
class WorldState{
public:
	WorldState(ConfigXML* config);
	virtual ~WorldState();

	WSGameState gameState;

	Robot robot[N_AGENTS-1];		// Pointers to all robots
	static Robot* me;				// Pointer to me
	ConfigXML* config;				// Configuration pointer

	CoachInfo coach;

	vector<Obstacle> obstacles;
	vector<Obstacle> sharedObstacles;

	Sonar dribbleSonar;
	Sonar freeMoveSonar;

	// Height Maps
	void calcMaps();
	HeightMap* calcReceiverSPMap(Vec testPoint, float maxDistance,int idReplacer, int robotIdx = Whoami()-1);

	HeightMap* mapObstacles;
	HeightMap* mapDribble;
	HeightMap* mapReceiveBallFP;
	HeightMap* mapTheirGoalFOV;
	HeightMap* mapKick2Goal;
	HeightMap* receiverSPMap;

	// for Integrator:
	bool isFormationCoachAvailable;
	unsigned long timeStamp;

	LowLevelInfo lowlevel;

	Vec origBallPos;
	Timer timeAfterOurSetPiece;
	Timer timeAfterTheirSetPiece;

	int pointsRighOfBall;
	int pointsLeftOfBall;


	/**
	 * Returns a Field object with the current field characteristics
	 */
	Field* getField(){ return field; }

	/* FUNCTIONS */

	/**
	 * Translates and rotates a vector from the running agent relative coordinates to absolute coordinates
	 * \param rel vector in relative coordinates to transform
	 * \param robotIdx index of the base robot for the coordinate transformation <b>(default: caller agent)</b>
	 * \return transformed vector in absolute coordinates
	 */
	Vec rel2abs(const Vec& rel, int robotIdx=Whoami()-1);

	/**
	 * Rotates a vector from the running agend coordinates to absolute coordinates
	 * \param relative vector in agent coordinates to rotate
	 * \return rotated vector in absolute coordinates
	 */
	Vec rel2absDelta(const Vec&);

	/**
	 * Translates and rotates a vector from absolute coordinates to the running agent coordinates
	 * \param abs vector in absolute coordinates
	 * \param robotIdx index of the base robot for the coordinate transformation <b>(default: caller agent)</b>
	 * \return transformed vector in relative coordinates
	 */
	Vec abs2rel(const Vec& abs, int robotIdx=Whoami()-1);

	/**
	 * Translates and rotates a vector from absolute coordinates to the running agent coordinates
	 * 	\param position the position of the absolute coordinates center (typicaly an agent position)
	 * 	\param orientation the angle of the absolute coordinates center (typicaly an agent orientation)
	 * 	\param abs vector in absolute coordinates
	 * 	\return transformed vector in relative coordinates
	 */
	Vec abs2rel(const Vec& position , const double& orientation, const Vec& abs);

	/*! Rotates a vector from absolute coordinates to the running agent coordinates
		\param abs vector in absolute coordinates to rotate
		\return rotated vector in relative coordinates*/
	Vec abs2relDelta(const Vec&);

	/*!This method uses the FreeSensor to find an adjusted RELATIVE position for the robot to move while avoiding obstacles.
		\param targetRel the RELATIVE position where the robot originaly wants to move.
		\param moveFree a boolean indicating if the robot is moving freely or not. <b>Dribble must use true on this parameter.</b>
		\param avBall a boolean indicating if the ball is also to be avoided. <b>Default is false, the ball should not be avoided.</b>
		\param avLevel the avoidance level desired \todo Levels are not really implemented. <b>Default is full avoidance.</b>
		\return A position adjusted according to the FreeSensor, so the robot will go through an obstacle free area.*/
	Vec getAvoidAdjustedPosition(Vec targetRel, bool moveFree, bool avBall=false, AvoidLevel avLevel=avoidFull);

	/*! Checks the ball distance and position seen by all the robots (including itself)
		\return the ball absolute position viewed by the robot that is closest to it*/
	Vec getBestBallAbs(bool restrictToVisible = true) const;

	/*! Checks if the ball is visible by any robot*/
	bool isBallVisible() const;

	/**
	 * Gets a vector of indexes for all running robots, excluding the goalie
	 * \return a vector of robot's Idx (starting from 0)
	 */
	vector<int> getRunningFieldRobotsIdx();

	/**
	 * \return the number of running robots in field
	 */
	int getNumberOfRunningFieldRobots();

	/** Checks for a teammate with a given role
	 * \param role the role id we wish to search for
	 * \param meIncluded if true include me in the list
	 * \return a vector of indexes of teammates with the given role active (empty if none has the role)
	 */
	vector<int> getRoleIndex(RoleID role, bool meIncluded=false);

	/**
	 * Returns a Robot object with specified number
	 */
	const Robot playerWithNumber( unsigned int player ) const;

	/**
	 * Gets current agent IDX (starting at 0)
	 */
	int getMyIdx();

	/**
	 * Returns true if ball in our possession
	 */
	bool isTeamEngaged();

	/**
	 * \brief Returns how clear is a line (float value)
	 */
	float lineClear( Vec origin, Vec destiny, int indexToIgnore, float obsIgnoreDist = 0.0, int robotIdx=Whoami()-1);

	/*! Checks if the line between mySelf and an absolute position is free of obstacles (within a few cm from the theoretical value)
	\param absPosition the absolute position of the end of the line
	\return true if none of the objects is in the desired line*/
	bool isLineClear(Vec absPosition, double safetyDist, int indexToIgnore=-1, Vec ownPos = me->pos, float obsIgnoreDist = 0.0, int robotIdx=Whoami()-1);

	/**
	 * \brief returns true if there are obstacles in front of me (not letting me kick)
	 */
	bool obstaclesInFront(float distance);

	/**
	 * \brief returns true if there are obstacles between me and their goal
	 */
	bool obstaclesToTheirGoal(float distance, Vec position = me->pos);

	/*! Verifies if the ball is being dribbled by an opponent, by verifying if any of the team robots is seeing it.
	* \return True if the an opponent is dribbling the ball
	*/
	bool isOpponentDribbling();

	/*! updates the robot's opponent dribbling flag by analyzing the current obstacles and ball
	 * \return True if the robot sees an opponent dribbling
	 */
	bool updateOpponentDribbling();

	/**
	 * Parking timer
	 */
	double parkingTimeMS();
	void resetParkingTimer();

	void update();
	void updateEndCycle();

	Vec getScalingFactor();

	double getGoalSideOffset(double dist);
	double getGoalSideOffset(double dist, float x);

	bool isMovingOutside(Vec movePos,Vec& clippedPos);
	Angle getVectorAlignment(Vec v1, Vec v2);
	double getBallPathAlignment();
	bool isBallRollingAwayOrStopped();
	Vec getReplacerSearchPos();
	Vec getOpponentGoalie();
	bool isBallGoingInMyDirection(double angle=M_PI_2);

	bool isBallPassedToMe();

	/**
	 * Was ball engaged/disengaged on this cycle
	 * \param engagedNow true - change from disengaged to engaged, false otherwise
	 */
	bool isBallEngagedChanged(bool engagedNow);

	/**
	 * Has ball appeared/disappeared on this cycle
	 * \param visibleNow true - change from not visible to visible, false otherwise
	 */
	bool isBallVisibleChanged(bool visibleNow);

	/**
	 * Checks if I can kick to their goal, whatever the conditions are (rules)
	 */
	bool isOk2Kick();

	/**
	 * Checks if the grabber arms have been touched this cycle
	 * \input byBall Test for ball touching the grabber, false tests for grabber raised
	 * \return True if grabber raised
	 */
	bool grabberTouched(bool byBall);
	void setgrabberTouched(bool touched);

	/**
	 * Checks if the Vec is near a Teammate
	 */
	bool isNearTeammate(Vec pos, double distance, int idxToIgnore);

private:
	Zones* SetPiecesZones;
	double parkingTimer;
	Field* field;

	bool lastCycleEngaged; // was ball engaged last cycle?
	bool lastCycleVisible; // was ball visible last cycle?

	bool ok2kick; // can I kick to their goal (limited by the rules)
	bool grabberWasTouched;	/*!< At least one grabber arm was touched (raised) on the current cycle*/

	void ok2kick_update();

};

} /* namespace cambada */
#endif /* WORLDSTATE_H_ */
