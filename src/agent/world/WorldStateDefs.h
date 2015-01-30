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

#ifndef _WORLDSTATEDEFS_H_
#define _WORLDSTATEDEFS_H_

#include "rtdb.h"

#define MAX_POINTS		500
#define MAX_BALLS		5	//maximum nunmber of possible balls (used for frontVision)
#define MAX_SHARED_OBSTACLES	20

#define MAX_LINES		500 // not used, using MAX_POINTS
#define MAX_OBSTACLES	360 // not used, using MAX_POINTS
#define MAX_GOAL		360	// maximum number of goal sensors
	
#define N_CAMBADAS		(N_AGENTS-1)
#define OBSTACLE_RADIUS 0.30

#define BASE_STATION	0

#define N_BATTERIES		3

#define V_9_6_VOLTAGE_LIMIT		7.1
#define V_12_0_VOLTAGE_LIMIT	13.5
#define STATUSLAPTOP_INFO_LIMIT	10

#define NOT_RUNNING_TIMEOUT				2000	// in ms	
#define NOT_RUNNING_VOLTAGE				13		// in deciVolt ;) : V*10

#define BALL_POSITION_BUFFER_SIZE		(20*33/MOTION_TICK)
#define BALL_POSITION_RESET_SIZE		(5*33/MOTION_TICK)

#define PLAYER_POSITION_BUFFER_SIZE		30

#define EGOMOTION_BUFFER_SIZE			30

#define CYCLES_NOT_VISIBLE_LIMIT		20 // 8
#define BALL_CYCLES_NOT_VISIBLE_LIMIT	CYCLES_NOT_VISIBLE_LIMIT
#define BALL_CYCLES_NOT_VISIBLE_LIMIT_FRONTVISION	2
#define GOAL_CYCLES_NOT_VISIBLE_LIMIT	CYCLES_NOT_VISIBLE_LIMIT
#define	BALL_ENGAGED_DISTANCE			0.5
#define BALL_ENGAGED_DEG				45.0

#define WAIT_TIME_SEC					10	// in seconds
#define PLAY_TIME_SEC					12 //in seconds - we have to finish the play in 7 seconds...
#define PASS_TIME_SEC					9	// in seconds 
#define MOVEAWAY_TIME_SEC				2	// in seconds			

#define USE_INTPASSIVE					1	// defines if striker uses passive interception when ball comes to our field

#define PASS_ONLY						0

#define BALL_CHALLENGE_ACTIVE			0

#define NO_AVOID_DISTANCE				0.4 /*!<Definition of the distance from the target position at which the avoidance is deactivated.*/

//definitions for the iterative interception point calculus (v2)
#define MAX_TIME 10.0
#define TIME_STEP 0.100
#define MAX_SPEED 3.0
#define DIST_ROBOT_BALL 0.30

#define DEAD_ANGLE 20

enum DangerType
{
	NoDanger,
	SideBand,
	OurBand,
	TheirBand
};

enum WSColor
{
	Blue,
	Yellow,
	Magenta,
	Cyan
};


enum BallType
{
	Own=0,
	Known,
	Unknown
};


enum KickEvaluation
{
	keTryToScore = 0,
	keTooFar,
	keDeadAngle,
	keNotAligned,
	keLineClear,
	keObstacle
};


enum CornerType
{
	CornerTypeDribble = 0,
	CornerTypePass,
	CornerTypeCross
};


enum AvoidLevel
{
	avoidNone = 0,
	avoidSemi = 1,
	avoidFull = 2

};



// NAO MUDAR A ORDEM GUS
enum WSGameState 
{ 
	stopRobot,               
	freePlay,
	preOwnKickOff,           
	postOwnKickOff,           
	preOpponentKickOff,      
	postOpponentKickOff,     
	preOwnGoalKick,          
	postOwnGoalKick,          
	preOpponentGoalKick,     
	postOpponentGoalKick,    
	preOwnCornerKick,        
	postOwnCornerKick,        
	preOpponentCornerKick,   
	postOpponentCornerKick,  
	preOwnThrowIn,           
	postOwnThrowIn,           
	preOpponentThrowIn,      
	postOpponentThrowIn,     
	preOwnFreeKick,          
	postOwnFreeKick,          
	preOpponentFreeKick,     
	postOpponentFreeKick,    
	preOwnPenalty,           
	postOwnPenalty,           
	preOpponentPenalty,      
	postOpponentPenalty,    
	dropBall,
	parking,
	errorState,
	postOwnGoal,
	postOpponentGoal               
};

inline WSGameState operator++( WSGameState &rs, int ) {
    return rs = (WSGameState)(rs + 1);
}

static const int num_referee_states = 31;
static const char referee_state_names [num_referee_states][25] = 
{
"Stop Robot              ",
"Play On	             ",
"Before Our KickOff      ",
"After Our KickOff       ",
"Before Their KickOff    ",
"After Their KickOff     ",
"Before Our GoalKick     ",
"After Our GoalKick      ",
"Before Their GoalKick   ",
"After Their GoalKick    ",
"Before Our CornerKick   ",
"After Our CornerKick    ",
"Before Their CornerKick ",
"After Their CornerKick  ",
"Before Our ThrowIn      ",
"After Our ThrowIn       ",
"Before Their ThrowIn    ",
"After Their ThrowIn     ",
"Before Our FreeKick     ",
"After Our FreeKick      ",
"Before Their FreeKick   ",
"After Their FreeKick    ",
"Before Our Penalty      ",
"After Our Penalty       ",
"Before Their Penalty    ",
"After Their Penalty     ",
"DropBall                ",
"Parking @ Home          ",
"GameStateError          ",
"After Own Goal          ",
"After Opp Goal          "
};

// NAO MUDAR A ORDEM GUS
enum WSRefereeSignal 
{
	SIGnop,
	SIGstop,
	SIGhalt,
	SIGstart,
	SIGready,
	SIGdropBall,
	SIGourKickOff,
	SIGtheirKickOff,
	SIGourFreeKick,
	SIGtheirFreeKick,
	SIGourGoalKick,
	SIGtheirGoalKick,
	SIGourCornerKick,
	SIGtheirCornerKick,
	SIGourThrowIn,
	SIGtheirThrowIn,
	SIGourPenalty,
	SIGtheirPenalty,
	SIGourGoalScored,
	SIGtheirGoalScored,
	SIGparking
};



static const int num_refbox_signals = 21;
static const char refbox_signal_names [num_refbox_signals][20] = 
{
"SIGnop             ",
"SIGstop            ",
"SIGhalt            ",
"SIGstart           ",
"SIGready           ",
"SIGdropBall        ",
"SIGourKickOff      ",
"SIGtheirKickOff    ",
"SIGourFreeKick     ",
"SIGtheirFreeKick   ",
"SIGourGoalKick     ",
"SIGtheirGoalKick   ",
"SIGourCornerKick   ",
"SIGtheirCornerKick ",
"SIGourThrowIn      ",
"SIGtheirThrowIn    ",
"SIGourPenalty      ",
"SIGtheirPenalty    ",
"SIGourGoalScored   ",
"SIGtheirGoalScored ",
"SIGparking         "
};


enum RoleID 
{	
	rNone,
	rStop,
	rGoalie ,
	rStriker ,
	rMidfielder,
	rTest,
	rReplacer,
	rBarrier,
	rCursor,
	rPenalty,
	rParking,
	rTour,
	rA,
	rB,
	rC,
	rReceiver,
	rReceiverFP,
	rKickCalibrate,
	rTaxi,
	rGoaliePenalty
};

static const int num_roles = 20;

static const char role_names [num_roles][16] =
{
	"No Role        ",
	"Stop           ",
	"Goalie         ",
	"Striker        ",
	"Midfielder     ",
	"Test           ",
	"Replacer       ",
	"Barrier        ",
	"Cursor         ",
	"Penalty        ",
	"Parking        ",
	"Tour           ",
	"A              ",
	"B              ",
	"C              ",
	"Receiver       ",
	"ReceiverFP     ",
	"Kick Calib     ",
	"Taxi           ",
	"Goalie Penalty "
};

enum BehaviourID 
{	
	bBehaviour, 
	bStopRobot,
	bMoveRel,
	bMoveAbs,
	bDribble,
	bSearchBall,
	bAvoidTheirGoalArea,
	bGoToVisibleBall,
	bDribbleOurField,
	bReceiverFPpos,
	bPass,
	bParking,
	bBarrier,
	bKick,
	bTest,
	bGoalieDefend,
	bAlign,
	bCatchBall,
	bActiveInter,
	bReceiver,
	bAvoid,
	bBlock,
	bRelieve,
	bTour,
	bTouchBall
};

static const int num_behaviours = 25;
static const char behaviour_names [num_behaviours][16] = 
{
	"bNoBehaviour   ",
	"bStopRobot     ",
	"bMoveRel       ",
	"bMoveAbs       ",
	"bDribble       ",
	"bSearchBall    ",
	"bAvTheirGArea  ",
	"bGoToVisibBall ",
	"bDribbleOurFiel",
	"bReceiverFPpos ",
	"bPass          ",
	"bParking       ",
	"bBarrier       ",
	"bKick          ",
	"bTest          ",
	"bGoalieDefend  ",
	"bAlign         ",
	"bCatchBall     ",
	"bActiveInter   ",
	"bReceiver      ",
	"bAvoid         ",
	"bBlock         ",
	"bRelieve       ",
	"bTour          ",
	"bTouch         "
};

enum coordinationType
{
	None = 0,
	BallPassed,
	BallPassed0,
	BallPassed1,
	BallPassed2,
	BallPassed3,
	BallPassed4,
	BallPassed5,
	TryingToPass,
	TryingToPass0,
	TryingToPass1,
	TryingToPass2,
	TryingToPass3,
	TryingToPass4,
	TryingToPass5,
	LineClear,
   	Ready,
	NotClear,
	NotFree,
	Receiver_1,
	Receiver_2,
	Receiver_3,
	KickCalibCode,
	Touch
};

static const int num_coordination_types = 24;
static const char coordination_names [num_coordination_types][12]=
{
	"None       ",
	"BallPassed ",
	"BallPassed0",
	"BallPassed1",
	"BallPassed2",
	"BallPassed3",
	"BallPassed4",
	"BallPassed5",
	"TryPass    ",
	"TryPass0   ",
	"TryPass1   ",
	"TryPass2   ",
	"TryPass3   ",
	"TryPass4   ",
	"TryPass5   ",
	"LineClear  ",
	"Ready      ",
	"NotClear   ",
	"NotFree    ",
	"Receiver_1 ",
	"Receiver_2 ",
	"Receiver_3	",
	"KickCalibC ",
	"Touch"
};

inline int idx2tryingToPass(int idx)
{
	switch(idx)
	{	
		case 0 : return TryingToPass0;
		case 1 : return TryingToPass1;
		case 2 : return TryingToPass2;
		case 3 : return TryingToPass3;
		case 4 : return TryingToPass4;
		case 5 : return TryingToPass5;
		default : return TryingToPass; 
	}
}

inline int tryingToPass2idx(int flag)
{
	switch(flag)
	{	
		case TryingToPass0 : return 0;
		case TryingToPass1 : return 1;
		case TryingToPass2 : return 2;
		case TryingToPass3 : return 3;
		case TryingToPass4 : return 4;
		case TryingToPass5 : return 5;
		default : return -1; 
	}
}

inline int idx2ballPassed(int idx)
{
	switch(idx)
	{	
		case 0 : return BallPassed0;
		case 1 : return BallPassed1;
		case 2 : return BallPassed2;
		case 3 : return BallPassed3;
		case 4 : return BallPassed4;
		case 5 : return BallPassed5;
		default : return BallPassed; 
	}
}

inline int ballPassed2idx(int flag)
{
	switch(flag)
	{	
		case BallPassed0 : return 0;
		case BallPassed1 : return 1;
		case BallPassed2 : return 2;
		case BallPassed3 : return 3;
		case BallPassed4 : return 4;
		case BallPassed5 : return 5;
		default : return -1; 
	}
}

#endif

