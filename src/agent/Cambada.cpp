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

#include "Cambada.h"
#include "Field.h"
#include "Behaviour.h"

using namespace cambada;

namespace cambada {

Cambada::Cambada() {

	config = new ConfigXML();						// Create the config object of type ConfigXML
	config->parse("../config/cambada.conf.xml");	// Parse xml configuration file

	world = new WorldState(config); 		// Create the world object of type WorldState
	field = world->getField();						// Create field object

	world->robot[Whoami()-1].number = Whoami();
	WorldState::me	= &world->robot[Whoami()-1];	// TODO: change this to WorldState constructor

	// Initialize static objects
	Controller::world	= world;
	Controller::config	= config;

	Behaviour::world	= world;
	Behaviour::config	= config;
	Behaviour::field	= field;

	// Init behaviour controllers
	Behaviour::cArc 	= new CArc();						// For arc movements
	Behaviour::cMove 	= new CMove();						// For straight movements
	Behaviour::cRotateAroundBall = new CRotateAroundBall();	// For rotations around the ball
	Behaviour::cRotate = new CRotate();						// For rotations around the robot

	Role::field 		= field;							// set Role::field static object
	Role::world			= world;							// set Role::world static object
	Role::config		= config;							// set Role::config static object

	strategy	= new Strategy(config,world);				// Init strategy
	integrator	= new Integrator(config,world,strategy); 	// Init integrator

	reconfigure();

	Behaviour::strategy = strategy;							//Assign the strategy pointer to the Behavior static pointer

	Role::strategy = strategy;								// Assign strategy to Role static field
	dv = new DriveVector(world);

	//Behaviour::ktable	= new KickerTable(Whoami());
	decision	= new Decision(world);			// Init decision
}

Cambada::~Cambada()
{
	delete decision; decision = NULL;

	delete Behaviour::cArc; Behaviour::cArc = NULL;
	delete Behaviour::cMove; Behaviour::cMove = NULL;
	delete Behaviour::cRotateAroundBall; Behaviour::cRotateAroundBall = NULL;
	delete Behaviour::cRotate; Behaviour::cRotate = NULL;

	delete strategy; strategy = NULL;
	delete config; config = NULL;
	delete integrator; integrator = NULL;
	delete dv; dv = NULL;
	delete world; world = NULL;
}

void Cambada::printHelp()
{
	fprintf(stdout,"The Cambada Agent Help:\n\n");
	fprintf(stdout,"\t-nc, --nocoach	\n\n");
}

bool Cambada::parseArguments( int argc , char* argv[] )
{
	Robot* me = world->me;

	if(me == NULL)
		return false;

	me->roleAuto = true;

	for( int i = 0 ; i < argc ; i++ ) {
		if( strcasecmp(argv[i] , "-h") == 0 || strcasecmp(argv[i] , "--help") == 0 ) {
			printHelp();
			return false;
		}else if( strcasecmp(argv[i] , "-nc") == 0 || strcasecmp(argv[i] , "--nocoach") == 0 ) {
			me->coaching = false;
		}
	}

	return true;
}

void Cambada::thinkAndAct()
{
	struct timeval tv;
	int time, t1, t2, t3, t4, t5;
	gettimeofday(&tv,NULL);
	time = tv.tv_sec*1000+tv.tv_usec/1000;

	integrator->integrate();

	gettimeofday(&tv,NULL);
	t1 = tv.tv_sec*1000+tv.tv_usec/1000; // TIME 1

	if (world->gameState == preOpponentKickOff || world->gameState == postOpponentKickOff
			|| world->gameState == preOpponentGoalKick || world->gameState == postOpponentGoalKick
			|| world->gameState == preOpponentThrowIn || world->gameState == postOpponentThrowIn
			|| world->gameState == preOpponentFreeKick || world->gameState == postOpponentFreeKick
			|| world->gameState == preOpponentCornerKick || world->gameState == postOpponentCornerKick
			|| world->gameState == dropBall)
	{
		strategy->updateSP();
	}
	else
	{
		strategy->updateFreePlay();
	}

	gettimeofday(&tv,NULL);
	t2 = tv.tv_sec*1000+tv.tv_usec/1000; // TIME 2

	// Update Agent HeightMaps
	world->calcMaps();

	gettimeofday(&tv,NULL);
	t3 = tv.tv_sec*1000+tv.tv_usec/1000; // TIME 3

	//Initialize kickPower and grabberMode
	dv->kickPower = 0;											// kickPower is 0 by default
	dv->grabber = GRABBER_DEFAULT;								// reset grabber state to default
	dv->smallAdjustment = false;								// by default, no small adjustments
	if(dv->passedCounter < 2014) dv->passedCounter++;			// Increment passedCounter
	if(dv->kickedCounter < 2014) dv->kickedCounter++;			// Increment kickedCounter

	// Restart controller variables
	Behaviour::cArc->cycleRestart();
	Behaviour::cMove->cycleRestart();
	Behaviour::cRotateAroundBall->cycleRestart();
	Behaviour::cRotate->cycleRestart();

	decision->decide(dv);										// Call DECISION LAYER

	config->checkConpensators(); 								// reset all not used compensators

	gettimeofday(&tv,NULL);
	t4 = tv.tv_sec*1000+tv.tv_usec/1000; // TIME 4

	if (dv->grabber == GRABBER_DEFAULT)							// If grabber state was not set
		dv->grabberControl();									// Call default grabberControl()

	if( world->me->role != rCursor ) {							// If im in cursor role, then it will generate the CMDs...
		if( world->me->running ) {								// If im running

			rampVelA();											// Limit angular velocity if needed
			CMD_Vel_SET(dv->velX, dv->velY, dv->velA, dv->smallAdjustment );	// Set velocities command
			if( dv->kickPower > 0 && world->me->ball.engaged )	// If ball engaged and kickPower > 0
			{
				CMD_Kicker_SET(dv->kickPower);					// Set kickPower
				if( (dv->kickPower & 0x80) > 0) 				// is a pass
					dv->passedCounter = 0;						// restart passedCounter
				else if(dv->kickPower > 0)						// is a kick
					dv->kickedCounter = 0;						// restart kickedCounter
			}
			else
				CMD_Kicker_SET(0);								// Else disable kicker

			CMD_Grabber_SET(dv->grabber);
		} else {												// If not running...
			CMD_Vel_SET( 0.0 , 0.0 , 0.0 , false );				// Stop robot
			CMD_Grabber_SET( 0 );								// Disable grabber
			CMD_Kicker_SET(0);									// Disable kicker
		}
	}

	DB_put( ROBOT_WS , (void*)(world->me) );

	world->updateEndCycle();

	gettimeofday(&tv,NULL);
	t5 = tv.tv_sec*1000+tv.tv_usec/1000; // TIME 5

	fprintf(stderr, "Agent[%1d]: %3d ms (int %3d + strat %3d + maps %3d + dec %3d + CMD %3d)\n",world->me->number, t5 - time, t1-time, t2-t1, t3-t2, t4-t3, t5-t4);
}

bool Cambada::reconfigure()
{
	bool parserResult = config->parse("../config/cambada.conf.xml");
//	bool parserResult2 = strategy->loadFreePlay((char *)"../config/formation.conf");
//	bool parser_SetPiecesFormation = strategy->loadSP((char *)"../config/setpieces.conf");
	bool parserResult4 = true;//Behaviour::ktable->load("../config/kicker.map");
	bool parserResult5 = true;//world->myField->load("../config/rawfield.txt");
	bool parserResult6 = true;

	/*try
	{
		auto_ptr<SetPieces> tmpSetPieces (SetPieces_("../config/SetPieces.xml"));
		setPieces 	= new SetPieces(*tmpSetPieces);  // TODO: eliminate memory leak
	}catch(...)
	{
	                parserResult6 = false;
	}*/
	return (parserResult && parserResult4 && parserResult5 && parserResult6);// && parserResult2);// && parser_SetPiecesFormation);
}

void Cambada::rampVelA()
{
	// limit velA when ball is engaged
	if(world->me->ball.engaged) {
		float maxDiff = 0.08;						// Max difference between cycles
		float diff = dv->velA - lastVelA;			// Acceleration difference

		if(fabs(diff) > maxDiff) {
			if(diff < 0)
				dv->velA = lastVelA - maxDiff;
			else if(diff > 0)
				dv->velA = lastVelA + maxDiff;
		}

		dv->limitVel(MAX_SPEED, M_PI);				// Now limit the maximum angular velocity
	}

	lastVelA = dv->velA;
}

} /* namespace cambada */
