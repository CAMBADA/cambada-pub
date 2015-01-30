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

#ifndef _BEHAVIOUR_H_
#define _BEHAVIOUR_H_

#include "WorldState.h"
#include "WorldStateDefs.h"
#include "ConfigXML.h"
#include "SlidingWindow.h"
#include "DriveVector.h"
#include "Strategy.h"
#include "common.h"

// Controllers
#include "controllers/CArc.h"
#include "controllers/CMove.h"
#include "controllers/CRotateAroundBall.h"
#include "controllers/CRotate.h"

#include <stdio.h>
#include <string.h>

namespace cambada {

/**
 * \brief Generic Behaviour class
 */
class Behaviour {
public:
	Behaviour();
	Behaviour(BehaviourID id);
	virtual ~Behaviour();

	static WorldState* world; 		// Pointer to worldstate
	static ConfigXML* config; 		// Pointer to configuration
	static Field* field;			// field object
	static Strategy* strategy;		// Pointer to strategy for decisions based on strategic positioning

	// static Controllers
	static CArc* cArc;
	static CMove* cMove;
	static CRotateAroundBall* cRotateAroundBall;
	static CRotate* cRotate;

	BehaviourID	behaviourRtti;		// The behaviour id

	/**
	 * Virtual function to be implemented in each behaviour
	 */
	virtual void calculate(DriveVector* dv);

	/**
	 * returns true if this is a reasonable action
	 */
	virtual bool checkCommitmentCondition(){ return true; }

	/**
	 * Returns true if all the requirements for this action were met
	 */
	virtual bool checkInvocationCondition() { return true; }

	/**
     * notifies the command generator, that it is to gain control during the
     * present control cycle. May be implemented by deliberative command
     * generators. May throw an exception, if it's not able to gain control.
     * In such a case checkCommitmentCondition should return false.
     * \param Time the expected execution time of the present control cycle */
    virtual void gainControl() {}

    /**
     * notifies the command generator, that it is to loose control in the
     * present control cycle. May be implemented by deliberative command
     * generators. The behavior must accept to loose control in every possible
     * time step, if it was active.
     * \param Time the expected execution time of the present control cycle */
    virtual void loseControl() {}

    /**
     * Returns the behaviour name
     */
    virtual std::string getName() const { return behaviour_names[behaviourRtti]; }

    /**
     * Prints call hierarchy
     */
    virtual void printHierarchy (unsigned int) const;

	/**
	 * gets the behaviourRtti
	 */
    virtual BehaviourID getRtti() { return behaviourRtti; }

};
}

#endif
