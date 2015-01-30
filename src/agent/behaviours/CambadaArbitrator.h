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

#ifndef CAMBADA_ARBITRATOR_H_
#define CAMBADA_ARBITRATOR_H_

#include "Behaviour.h"
#include "DriveVector.h"

namespace cambada {

class CambadaArbitrator: public Behaviour {
public:
	CambadaArbitrator();
	CambadaArbitrator(BehaviourID id);
	virtual ~CambadaArbitrator();

	virtual void calculate(DriveVector* dv); // Returns the DriveVector of the active option
	virtual bool checkInvocationCondition(); // true if the invocation condition of at least one option is true
	virtual bool checkCommitmentCondition(); // true if commitmentCondition of present intention is true or invocationCondition of another option is true
	virtual void loseControl();
    virtual void printHierarchy (unsigned int) const;

	void addOption (Behaviour* b);
	int size();
	void clear();

private:
	std::vector<Behaviour*> options;
	int currentBehIdx;
	int nextBehIdx;
	bool behFinished;

	void updateIntention ();
	bool checkConditions ();
};

} /* namespace cambada */
#endif /* CAMBADA_ARBITRATOR_H_ */
