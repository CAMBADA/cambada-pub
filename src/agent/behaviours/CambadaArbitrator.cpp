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

#include "CambadaArbitrator.h"

namespace cambada {

CambadaArbitrator::CambadaArbitrator(): Behaviour(){
	currentBehIdx = -1;
	nextBehIdx = -1;
	behFinished = false;
}

CambadaArbitrator::CambadaArbitrator(BehaviourID id): Behaviour(id){
	currentBehIdx = -1;
	nextBehIdx = -1;
	behFinished = false;
}

CambadaArbitrator::~CambadaArbitrator() {
	for (unsigned int i=0; i < options.size(); i++) {
		delete options[i];
	}
}

void CambadaArbitrator::addOption (Behaviour* b){
	options.push_back(b);
}

void CambadaArbitrator::loseControl() {
	if (currentBehIdx>=0 && currentBehIdx < (int)(options.size())) {
		options[currentBehIdx]->loseControl();
	}
	currentBehIdx=nextBehIdx=-1;
}

bool CambadaArbitrator::checkConditions() {
	updateIntention();
	return (nextBehIdx >= 0 && nextBehIdx < (int)(options.size()));
}

void CambadaArbitrator::updateIntention() {
	nextBehIdx=options.size();  // Means no option selected

	bool wantContinue=false;
	behFinished=true;
	if (currentBehIdx >= 0 && currentBehIdx < (int)(options.size())) {
		if (options[currentBehIdx]->checkCommitmentCondition()) {
			wantContinue=true;
			behFinished=false;
			nextBehIdx=currentBehIdx;
		}
	}
	int maxCheck = (wantContinue ? currentBehIdx : options.size());
	for (int i=0; i<maxCheck; i++) {
		if (options[i]->checkInvocationCondition()) {
			nextBehIdx=i;
			break;
		}
	}
}

void CambadaArbitrator::calculate(DriveVector* dv) {
	if (nextBehIdx < 0)												// If there is no scheduled intention
		updateIntention();										// Calculate a possible intention

	if( currentBehIdx != nextBehIdx || behFinished ) {		// If scheduled != present intentions or the current intention is finished
		if(currentBehIdx >= 0 && currentBehIdx < (int)(options.size()))
		{
			options[currentBehIdx]->loseControl();				// Old behaviour loses control

			if( nextBehIdx < (int)(options.size()) ) {
				options[nextBehIdx]->gainControl();		// New behaviour gains control
			}
		} else if( nextBehIdx >= 0 && nextBehIdx < (int)(options.size()) ) {
			options[nextBehIdx]->gainControl(); 			// New behaviour gains control, there was no previous active option
		}
	}

	currentBehIdx = nextBehIdx;									// Update current intention
	if (currentBehIdx < (int)(options.size())) {				// If valid option
		options[currentBehIdx]->calculate(dv);					// Calculate the behaviour DriveVector
		behaviourRtti = options[currentBehIdx]->getRtti();		// Update behaviourRtti
	} else {																// If the option index was not valid
		fprintf(stderr,"\nERROR :: BDI no option executable\n");			// Print error
		behaviourRtti = bBehaviour;
		dv->allOff();
		// WARNING: BDI::calculate() called although no option is executable
	}

	nextBehIdx=-1;													// Reset scheduled intention
}

bool CambadaArbitrator::checkInvocationCondition() {
	return checkConditions();
}

bool CambadaArbitrator::checkCommitmentCondition() {
	return checkConditions();
}

int CambadaArbitrator::size() {
	return options.size();
}

void CambadaArbitrator::clear() {
	for (unsigned int i=0; i < options.size(); i++) {
		delete options[i];
	}
	options.clear();
}

void CambadaArbitrator::printHierarchy (unsigned int level) const {
	for (unsigned int i=0; i<level; i++)
		fprintf(stderr,"  ");
	fprintf(stderr,"%s (BDI)\n",getName().c_str());
	for (unsigned int i=0; i<options.size(); i++)
		options[i]->printHierarchy (level+1);
}

} /* namespace cambada */
