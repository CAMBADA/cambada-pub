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

#ifndef _STRATEGY_H_
#define _STRATEGY_H_

#include "Vec.h"
#include "WorldState.h"
#include "StrategyParser.h"
#include "Formation.h"
#include "CoachInfo.h"
#include "ConfigXML.h"

#include <vector>
using namespace std;

#define BALLOFFSETUPDATE 0.2

namespace cambada
{

class Strategy
{
public:
	Strategy(ConfigXML* config, WorldState* world = NULL);

	virtual bool loadFreePlay(char* fname);
	virtual bool loadSP(char* fname);
	virtual void updateFreePlay();
	virtual void updateSP(bool force=false);
	virtual void exchange();
	virtual void exchange(vector<Vec> positions, int gready=0);
	virtual void exchangeGreedy();
	virtual ~Strategy();

	virtual Vec getSPosition(int p)
	{
		//cerr << "---------------I'm " << p << " and SP is " << SPosition[p] << " **********\n";
		//cout << "---------------I'm " << p << " and SP is " << SPosition[p] << " **********\n";
		return SPosition[p];
	}
	;
	// TODO: getFormationList
	// TODO: setFormation

//protected:
	Vec SPosition[N_CAMBADAS];
	FormationInfo finfo;

	vector<Formation *> formationFreePlay;
	vector<Formation *> formationSP;

	WorldState* world;
	ConfigXML* config;
	Field* field;
private:
	void distanceRestrictions();
	void minDisPositions(float distToRob=0.5);
	Vec previousBall;
	WSGameState lastGameState;
};

}

#endif

