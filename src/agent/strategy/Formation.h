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

#ifndef _FORMATION_H_
#define _FORMATION_H_

#include "Vec.h"
#include "WorldStateDefs.h"
#include <iostream>
#include "WorldState.h"
#include "CoachInfo.h"
#include "ConfigXML.h"

using namespace std;

namespace cambada {

class Strategy;

class Formation 
{
public:
	string name;

        Formation() { world=0; config = 0; strategy = 0;};
        Formation( ConfigXML* config, WorldState* world ) ;

	virtual ~Formation() {};

	void setWorldConfigStrategy(WorldState * w, ConfigXML *conf,Strategy *st) {world = w; config = conf; strategy = st;};

	virtual bool load() { return false; };
	virtual void update() = 0;

	WorldState* world;
	ConfigXML* config;
	Strategy*  strategy;

};

class FormationSBSP : public Formation
{
public:
        FormationSBSP(void)  ;
        FormationSBSP( ConfigXML* config, WorldState* world ) ;

	virtual ~FormationSBSP() {};

	virtual void update();

	Vec	pos[N_CAMBADAS-1];
	Vec att[N_CAMBADAS-1];
	Vec min[N_CAMBADAS-1];
	Vec max[N_CAMBADAS-1];

	void dump()
	{
		cout << "Formation : "<<name<<endl << "----------"<<endl;
		
		for( int i = 0 ; i < N_CAMBADAS; i++ )
		{
			cout << "Player " <<i<<endl;
			cout << "\tPos  " <<pos[i]<<endl;
			cout << "\tAtt  " <<att[i]<<endl;
			cout << "\tMin  " <<min[i]<<endl;
			cout << "\tMax  " <<max[i]<<endl;
		}
	}
};

/*class FormationDT : public Formation
{
public:
        FormationDT(void) ;
        FormationDT( ConfigXML* config, WorldState* world ) ;

	virtual ~FormationDT() {};

	void update();
	bool load(char* fname);

        rcsc::Formation::Ptr M_formation;
};*/

}

#endif

