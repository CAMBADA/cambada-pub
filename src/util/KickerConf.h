/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA UTILITIES
 *
 * CAMBADA UTILITIES is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA UTILITIES is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KICKERCONF_H
#define KICKERCONF_H

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <map>
#include "math.h"
#include "WorldState.h"
#include "ClippedRamp.h"

#define N_VALS		8
#define KICK_MIN	15
#define	KICK_STEP	5

class KickerTable
{
public:
    KickerTable(){}
    void addEntry(int distance, int power);
    void delEntry(int distance);
    int size(); // number of entries in the table
    int getPower(int distance); // get the power from a certain distance
    void print(FILE* fout = stderr);

    std::vector<int> distances;
    std::map<int,int> mapping;

private:
    struct distSort {
      bool operator() (int i,int j) { return (i<j);}
    } _distSort;
};

class KickerConf
{
public:
	KickerConf(int agentNumber, char* file = (char*)"../config/kicker.conf");
	KickerConf(WorldState* pointer, int agentNumber, char* file = (char*)"../config/kicker.conf");
	bool load(char* file = (char*)"../config/kicker.conf");
	bool save(char* file = (char*)"../config/kicker.conf");
	int getPower(float distance); // get the power from a certain distance (in meters)
	KickerTable* getTablePtr(); // returns the pointer to the table of the current agent
	void setAgent(int agent);
	void print();

	bool loadThroughParab(char* file = (char*)"../config/kicker.map");
	int getPowerThroughParab(float distance, float height=defaultHeight);

private:
	KickerTable table[6];
	int currentAgent;

	WorldState* world;

	std::vector<double> vel;		/*!<Velocity setPoints of measured power levels*/
	double angDeg;					/*!<Ball kick exit angle (mean of powers 25:50)*/
	static double defaultHeight;	/*!<Height to which we want to kick (defined on config file and can be defined on the function call*/
	double velRobotContribution;	/*!<Percentage of robot front velocity that contributes to ball exit velocity (default 0.5 on IRIS field)*/
};

#endif // KICKERCONF_H
