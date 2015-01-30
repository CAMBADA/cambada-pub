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

#include "KickerConf.h"
#include <algorithm>

using namespace std;

double KickerConf::defaultHeight = 0.0;

void KickerTable::addEntry(int distance, int power){
    if(distance<0)
        return;

    mapping[distance] = power;  // Assign power to that distance

    // If there was no entry in the table
    if(find(distances.begin(), distances.end(), distance) == distances.end())
        distances.push_back(distance); // add entry in distances vector

    sort(distances.begin(), distances.end(), _distSort); // Sort distances table
}

void KickerTable::delEntry(int distance){
    if(mapping.count(distance) > 0){ // If this distance exists in table
        distances.erase(remove(distances.begin(), distances.end(), distance), distances.end());
        mapping.erase(distance);
    }

    sort(distances.begin(), distances.end(), _distSort); // Sort distances table
}

int KickerTable::size(){
    return distances.size();
}

void KickerTable::print(FILE* fout) {
    for(int i = 0; i < size(); i++){
        fprintf(fout,"%d %d\n", distances.at(i), mapping[distances.at(i)]);
    }
}

int KickerTable::getPower(int distance){
    if(size() == 0)
        return 0;
    if(size() == 1)
        return mapping[distances.at(0)];
    else if(find(distances.begin(), distances.end(), distance) != distances.end()) // If that distance exists in the table
        return mapping[distance];
    else {
        if(distances.at(0) > distance)                       // If distance is less than the first distance in the table
            return mapping[distances.at(0)];                 // return that distance

        int leftDistIdx = 0;
        while(leftDistIdx < size()-1 && distances.at(leftDistIdx+1) < distance)
            leftDistIdx++;

        if(leftDistIdx == size()-1)                         // check if we reached the end
            return mapping[distances.at(leftDistIdx)];      // In that case return the power for the highes distance
        else {                                              // Else, we still have points to the right
            int rightDistIdx = leftDistIdx+1;               // Then this is the point to the right

            float d1 = (float)distances.at(leftDistIdx);           // Distance 1
            float d2 = (float)distances.at(rightDistIdx);          // Distance 2
            float dDist = d2 - d1;                          // delta Distance
            float dPow = (float)mapping[d2] - (float)mapping[d1]; // delta Power

            float m = dPow/dDist;                           // Calc slope
            float b = (float)mapping[d1] - m*d1;            // calc b of the line

            return (int)(m*distance + b);                   // Return the value of the line in that distance
        }
    }
}

KickerConf::KickerConf(int agentNumber, char* file)
{
	if(agentNumber < 1 || agentNumber > 6){
		fprintf(stderr,"ERROR : KickerConf invalid agent (given %d)\n", agentNumber);
		exit(0);
	}
	currentAgent = agentNumber;                                 // Save current agent number
	load(file);
	if (!loadThroughParab())
	{
		fprintf(stderr,"ERROR LOADING kicker.map\n");
		exit(1);
	}
	world = NULL;
}

KickerConf::KickerConf(WorldState* pointer, int agentNumber, char* file)
{
	if(agentNumber < 1 || agentNumber > 6){
		fprintf(stderr,"ERROR : KickerConf invalid agent (given %d)\n", agentNumber);
		exit(0);
	}
	currentAgent = agentNumber;                                 // Save current agent number
	load(file);
	if (!loadThroughParab())
	{
		fprintf(stderr,"ERROR LOADING kicker.map\n");
		exit(1);
	}
	world = pointer;
}

bool KickerConf::load(char *file){
    FILE *fp = fopen(file, "r");                                // Open config file for reading
    if(fp == NULL)
        return false;

    int d;                                                      // Aux var for distance
    int k;                                                      // Aux var for kicking power
    for(unsigned int i = 0; i < 6; i++) {                       // For each robot
        int robotNumber = -1;                                   // get robot number
        int numEntries = -1;                                    // get number of entries in the table

        if(fscanf(fp,"R%d %d\n", &robotNumber, &numEntries) == 2 && robotNumber != -1 && numEntries != -1){ // If valid values
            KickerTable* nowTable = &table[robotNumber-1];
            nowTable->mapping.clear();
            nowTable->distances.clear();
            for(int e = 0; e < numEntries; e++){       // iterate through each entry in table
                if(fscanf(fp,"%d %d\n",&d,&k) == 2)
                    nowTable->addEntry(d,k);
            }
        }
    }
    fclose(fp);

    return true;
}

bool KickerConf::save(char *file){
    FILE *fp = fopen(file, "w");                                // Open config file for writing (discard contents)
    if(fp == NULL)
        return false;

    for(unsigned int i = 0; i < 6; i++) {                       // For each robot
        KickerTable* nowTable = &table[i];
        fprintf(fp, "R%d %d\n",i+1,nowTable->size());
        nowTable->print(fp);
    }

    fclose(fp);

    return true;
}

int KickerConf::getPower(float distance){
    int distanceCM = (int)(floor(distance*100.0 + 0.5));
    //fprintf(stderr,"GETPOWER @ %d = %d\n",distanceCM,table[currentAgent-1].getPower(distanceCM));
    return table[currentAgent-1].getPower(distanceCM);
}

KickerTable* KickerConf::getTablePtr(){
    return &table[currentAgent-1];
}

void KickerConf::setAgent(int agent)
{
    if(agent < 1 || agent > 6){
        fprintf(stderr,"ERROR : KickerConf invalid agent (given %d)\n", agent);
    }
    currentAgent = agent;
}
void KickerConf::print()
{
    for(int i=0; i<6; i++){
        fprintf(stderr,"R%d %d\n",i+1,table[i].size());
        table[i].print();
    }
}

bool KickerConf::loadThroughParab(char* file)
{
	FILE *fp = fopen(file, "r");                                // Open config file for reading
	char trash[100];

	if(fp == NULL)
		return false;

	float velContribution, vel15, vel20, vel25, vel30, vel35, vel40, vel45, vel50, ang, height;

	/* this is for skipping the lines that are not needed in the conf file */
	for(int i = 0; i < Whoami()-1; i++)
	{
		fgets (trash , 100 , fp);
	}
	fscanf(fp, "%f %f %f %f %f %f %f %f %f %f %f", &velContribution, &vel15, &vel20, &vel25, &vel30, &vel35, &vel40, &vel45, &vel50, &ang, &height);

	fclose(fp);

	vel.push_back(vel15);
	vel.push_back(vel20);
	vel.push_back(vel25);
	vel.push_back(vel30);
	vel.push_back(vel35);
	vel.push_back(vel40);
	vel.push_back(vel45);
	vel.push_back(vel50);

	defaultHeight = height;
	angDeg = ang;
	velRobotContribution = velContribution;

	return true;
}

int KickerConf::getPowerThroughParab(float distance, float height)
{
	int kickPower;
	double distOrig = distance;
	double g = 9.8067;
	double teta = (angDeg * M_PI) / 180.0;
	double v0, vkick;
	double mm=0.0, bb=0.0;
	int i, kick;

	//HACK "Correct" height according to XX speed
	ClippedRamp xSpeedFactor = ClippedRamp(0.0, 1.0, 1.74, 0.5, 0.0, 1.0);
	height *= xSpeedFactor.getValue(world->lowlevel.getVelX());

	distance = distance + height / tan(teta); //for the ball to enter approx. height meters of the ground on the goal line

	// for kicking purposes, directly use the odometry measured velocity
	double vRobot_y;
	vRobot_y = (world->lowlevel.getDY()/(MOTION_TICK/1000.0)) * velRobotContribution;
	if (vRobot_y < 0.0)
		vRobot_y *= 0.5;	//do not apply full correction when going back (ball was too high)

	//Correction of distance needed if predict() is not active
//	double actuationDelay = 0.08;	// 50 ms is mean, 80 ms is max
//	distance = distance - vRobot_y * actuationDelay;

	distance -= 0.27;	//0.27 is the robotCenter->ballCenter (following calcs are for the ball arc)

	v0 = sqrt(distance * g / sin(2 * teta));
	vkick = v0 - vRobot_y * cos(teta);

	bool afterLast = true;
	for(i = 1, kick = KICK_MIN; i < N_VALS; i++, kick += KICK_STEP)
	{
		if(vkick < vel.at(i))
		{
			mm = (vel.at(i)-vel.at(i-1)) / KICK_STEP;
			bb = vel.at(i-1) - mm * kick;
			afterLast = false;
			break;
		}
	}
	if(afterLast)
	{
		mm = (vel.at(vel.size()-1)-vel.at(vel.size()-2)) / KICK_STEP;
		bb = vel.at(vel.size()-2) - mm * kick;
	}

	kickPower = (int)((vkick - bb) / mm);

#if false
	printf("KICK_CALC dist=%5.3f / %5.3f, V0=%5.2f, vkick=%5.2f, vRobot=%5.3f (vX=%5.2f), KickPow=%d, justK:%d\n",
			distOrig, distance, v0, vkick, vRobot_y, world->lowlevel.getDX()/(MOTION_TICK/1000.0),
			kickPower, world->me->justKicked);
#endif

	return kickPower;
}
