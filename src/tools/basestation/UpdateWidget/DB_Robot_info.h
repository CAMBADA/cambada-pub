/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA BASESTATION
 *
 * CAMBADA BASESTATION is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA BASESTATION is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DB_INFO_H
#define DB_INFO_H

#include "Robot.h"
#include <CoachInfo.h>
#include "SystemInfo.h"
#include "WorldStateDefs.h"
#include <time.h>

#include <QTime>

#include <iostream>

#define STATUS_NA 0
#define STATUS_OK 1
#define STATUS_SB 2
#define STATUS_KO 3

#define NROBOTS	  6

using namespace std;
using namespace cambada;

struct DB_Robot_Info
{
	Robot Robot_info[NROBOTS];
	char Robot_status[NROBOTS];
	long lifetime[NROBOTS];
	LaptopInfo lpBat[NROBOTS];
};


struct DB_Coach_Info
{
	CoachInfo Coach_Info;
	CoachInfo Coach_Info_in;
	
	QTime GameTime;
	int gTimeSecOffset;
	bool addLogTimeOffset;
	int logTimeOffset;

	time_t gameStartTime;
	char GamePart;

//	int ourGoals;
//	int theirGoals;	

	WSColor TeamColor;
	WSColor GoalColor;

	void print()
	{
		cout << "GameTime = ...; GamePart = " << int(GamePart) << endl
			<< "CoachInfo:\n"
			<< "\tgameState = " << Coach_Info.gameState << endl
			<< "\tourGoals = " << Coach_Info.ourGoals << endl
			<< "\ttheirGoals = " << Coach_Info.theirGoals << endl
			<< "----------------------------------------\n";
	}
};

#endif
