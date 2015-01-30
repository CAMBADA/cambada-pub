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

#include "UseCompass.h"
#include <syslog.h>

namespace cambada {

// Constructor
UseCompass::UseCompass(ConfigXML* conf)
{
	// Create internar objects
	loc =  new loc::CambadaLoc(conf);
	compass = Compass(conf->getField("theNorth"));
	this->config = conf;
}

// Destroctor
UseCompass::~UseCompass()
{
	delete loc;
	compass.~Compass();
}

void UseCompass::mirror()
{
	loc->mirror();

	Vec pos;
	Angle ori;
	data.errPos = loc->GetRobotPosition(pos,ori);
	data.errLoc = getLocVsCompassDegError();
	data.position = pos / 1000.0;
	data.orientation = ori.get_rad();
	data.oriEarth = getRobotOrientationRegardingEarth();
}

// Function integrate
void UseCompass::integrate(vector<Vec> vision_lines, float lowLevelDx, float lowLevelDy, WSColor goalColor, bool firstTime)
{
	// Update compass values
	updateCompass(goalColor);

	// Get Heading
	CMD_Imu info;
	DB_get( Whoami() , CMD_IMU, (void*)(&info) );
	Angle currentHeading = Angle(info.rawYaw / 180.0 * M_PI);

	// Update loc
	if(firstTime)
		loc->FindInitialPositionWithKnownOrientation( vision_lines, compass.getCompass() );
	else
		data.errPos = loc->UpdateRobotPosition_KF(vision_lines,lowLevelDx,lowLevelDy, (currentHeading-lastHeading).get_rad_pi());

	// update measures
	Vec pos;
	Angle ori;
	double errPos = loc->GetRobotPosition(pos,ori);
	data.errPos = (firstTime)? errPos : data.errPos;
	data.errLoc = getLocVsCompassDegError();
	data.oriEarth = getRobotOrientationRegardingEarth();
	data.position = (firstTime)? pos : pos / 1000.0;
	data.orientation = ori.get_rad();
	lastHeading = currentHeading;

	// printHeadingsToSyslog();
}

void UseCompass::updateCompass(WSColor goalColor)
{
	// Update compass goalColor
	compass.setGoalColor(goalColor);

	// Read and update other compass atributes
	int compassRead = 0;
	//if( DB_get( Whoami() , CMD_COMPASS, (void*)(&compassRead) ) != -1 )
	//if( DB_get( Whoami() , CMD_IMU, (void*)(&compassRead) ) != -1 )
	CMD_Imu info;
	if( DB_get( Whoami() , CMD_IMU, (void*)(&info) ) != -1 )
	{
		compassRead = info.yaw;
		
		Angle angle;
		angle.set_deg(config->getField("theNorth"));
		compass.setTheNorth( angle );

		compass.update( (double)(compassRead) );
		compass.updateTrue( (double)(info.compass) );
	}
}

double UseCompass::getLocVsCompassDegError()
{
	Vec pos;
	Angle ori;
	loc->GetRobotPosition(pos,ori);
	return fabs((ori-(compass.getCompass()) ).get_deg_180() );
}

int UseCompass::getRobotOrientationRegardingEarth()
{
	Vec pos;
	Angle ori;
	loc->GetRobotPosition(pos,ori);
	return (int)(ori+(compass.getTheNorth()) ).get_deg_180();
}

void UseCompass::printHeadingsToSyslog()
{
	Vec pos;
	Angle ori;
	loc->GetRobotPosition(pos,ori);
	double degError = fabs((ori-(compass.getCompass())).get_deg_180() );
	syslog(LOG_DEBUG, "GoalColor error: %.2f VisualOri: %.2f, compassOri: %.2f",degError,ori.get_deg_180(),compass.getCompass().get_deg_180());
}

}/* namespace cambada */
