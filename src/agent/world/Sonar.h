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

#ifndef _SONAR_H_
#define _SONAR_H_

#include "Vec.h"
#include <vector>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
#define MAX_SONAR_DISTANCE 6.0
#define MIN_SONAR_DISTANCE 1.0
#define MAX_SONAR_OPEN 6.0
#define MIN_THRESHOLD_DISTANCE 0.1
#define MAX_THRESHOLD_DISTANCE 6.0
#define MIN_N_SONARS 4
#define MAX_N_SONARS 36


#define DEBUG_SONAR 0
#define DEBUG_SONAR_HARD 0
#define DEBUG_SONAR_OBST 0

namespace cambada
{

class Sonar
{
private:
	double			maxSonarOpening;		/*!<Defines maximum width of each sonar slice, <b>in meters</b>.*/
	double			maxSonarDistance;		/*!<Defines maximum detection distance of each sonar slice, <b>in meters</b>.*/
	double			thresholdDistance;		/*!<Defines the distance at which the slice becomes straight, a "corridor", <b>in meters</b>.*/
	int				numberOfSonars;			/*!<The total number of slices on the sonar.*/
	geom::Angle			angularOffset;			/*!<The angular separation between slices.*/
	vector<geom::Angle>	opening;				/*!<Defines how the slice opens, a function of distance in multiples of 10cm.*/

	double			robotRad;				/*!<The radius of the robot, <b>in meters</b>.*/
	double			bodyOversize;			/*!<A factor applied to the body radius to allow compensation of slice oppening on its source.*/
	int				numberOfSegments;		/*!<The number of segments to define the \link opening \endlink array.*/
	int				lastIndex;				/*!<The last sonar index used, for history purposes.*/
	bool			decelerationFlag;		/*!<Boolean to indicate that the robot should decelerate to avoid colision.*/
	double			topSpeed;				/*!<Maximum linear speed that the robot can have after considering deceleration (used in pair with \link decelerationFlag \endlink).*/

public:
	/*!Default constructor. Defines 18 slices for the sonar, maxSonarOpening and thresholdDistance are 1.5, maxSonarDistance is 3.0, 64 segments are created for the opening and no oversize is considered (bodyOversize is 1.0).*/
	Sonar( double robotRad = 0.25 );

	/*!Main constructor.
	\param maxSonarDistance the value for the attribute with the same name
	\param maxSonarOpening the value for the attribute with the same name
	\param thresholdDistance the value for the attribute with the same name
	\param numberOfSonars the value for the attribute with the same name
	\param bodyOversize the value for the attribute with the same name. <b>Default is 1.0, neutral factor.</b>
	\param numberOfSegments the value for the attribute with the same name. <b>Default is 64.</b>
	\param robotRad the value for the attribute with the same name. <b>Default is the CAMBADAs radius, 0.25 meters.</b>*/
	Sonar(double maxSonarDistance, double maxSonarOpening, double thresholdDistance, int numberOfSonars, double bodyOversize = 1.1, int numberOfSegments = 64, double robotRad = 0.25);

	/*!Destructor.*/
	~Sonar();

	// SET/GET FUNCTIONS
	/*!Resets the value of \link lastIndex \endlink to 0.*/
	void resetLastIndex();

	/*!Get the value of \link lastIndex \endlink.
	\return The current value of \link lastIndex \endlink.*/
	int getLastIndex();

	/*!Get the value of \link maxSonarDistance \endlink.
	\return The current value of \link maxSonarDistance \endlink.*/
	double getMaxSonarDistance();

	/*!Get the distance over which we will evaluate the obstacle distance, that is the distance of the obstacle to the considered out radius of the robot.
	\return The cosidered radius of the robot, given by robotRad * bodyOversize.*/
	double getRobotCenterOffset();

	/*!Gets the current value of \link decelerationFlag \endlink.*/
	bool getDecelFlag();

	/*!Gets the current value of \link topSpeed \endlink.*/
	double getTopSpeed();

	// GENERAL FUNCTIONS
	/*!This method prints the internal information about the sonar.*/
	void printSonar();

	/*!This method receives the target of the robot and a list of positions of the obstacles to avoid <b>(BOTH IN RELATIVE POSITIONS)</b>.
	\param target the target where the robot wants to go, given in coordinates relative to the robot.
	\param obstacles a vector with the points of the obstacles we wish to avoid, also given in relative coordinates.
	\param robotVel the current linear velocity of the robot for \link decelerationFlag \endlink set.
	\return The angle of the best free direction, according to the coded rules.*/
	geom::Angle getFreeDirection(const geom::Vec& target, const vector<geom::Vec>& obstacles, geom::Vec robotVel);

private:
	//Set functions for class attributes
	/*!Set for \link maxSonarOpening \endlink.
	\param value the value to set.*/
	void setMaxSonarOpening(double value);

	/*!Set for \link maxSonarDistance \endlink.
	\param dist the value to set.*/
	void setMaxSonarDistance(double dist);

	/*!Set for \link thresholdDistance \endlink.
	\param dist the value to set.*/
	void setThresholdDistance(double dist);

	/*!Set for \link numberOfSonars \endlink.
	\param n the value to set.*/
	void setNumberOfSonars(int n);

	/*!Set for \link bodyOversize \endlink.
	\param factor the value to set.*/
	void setBodyOversize(double factor);

	/*!Calculates \link angularOffset \endlink and fills \link opening \endlink based on the parameters set by the user.*/
	void setSonars();

	/*!Method to verify if the given sonar is free.
	\param sonarAngle the angle of the current sonar to be tested.
	\param obstacles the list of obstacles to avoid.
	\return True if the sonar is free, false otherwise.*/
	bool isSonarFree(geom::Angle sonarAngle, const vector<geom::Vec>& obstacles, bool isTarget=false, double targetDist=0.0);
	
	/*!Method to test if deceleration is needed. This method sets both \link decelerationFlag \endlink and \link topSpeed \endlink attributes.
	\param obstacles the list of obstacles.
	\param linearVelocity the linear velocity vector <b>IN RELATIVE COORDINATES FROM THE ROBOT</b>.*/
	void testForDecel(const vector<geom::Vec>& obstacles, const geom::Vec& linearVelocity);

	/*!Private method to identify the index obstacle which is closer to the current velocity vector of the robot.
	\param obstacles the list of obstacles.
	\param linearVelocity the linear velocity vector <b>IN RELATIVE COORDINATES FROM THE ROBOT</b>.
	\return The index of the closest obstacle.*/
	int closestObstacleIndex(const vector<geom::Vec>& obstacles, const geom::Vec& linearVelocity);
};
}
#endif // _SONAR_H_

