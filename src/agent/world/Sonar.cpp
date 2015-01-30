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

#include "Sonar.h"

#if DEBUG_SONAR
	#include <sys/time.h>
	#include <time.h>
#endif

namespace cambada
{

using namespace cambada::geom;

Sonar::Sonar( double robotRad )
{
	this->robotRad = robotRad;
	maxSonarOpening = 1.5;
	maxSonarDistance = 3.0;
	thresholdDistance = 1.5;
	numberOfSonars = 18;
	numberOfSegments = 64;
	bodyOversize = 1.1;
	lastIndex = 0;
	decelerationFlag = false;

	setSonars();
}


Sonar::Sonar(double maxSonarDistance, double maxSonarOpening, double thresholdDistance, int numberOfSonars, double bodyOversize, int numberOfSegments, double robotRad)
{
	this->robotRad = robotRad;
	setMaxSonarDistance(maxSonarDistance);
	setMaxSonarOpening(maxSonarOpening);
	setThresholdDistance(thresholdDistance);
	setNumberOfSonars(numberOfSonars);
	setBodyOversize(bodyOversize);
	this->numberOfSegments = numberOfSegments;
	lastIndex = 0;
	decelerationFlag = false;

	setSonars();
}


Sonar::~Sonar()
{}


void Sonar::setMaxSonarOpening(double value)
{
	maxSonarOpening = value;
	if ( maxSonarOpening < (robotRad*bodyOversize) )
	{
		maxSonarOpening = robotRad * bodyOversize;
	}
	if ( maxSonarOpening > MAX_SONAR_OPEN )
	{
		maxSonarOpening = MAX_SONAR_OPEN;
	}
}

void Sonar::setMaxSonarDistance(double dist)
{
	maxSonarDistance = dist;
	if ( maxSonarDistance < (robotRad+MIN_SONAR_DISTANCE) )
	{
		maxSonarDistance = robotRad + MIN_SONAR_DISTANCE;
	}
	if ( maxSonarDistance > MAX_SONAR_DISTANCE )
	{
		maxSonarDistance = MAX_SONAR_DISTANCE;
	}
}

void Sonar::setThresholdDistance(double dist)
{
	thresholdDistance = dist;
	if ( thresholdDistance < (robotRad+MIN_THRESHOLD_DISTANCE) )
	{
		thresholdDistance = robotRad + MIN_THRESHOLD_DISTANCE;
	}
	if ( thresholdDistance > MAX_THRESHOLD_DISTANCE )
	{
		thresholdDistance = MAX_THRESHOLD_DISTANCE;
	}
}

void Sonar::setNumberOfSonars(int n)
{
	numberOfSonars = n;
	if ( numberOfSonars < MIN_N_SONARS )
	{
		numberOfSonars = MIN_N_SONARS;
	}
	if ( numberOfSonars > MAX_N_SONARS )
	{
		numberOfSonars = MAX_N_SONARS;
	}
}

void Sonar::setBodyOversize(double factor)
{
	bodyOversize = factor;
	if ( bodyOversize < 0.0 )
	{
		bodyOversize = 0.0;
	}
}


void Sonar::setSonars()
{
	double deltaX, deltaY, m, X, Y;

	opening.clear();
	for (int i=0; i<numberOfSegments; i++)
	{
		opening.push_back(Angle());
	}

	#if DEBUG_SONAR
	printSonar();
	#endif

	angularOffset.set_deg(360.0/(double)numberOfSonars);		//sets the angular separation between sonar slices
	deltaX = (maxSonarOpening/2) - (robotRad*bodyOversize);
	deltaY = thresholdDistance;
	m = deltaY / deltaX;
	for (int n=0; n<numberOfSegments; n++)
	{
		Y = n*0.1;
		X = (Y/m)+(robotRad*bodyOversize);
		if ( X > (maxSonarOpening/2) )
		{
			X = maxSonarOpening/2;
		}
		opening.at(n).set_rad( (M_PI/2.0) - atan2(Y,X) );
		#if DEBUG_SONAR_HARD
		printf("SONAR Opening %d -> %f [-180;180] OR %f [0;360]\n", n, opening.at(n).get_deg_180(), opening.at(n).get_deg() );
		#endif
	}
}


void Sonar::resetLastIndex()
{
	lastIndex = 0;
}

int Sonar::getLastIndex()
{
	return lastIndex;
}

double Sonar::getMaxSonarDistance()
{
	return maxSonarDistance;
}

double Sonar::getRobotCenterOffset()
{
	return robotRad*bodyOversize;
}

bool Sonar::getDecelFlag()
{
	return decelerationFlag;
}

double Sonar::getTopSpeed()
{
	return topSpeed;
}


void Sonar::printSonar()
{
	printf("SONAR Robot radius: %f, Number of sonars: %d, Number of segments: %d, Body oversize: %f\n", robotRad, numberOfSonars, numberOfSegments, bodyOversize);
	printf("SONAR Distance: %f, Opening: %f, Threshold distance: %f\n", maxSonarDistance, maxSonarOpening, thresholdDistance);
}


Angle Sonar::getFreeDirection(const Vec& target, const vector<Vec>& obstacles, Vec robotVel)
{
	#if DEBUG_SONAR
	struct timeval deltaTime;
	unsigned long startTime;
	gettimeofday( &deltaTime , NULL );
	startTime = deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000;
	fprintf(stderr,"SONAR Starting sonar evaluation\n\n");
	#endif

	int halfSonars = (int)(numberOfSonars/2);
	int posIndex, negIndex;		//index to positive or negative direction
	Angle targetAngle = target.angle();	//Gets the angle between the robot and the target (meaning this is the initial angle to consider for the sonars)
	Angle currentSonarAngle;

	/**Check the target angle (sensor 0)*/
	if ( isSonarFree(targetAngle, obstacles, true, target.length()) )
	{
		#if DEBUG_SONAR
		gettimeofday( &deltaTime , NULL );
		fprintf(stderr,"SONAR Sonar 0 (target) with %fº was chosen. Took %ld ms\n", targetAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
		#endif
		lastIndex = 0;
		decelerationFlag = false;
		return targetAngle;
	}
	else
	{
		#if DEBUG_SONAR
		fprintf(stderr,"SONAR Sonar 0 (target) is occupied: %fº\n", targetAngle.get_deg());
		#endif
	}

	/**Check from the lastIndex used*/
	if ( lastIndex != 0 )
	{
		#if DEBUG_SONAR
		fprintf(stderr,"SONAR Last index was %d\n", lastIndex);
		#endif

		if ( lastIndex > 0 )
		{
			// SITUATION WHERE THE LAST INDEX WAS LEFT TO TARGET
			if ( lastIndex <= (int)(numberOfSonars/4) )
			{
				//SITUATION WHERE ROBOT IS APPROACHING TARGET
				int li = 2*lastIndex;

				//FOR TO FIND BETWEEN 0 AND LASTINDEX SPAN
				for (posIndex=1; posIndex <= li; posIndex++ )
				{
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				//FOR TO FIND IN THE ALTERNATE UNTIL POSITIVE IS 180º OF THE CIRCLE SPAN
				for (negIndex=-1; posIndex <= halfSonars; posIndex++, negIndex--)
				{
					/*Analyze the left side sonar (positive angle)*/
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
						#endif
					}

					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				//FOR TO FIND IN THE REST OF THE CIRCLE
				for (; abs(negIndex) < halfSonars; negIndex--)
				{
					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				// IF WE CANNOT FIND ANY FREE SONAR, GO TO TARGET
				return targetAngle;
			}
			else
			{
				//SITUATION WHERE ROBOT IS MOVING AWAY FROM TARGET
				int li = lastIndex;

				//FOR TO FIND BETWEEN 0 AND LASTINDEX SPAN
				for (posIndex=1; posIndex <= li; posIndex++ )
				{
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				li = -(posIndex/2);
				//FOR TO FIND IN THE NEGATIVE SIDE AS MANY AS THE POSITIVE TO LASTINDEX
				for (negIndex=-1; negIndex >= li; negIndex--)
				{
					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				//FOR TO FIND IN THE ALTERNATE UNTIL POSITIVE IS 180º OF THE CIRCLE SPAN
				for (; posIndex <= halfSonars; posIndex++, negIndex--)
				{
					/*Analyze the left side sonar (positive angle)*/
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
						#endif
					}

					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				//FOR TO FIND IN THE REST OF THE CIRCLE
				for (; abs(negIndex) < halfSonars; negIndex--)
				{
					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				// IF WE CANNOT FIND ANY FREE SONAR, GO TO TARGET
				return targetAngle;
			}
		}
		else
		{
			// SITUATION WHERE THE LAST INDEX WAS RIGHT TO TARGET
			if ( abs(lastIndex) <= (int)(numberOfSonars/4) )
			{
				//SITUATION WHERE ROBOT IS APPROACHING TARGET
				int li = 2*lastIndex;

				//FOR TO FIND BETWEEN 0 AND LASTINDEX SPAN
				for (negIndex=-1; negIndex >= li; negIndex-- )
				{
					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}

				//FOR TO FIND IN THE ALTERNATE UNTIL NEGATIVE IS 180º OF THE CIRCLE SPAN
				for (posIndex=1; abs(negIndex) <= halfSonars; posIndex++, negIndex--)
				{
					/*Analyze the left side sonar (positive angle)*/
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
						#endif
					}

					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}

				//FOR TO FIND IN THE REST OF THE CIRCLE
				for (; posIndex < halfSonars; posIndex++)
				{
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",posIndex, currentSonarAngle.get_deg());
						#endif
					}
				}

				// IF WE CANNOT FIND ANY FREE SONAR, GO TO TARGET
				return targetAngle;
			}
			else
			{
				//SITUATION WHERE ROBOT IS MOVING AWAY FROM TARGET
				int li = lastIndex;

				//FOR TO FIND BETWEEN 0 AND LASTINDEX SPAN
				for (negIndex=-1; negIndex >= li; negIndex-- )
				{
					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				li = -(negIndex/2);
				//FOR TO FIND IN THE NEGATIVE SIDE AS MANY AS THE POSITIVE TO LASTINDEX
				for (posIndex=1; posIndex <= li; posIndex++)
				{
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",posIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				//FOR TO FIND IN THE ALTERNATE UNTIL POSITIVE IS 180º OF THE CIRCLE SPAN
				for (; abs(negIndex) <= halfSonars; posIndex++, negIndex--)
				{
					/*Analyze the left side sonar (positive angle)*/
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
						#endif
					}

					currentSonarAngle = targetAngle + negIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = negIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				//FOR TO FIND IN THE REST OF THE CIRCLE
				for (; posIndex < halfSonars; posIndex++)
				{
					currentSonarAngle = targetAngle + posIndex*angularOffset;
					if (isSonarFree(currentSonarAngle, obstacles))
					{
						#if DEBUG_SONAR
						gettimeofday( &deltaTime , NULL );
						fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
						#endif
						lastIndex = posIndex;
						testForDecel(obstacles, robotVel);
						return currentSonarAngle;
					}
					else
					{
						#if DEBUG_SONAR
						fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",posIndex, currentSonarAngle.get_deg());
						#endif
					}
				}
				
				// IF WE CANNOT FIND ANY FREE SONAR, GO TO TARGET
				return targetAngle;
			}
		}
	}	// CLOSE of IF the lastIndex was NOT 0
	else
	{
		/**If the last used index was 0, do the standard, start from the target (start from sensor 1 because the 0 is tested in the beggining). This "search" is made in pairs, i.e. 1 to the left, 1 to the right, 2 to the left, 2 to the right...*/
		#if DEBUG_SONAR
		fprintf(stderr,"SONAR Last index was 0\n");
		#endif

		for (posIndex=1, negIndex=-1; posIndex <= halfSonars; posIndex++, negIndex--)
		{
			/*Analyze the left side sonar (positive angle)*/
			currentSonarAngle = targetAngle + posIndex*angularOffset;
			if (isSonarFree(currentSonarAngle, obstacles))
			{
				#if DEBUG_SONAR
				gettimeofday( &deltaTime , NULL );
				fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", posIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
				#endif
				lastIndex = posIndex;
				testForDecel(obstacles, robotVel);
				return currentSonarAngle;
			}
			else
			{
				#if DEBUG_SONAR
				fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n", posIndex, currentSonarAngle.get_deg());
				#endif
			}

			/*Analyze the right side sonar (negative angle) only if we have an odd number of sonars. When even, the last cycle is the same sonar for both directions.*/
			if ( posIndex != negIndex )
			{
				currentSonarAngle = targetAngle + negIndex*angularOffset;
				if (isSonarFree(currentSonarAngle, obstacles))
				{
					#if DEBUG_SONAR
					gettimeofday( &deltaTime , NULL );
					fprintf(stderr,"SONAR Sonar %d with %fº was chosen. Took %ld ms\n", negIndex, currentSonarAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime);
					#endif
					lastIndex = negIndex;
					testForDecel(obstacles, robotVel);
					return currentSonarAngle;
				}
				else
				{
					#if DEBUG_SONAR
					fprintf(stderr,"SONAR Sonar %d is occupied: %fº\n",negIndex, currentSonarAngle.get_deg());
					#endif
				}
			}
		}	// END of for that goes from sensor 1 to half the sensors (if lastIndex was the target)
	}	// CLOSE of IF which verifies if the lastIndex was 0

	#if DEBUG_SONAR
	gettimeofday( &deltaTime , NULL );
	fprintf(stderr,"SONAR No sonar was chosen, returning target with %fº. Took %ld ms, %ld us\n",targetAngle.get_deg(), (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - startTime, (deltaTime.tv_sec*1000000 + deltaTime.tv_usec) - startTime*1000);
	#endif
	return targetAngle;
}


bool Sonar::isSonarFree(Angle sonarAngle, const vector<Vec>& obstacles, bool isTarget, double targetDist)
{
	double currentObstDist;
	int openingIndex;

	#if DEBUG_SONAR_OBST
	fprintf(stderr,"SONAR Testing sonar angle %fº for %d obstacles\n", sonarAngle.get_deg(), obstacles.size());
	#endif
	for ( unsigned int o=0; o<obstacles.size(); o++)
	{
		currentObstDist = obstacles.at(o).length() - (robotRad*bodyOversize);
		if(currentObstDist < 0)
			currentObstDist = 0;

		openingIndex = (int)(currentObstDist * 10);
		if (openingIndex >= numberOfSegments)
			openingIndex = numberOfSegments-1;
		
		#if DEBUG_SONAR_OBST
		fprintf(stderr,"SONAR openingIndex = %d\n", openingIndex);
		fprintf(stderr,"SONAR Testing obst %d: %fº to be between: %f <-> %f\n", o, obstacles.at(o).angle().get_deg(), (sonarAngle-opening.at(openingIndex)).get_deg(), (sonarAngle+opening.at(openingIndex)).get_deg() );
		#endif
		if ( ((obstacles.at(o).angle()).in_between( sonarAngle-opening.at(openingIndex), sonarAngle+opening.at(openingIndex) )) && 
			( !isTarget  || (isTarget && (obstacles.at(o).length() < targetDist)) ) )
//		if  ( (obstacles.at(o).angle()).in_between( sonarAngle-opening.at(openingIndex), sonarAngle+opening.at(openingIndex) ) )
		{
			#if DEBUG_SONAR_OBST
			fprintf(stderr,"SONAR obst %d is occupying this sonar\n", o);
			#endif
			return false;
		}
	}

	#if DEBUG_SONAR_OBST
	fprintf(stderr,"SONAR The sonar is free\n");
	#endif
	return true;
}


void Sonar::testForDecel(const vector<Vec>& obstacles, const Vec& linearVelocity)
{
	Angle aa = Angle();
	double anng, angPart;
	static double randomPart;	//this is static so the robot maitains the same random part while it needs to decelerate
	static double initialSpeed;
	int closestObstacleId = closestObstacleIndex(obstacles, linearVelocity);

	if (linearVelocity.length() < 0.6)  // Only do it  if speed is above 0.6m/s
	{
		decelerationFlag = false;
	}
	else if (closestObstacleId >= 0)       // Is tere an obstacle within 90º of my velocity?
	{
		// Get closer obstacle distance to me
		double dist = obstacles.at(closestObstacleId).length();

		if (dist < 0.5)       // If bellow 0,5m
		{
			if (decelerationFlag == false)     // If just got closer than 0.5m
			{
				decelerationFlag = true;

				// Save current robot speed for future use
				initialSpeed = linearVelocity.length();

				// Determine angle between robot linear movement and obstacle center
				aa = linearVelocity.angle() - obstacles.at(closestObstacleId).angle();
				anng = fabs(aa.get_deg_180());
//				aa = myRobots[rN].myObstacles.getObstacle(myRobots[rN].myObstacles.closestRobotIndex, 1).angle - myRobots[rN].myMotion.rSpeed.linearSpeed.angle;
//				anng = Math.Abs(aa.degSim);

				// Determine deceleration factor component based on "anng"
				angPart = anng / 450.0;

				// Determine random deceleration factor
				randomPart = (double)random()/(double)RAND_MAX*2.0 / 10.0;

				// Calculate deceleration factor as the sum of 0.5 with random and anng components
				topSpeed = (0.5 + randomPart + angPart) * linearVelocity.length();
				
				#if DEBUG_SONAR
				fprintf(stderr,"SONAR_DECEL angPart: %f, randPart: %f, linVel: %f, initTopSpeed: %f\n",angPart, randomPart,linearVelocity.length(), topSpeed);
				#endif
			}
			else
			{
				// Determine angle between robot linear movement and obstacle center
				aa = linearVelocity.angle() - obstacles.at(closestObstacleId).angle();
				anng = fabs(aa.get_deg_180());

				// Determine deceleration factor component based on "anng"
				angPart = anng / 450.0;

				// Calculate deceleration factor as the sum of 0.5 with random and anng components
				topSpeed = (0.5 + randomPart + angPart) * initialSpeed;
			}
		}
		else if (dist > 0.7)  // If above 0.7m turn off deceleration flag 
		{
			decelerationFlag = false;
		}
	}
	else
		decelerationFlag = false;
}


int Sonar::closestObstacleIndex(const vector<Vec>& obstacles, const Vec& linearVelocity)
{
	Angle velObstAng;
	double minDist=100;
	int closer=-1;

	for ( unsigned int o=0; o<obstacles.size(); o++)
	{
		velObstAng = linearVelocity.angle() - obstacles.at(o).angle();

		if ( fabs(velObstAng.get_deg_180()) <= 90 )
		{
			if ( obstacles.at(o).length() < minDist )
			{
				minDist = obstacles.at(o).length();
				closer = o;
			}
		}
	}
	return closer;
}
}
