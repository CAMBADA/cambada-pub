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

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "LinearRegression.h"
#include "Vec.h"
#include "Filter.h"
#include <sys/time.h>

namespace cambada {
namespace util {
using namespace geom;

/* KalmanFilter class*/
class KalmanFilter : public Filter
{
public:
	/*!Main constructor
	\param readingDeviation standard deviation of the measures*/
	KalmanFilter( double readingDeviation, struct timeval instant );

	/*!Class destructor*/
	~KalmanFilter();

	/*!Method to set the measure standard deviation to the desired value
	\param readingDeviation standard deviation of the measures*/
	void setNoise( double readingDeviation );

	/*!Method to apply the filter to a measured position
	\param readPosition absolute position of the read sample*/
	void updateFilter( Vec readPosition, struct timeval instant );

	/*!Method to restart the kalman filter internal variables
	\param initialPosition is initial aproximation to the filter*/
	void resetFilter( Vec initialPosition, struct timeval instant );

	/*!Method to set the ball as not visible, in the current cycle (used for reset control)*/
	void setNotVisible();

	/*!Tests for a hard deviation between the measures and the estimation, to indicate direction changes
	\return True if a hard deviation has occured*/
	bool hardDeviation();

	/*!Method that return last velocity estimated*/
	Vec getVelocity(int omniCyclesNotVisible=0);

private:
	LinearRegression linearRegression;	/*!<Object for using the linear regression to estimate the ball velocity.*/

	bool lastCycleVisible;				/*!<An indication wheter or not the ball was visible on the last cycle (for reset purposes when the ball has been unavailable).*/
	unsigned long lastTime;				/*!<The last time an update to was made to the filter state.*/
	int hardDeviationCount;				/*!<Counter for keeping the number of hard deviations found (for reset purposes).*/
	double readingDeviation;			/*!<The deviation of the measurements (vision sensor error, for sensor model).*/

	double R;							//R = readingDeviation^2
	double Q[2][2];						//Q = (readingDeviation/WHATEVER)^2 in main diagonal
	double S;							//covariance innovation
	double covarMatrix[2][2];			//P
	double modelMatrix[2][2];			//F
	double kalmanGain[2];				//K
};

}/* namespace util */
}/* namespace cambada */

#endif /* KALMANFILTER_H_ */
