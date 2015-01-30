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

#ifndef _KALMAN_
#define _KALMAN_

#include "Vec.h"
#include <sys/time.h>
#include <vector>

using namespace std;

namespace cambada {
	
/*! Filters a position based on a kalman filter. Built specifically for dynamic objects position, the model is hardcoded to minimize running time. It assumes the usual sensor setup, track object through a (Position, Velocity) state, where only Position is observable.
\brief Kalman filter implementation*/
class ObstaclePositionKalman
{
public:
	/*!Default constructor.*/
	ObstaclePositionKalman();
	
	/*!Main constructor.
	\param readingDeviation standard deviation of the measures*/
	ObstaclePositionKalman( double readingDeviation );
	
	/*!Class destructor.*/
	~ObstaclePositionKalman();
	
	/*!Method to set the measure standard deviation to the desired value.
	\param readingDeviation standard deviation of the measures*/
	void setNoise( double readingDeviation );
	
	/*!Method to execute the prediction phase of the filter state (sets the time of the current cycle).
	 \param instant time instant of the current cycle <b>in miliseconds</b>*/
	void update_PredictPhase(unsigned long instant);
	
	/*!Method to execute the observation phase of the filter state (sets the observation of the current cycle)
	\param readPosition absolute position of the read sample*/
	void update_ObservationPhase( geom::Vec readPosition );
	
	/*!Method to restart the kalman filter internal variables.
	\param setAsLast initial aproximation to the filter*/
	void resetFilter( geom::Vec setAsLast, unsigned long instant );

	/*!Method to get the count of cycles where only the prediction phase was performed.
	 * <b>Will return 1 between the calls to prediction phase and observation phase.</b>
	\return The number of cycles with only prediction*/
	int getOnlyPredictionCount();
	
	/*!Method to set the ID of the obstacle for this instance.*/
	void setID();
	
	/*!Method to get the ID of the current obstacle.
	 \return The value of ObstacleID*/
	unsigned int getID();
	
	/*!Method to get the value of the position covariance of the current filter state.
	 \return Current state position covariance.*/
	double getPositionVariance();
	
	/*!Method to get the value of the velocity covariance of the current filter state.
	 \return Current state velocity covariance.*/
	double getVelocityVariance();
	
	/*!Method to get the current position estimation.
	\return The current position estimation.*/
	geom::Vec getFilterPosition();
	
	/*!Method to get the current velocity estimation.
	\return The current velocity estimation.*/
	geom::Vec getFilterVelocity();
	
	/*!\todo*/
	vector<double> getCovarMatrix();
	
	/*!\todo*/
	vector<geom::Vec> getStateVector();

private:
	geom::Vec lastMeasure;					/*!<Last position estimation of the filter (updated by \link filterPosition \endlink).*/
	geom::Vec lastVel;						/*!<Last velocity estimation of the filter (updated by \link filterPosition \endlink).*/

	bool firstExecution;				/*!<An indication of the first time the filter is run, so the initial state can be set without filter advancement.*/
	unsigned long lastTime;				/*!<The last timestamp an update was made to the filter state.*/
	
	unsigned char obstacleID;			/*!<The ID of the obstacle being tracked in this instance.*/
	static unsigned int instanceCount;	/*!<Static counter to use as IDs. Always incremental, even when a given instance is deleted.*/

	int onlyPredictionCount;			/*!<Counter for keeping the number of cycles where only prediction occured.*/
	unsigned int cyclesAlive;			/*!<Counter of the lifetime of the instance, in cycles.*/

	double readingDeviation;			/*!<The deviation of the measurements (vision sensor error, for sensor model).*/

	double R;				//R = readingDeviation^2
	double Q[2][2];			//Q = (readingDeviation/WHATEVER)^2 in main diagonal

	double S;					//covariance innovation
	double covarMatrix[2][2];	//P
	double modelMatrix[2][2];	//F
	double kalmanGain[2];		//K
	
	geom::Vec positionPrediction;				/*!<\todo*/
	geom::Vec velocityPrediction;				/*!<\todo*/
	double predictedP[2][2];			/*!<F*P*F' + Q*/
	
	/*!Method to initialize the internal attributes.*/
	void init();
};

}//Close namespace

#endif
