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

#include "KalmanFilter.h"
#include "WorldStateDefs.h"
#include <cstdio>

namespace cambada {
namespace util {
using namespace geom;

KalmanFilter::KalmanFilter( double readingDeviation, struct timeval instant )
{
	linearRegression = LinearRegression(BALL_POSITION_BUFFER_SIZE);
	lastPosition	= Vec::zero_vector;
	lastVelocity	= Vec::zero_vector;
	lastTime		= instant.tv_sec*1000 + instant.tv_usec/1000;

	setNoise(readingDeviation);

	covarMatrix[0][0]	= 1.0;
	covarMatrix[0][1]	= 0.0;
	covarMatrix[1][0]	= 0.0;
	covarMatrix[1][1]	= 1.0;

	modelMatrix[0][0]	= 1.0;
	modelMatrix[0][1]	= 0.0;
	modelMatrix[1][0]	= 0.0;
	modelMatrix[1][1]	= 1.0;

	lastCycleVisible	= false;
	hardDeviationCount = 0;
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::setNoise( double readingDeviation )
{
	this->readingDeviation = readingDeviation;

	R = readingDeviation * readingDeviation;
	Q[0][0] = (readingDeviation/3) * (readingDeviation/3);
	Q[0][1] = 0.0;
	Q[1][0] = 0.0;
	Q[1][1] = (readingDeviation*2) * (readingDeviation*2);
}

void KalmanFilter::updateFilter( Vec readPosition, struct timeval instant )
{
	unsigned long instant_seconds = instant.tv_sec*1000 + instant.tv_usec/1000;

	//if the ball was not visible, take the read position as the initial aproach for the filter
	if ( !lastCycleVisible )
	{
		resetFilter( readPosition, instant );
		lastCycleVisible = true;
		// fprintf(stderr,"LastCycleVisible changed: %d\n",lastCycleVisible);
		return;
	}

	//calculate time variation between last and current cycle
	double deltaT;
	deltaT = (instant_seconds - lastTime)/1000.0;	//time in seconds

	// fprintf(stderr,"deltaT: %f\n",deltaT);

	//refresh the model matrix with the correct deltaT
	modelMatrix[0][1] = deltaT;

	//calculate the predicted position of the ball
	Vec positionPrediction = modelMatrix[0][0]*lastPosition + modelMatrix[0][1]*lastVelocity;

	//calculate the predicted velocity of the ball (with the base model is always the same)
	Vec velocityPrediction = modelMatrix[1][0]*lastPosition + modelMatrix[1][1]*lastVelocity;

	//calculate the predicted covariance matrix for the current sample F*P*F' + Q
	double FP[2][2];	//F*P
	FP[0][0] = modelMatrix[0][0]*covarMatrix[0][0] + modelMatrix[0][1]*covarMatrix[1][0];
	FP[0][1] = modelMatrix[0][0]*covarMatrix[0][1] + modelMatrix[0][1]*covarMatrix[1][1];
	FP[1][0] = modelMatrix[1][0]*covarMatrix[0][0] + modelMatrix[1][1]*covarMatrix[1][0];
	FP[1][1] = modelMatrix[1][0]*covarMatrix[0][1] + modelMatrix[1][1]*covarMatrix[1][1];

	double predictedP[2][2];	//(F*P)*F' + Q
	predictedP[0][0] = FP[0][0]*modelMatrix[0][0] + FP[0][1]*modelMatrix[0][1] + Q[0][0];
	predictedP[0][1] = FP[0][0]*modelMatrix[1][0] + FP[0][1]*modelMatrix[1][1] + Q[0][1];
	predictedP[1][0] = FP[1][0]*modelMatrix[0][0] + FP[1][1]*modelMatrix[0][1] + Q[1][0];
	predictedP[1][1] = FP[1][0]*modelMatrix[1][0] + FP[1][1]*modelMatrix[1][1] + Q[1][1];

	//calculate innovation covariation S, based on the predicted P; S = H*P*H' + R (being H = [1 0], S = P[0][0] + R)
	S = predictedP[0][0] + R;

	//calculate the optimal kalman gain for this cycle K = predictedP*H'*inv(s)
	kalmanGain[0] = predictedP[0][0] * 1/S;	//multiplication by H = [1 0]' is 1st column of tempP
	kalmanGain[1] = predictedP[1][0] * 1/S;

	//calculate the measurement innovation
	Vec residual = readPosition - positionPrediction;

	//based on the kalman gain (and therefore the predicted covariance for the current cycle), refresh the base covariance matrix for next cycle; P = (I-K*H)*P
	double tmp[2][2];	//I - K*H, K*H returns a [2][2] matrix with K in 1st column and zeros in 2nd
	tmp[0][0] = 1.0 - kalmanGain[0];
	tmp[0][1] = 0.0;	//I[0][1] 0.0 - K*H[0][1] 0.0
	tmp[1][0] = 0.0 - kalmanGain[1];
	tmp[1][1] = 1.0;	//I[0][1] 1.0 - K*H[0][1] 0.0

	//(I - K*H) * P
	covarMatrix[0][0] = tmp[0][0]*predictedP[0][0] + tmp[0][1]*predictedP[1][0];
	covarMatrix[0][1] = tmp[0][0]*predictedP[0][1] + tmp[0][1]*predictedP[1][1];
	covarMatrix[1][0] = tmp[1][0]*predictedP[0][0] + tmp[1][1]*predictedP[1][0];
	covarMatrix[1][1] = tmp[1][0]*predictedP[0][1] + tmp[1][1]*predictedP[1][1];

	//finaly calculate the filtered position to return (and some residual velocity refresh), also used as base to the next cycle prediction
	lastPosition	= positionPrediction + kalmanGain[0]*residual;
	lastVelocity	= velocityPrediction + kalmanGain[1]*residual;
	lastTime 		= instant_seconds;

	//protection so that speed doesn't go out of hand
	// if ( fabs(lastVel.x) > 10.0 || fabs(lastVel.y) > 10.0 )
	// 	resetFilter( readPosition );

	//test for hard deviation from predicted to measured positions
	if ( fabs(lastPosition.length() - readPosition.length()) > (readingDeviation + 0.15) )
		hardDeviationCount++;
	else
		hardDeviationCount = 0;

	if( hardDeviation() )
	{
		linearRegression.resetToNSamples( BALL_POSITION_RESET_SIZE );
		linearRegression.putNewValues( lastPosition*1000, instant);
		// myprintf("KALMAN_FILTER: BALL hardDeviation\n");
	}
	else
	{
		// if ( !world->me->ball.own )	//guarantee that velocity is not affected by ball source transitions
		// 	ballVel.resetToNSamples( 0 );
		linearRegression.putNewValues( lastPosition*1000, instant);
		//myprintf("KALMAN_FILTER: BALL buffer added %f,%f\n", (lastPosition*1000).x, (lastPosition*1000).y);
	}

	lastVelocity = linearRegression.getDeclivity();

	// fprintf(stderr,"KALMAN_FILTER: KalmanSpeed %f, %f\n", lastVel.x, lastVel.y);
}


void KalmanFilter::resetFilter( Vec initialPosition, struct timeval instant )
{
	cerr << "[FILTER] : reset 1" << endl;

	lastPosition	= initialPosition;
	lastVelocity	= Vec::zero_vector;

	// struct timeval tmpTime;
	// gettimeofday( &tmpTime , NULL );
	unsigned long instant_seconds = instant.tv_sec*1000 + instant.tv_usec/1000;

	if ( (instant_seconds - lastTime) > 1000 )
	{
		covarMatrix[0][0]	= 1.0;
		covarMatrix[0][1]	= 0.0;
		covarMatrix[1][0]	= 0.0;
		covarMatrix[1][1]	= 1.0;
	}

	lastTime = instant_seconds;
	hardDeviationCount = 0;
	linearRegression.resetToNSamples(0);
}


void KalmanFilter::setNotVisible()
{
	linearRegression.clearBuffs();
	lastCycleVisible = false;
}


bool KalmanFilter::hardDeviation()
{
	if ( hardDeviationCount < (int)(3*33/MOTION_TICK + 0.5) ) return false;
	hardDeviationCount = 0;
	return true;
}


Vec KalmanFilter::getVelocity(int omniCyclesNotVisible){
	return this->lastVelocity;
}

}/* namespace util */
}/* namespace cambada */



