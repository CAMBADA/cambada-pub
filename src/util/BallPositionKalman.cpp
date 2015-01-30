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

#include "BallPositionKalman.h"
#include <cstdio>

using namespace cambada;
using namespace cambada::geom;

namespace cambada {

namespace util {

int BallPositionKalman::hardDeviationCount = 0;

BallPositionKalman::BallPositionKalman()
{}


BallPositionKalman::BallPositionKalman( double readingDeviation )
{
	lastMeasure	= Vec::zero_vector;
	lastVel		= Vec::zero_vector;
	
	struct timeval tmpTime;
	gettimeofday( &tmpTime , NULL );
	
	lastTime = tmpTime.tv_sec*1000 + tmpTime.tv_usec/1000;
	
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
	
//	fprintf(stderr,"Constructor called\n");
}


BallPositionKalman::~BallPositionKalman()
{}


void BallPositionKalman::setNoise( double readingDeviation )
{
	this->readingDeviation = readingDeviation;

	R = readingDeviation * readingDeviation;
	Q[0][0] = (readingDeviation/3) * (readingDeviation/3);
	Q[0][1] = 0.0;
	Q[1][0] = 0.0;
	Q[1][1] = (readingDeviation*2) * (readingDeviation*2);
}


Vec BallPositionKalman::filterPosition( Vec readPosition, unsigned long instant )
{
//	fprintf(stderr,"LastCycleVisible: %d\n",lastCycleVisible);
	//if the ball was not visible, take the read position as the initial aproach for the filter
	if ( !lastCycleVisible )
	{
		resetFilter( readPosition, instant );
		lastCycleVisible = true;
//		fprintf(stderr,"LastCycleVisible changed: %d\n",lastCycleVisible);
		return readPosition;
	}
	
	//calculate time variation between last and current cycle
	double deltaT;
	deltaT = (instant - lastTime)/1000.0;	//time in seconds

//	fprintf(stderr,"deltaT: %f\n",deltaT);
	
	//refresh the model matrix with the correct deltaT
	modelMatrix[0][1] = deltaT;
	
	//calculate the predicted position of the ball
	Vec positionPrediction = modelMatrix[0][0]*lastMeasure + modelMatrix[0][1]*lastVel;
	
	//calculate the predicted velocity of the ball (with the base model is always the same)
	Vec velocityPrediction = modelMatrix[1][0]*lastMeasure + modelMatrix[1][1]*lastVel;
	
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
	lastMeasure	= positionPrediction + kalmanGain[0]*residual;
	lastVel		= velocityPrediction + kalmanGain[1]*residual;

	lastTime = instant;
	
	//protection so that speed doesn't go out of hand
//	if ( fabs(lastVel.x) > 10.0 || fabs(lastVel.y) > 10.0 )
//		resetFilter( readPosition );
	
	//test for hard deviation from predicted to measured positions
	if ( fabs(lastMeasure.length() - readPosition.length()) > (readingDeviation + 0.15) )
		hardDeviationCount++;
	else
		hardDeviationCount = 0;

//	fprintf(stderr,"INTEGRATOR: KalmanSpeed %f, %f\n", lastVel.x, lastVel.y);
	return lastMeasure;
}


void BallPositionKalman::resetFilter( Vec setAsLast, unsigned long instant )
{
	lastMeasure	= setAsLast;
	lastVel		= Vec::zero_vector;
	
// 	struct timeval tmpTime;
// 	gettimeofday( &tmpTime , NULL );
// 	unsigned long actualTime = tmpTime.tv_sec*1000 + tmpTime.tv_usec/1000;
	unsigned long actualTime = instant;
	
	if ( (actualTime - lastTime) > 1000 )
	{
		covarMatrix[0][0]	= 1.0;
		covarMatrix[0][1]	= 0.0;
		covarMatrix[1][0]	= 0.0;
		covarMatrix[1][1]	= 1.0;
	}

	lastTime = actualTime;
	
//	fprintf(stderr,"INTEGRATOR RESET KALMAN FILTER CHAMADO BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB\n");

	hardDeviationCount = 0;
}


void BallPositionKalman::setNotVisible()
{
	lastCycleVisible = false;
}


bool BallPositionKalman::hardDeviation()
{
	if ( hardDeviationCount >= 3 )
	{
		hardDeviationCount = 0;
		return true;
	}
	
	return false;
}

}}
