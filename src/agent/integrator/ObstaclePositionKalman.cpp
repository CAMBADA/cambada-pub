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

#include "ObstaclePositionKalman.h"
#include <cstdio>

using namespace cambada::geom;

namespace cambada {
	
unsigned int ObstaclePositionKalman::instanceCount = 10;

ObstaclePositionKalman::ObstaclePositionKalman()
{
	init();
}


ObstaclePositionKalman::ObstaclePositionKalman( double readingDeviation )
{
	init();
	setNoise(readingDeviation);
}

void ObstaclePositionKalman::init()
{
	lastMeasure	= Vec::zero_vector;
	lastVel		= Vec::zero_vector;
	
	struct timeval tmpTime;
	gettimeofday( &tmpTime , NULL );
	
	lastTime = tmpTime.tv_sec*1000 + tmpTime.tv_usec/1000;

	covarMatrix[0][0] = 1.0;
	covarMatrix[0][1] = 0.0;
	covarMatrix[1][0] = 0.0;
	covarMatrix[1][1] = 1.0;
	
	modelMatrix[0][0] = 1.0;
	modelMatrix[0][1] = 0.0;
	modelMatrix[1][0] = 0.0;
	modelMatrix[1][1] = 1.0;
	
	Q[0][0] = 0.0;
	Q[0][1] = 0.0;
	Q[1][0] = 0.0;
	Q[1][1] = 0.0;
	
	firstExecution = true;
	obstacleID = 0;
	onlyPredictionCount = 0;
	cyclesAlive = 0;
}

ObstaclePositionKalman::~ObstaclePositionKalman()
{
}


void ObstaclePositionKalman::setNoise( double readingDeviation )
{
	this->readingDeviation = readingDeviation;

	R = readingDeviation * readingDeviation;
	Q[0][0] = (readingDeviation/2) * (readingDeviation/2);
	Q[0][1] = 0.0;
	Q[1][0] = 0.0;
	Q[1][1] = (readingDeviation*2) * (readingDeviation*2);
}


void ObstaclePositionKalman::update_PredictPhase(unsigned long instant)
{
	if (!firstExecution)		//Estimate prediction only when it is not the first time. If first time, only the time is set
	{
		//calculate time variation between last and current cycle
		double deltaT;
		deltaT = (instant - lastTime)/1000.0;	//time in seconds

	//	fprintf(stderr,"deltaT: %f\n",deltaT);
		
		//refresh the model matrix with the correct deltaT
		modelMatrix[0][1] = deltaT;

		//calculate the predicted position of the ball
		positionPrediction = modelMatrix[0][0]*lastMeasure + modelMatrix[0][1]*lastVel;
		//calculate the predicted velocity of the ball (with the base model is always the same)
		velocityPrediction = modelMatrix[1][0]*lastMeasure + modelMatrix[1][1]*lastVel;

		//consider the prediction as the state, because there can be the case that the observation phase does not exist and only the prediction is made
		lastMeasure = positionPrediction;
		lastVel = velocityPrediction;
		
//		fprintf(stderr,"modelMat:  %f, %f\n           %f, %f\n", modelMatrix[0][0], modelMatrix[0][1], modelMatrix[1][0], modelMatrix[1][1]);

		//calculate the predicted covariance matrix for the current sample F*P*F' + Q
		double FP[2][2];	//F*P
		FP[0][0] = modelMatrix[0][0]*covarMatrix[0][0] + modelMatrix[0][1]*covarMatrix[1][0];
		FP[0][1] = modelMatrix[0][0]*covarMatrix[0][1] + modelMatrix[0][1]*covarMatrix[1][1];
		FP[1][0] = modelMatrix[1][0]*covarMatrix[0][0] + modelMatrix[1][1]*covarMatrix[1][0];
		FP[1][1] = modelMatrix[1][0]*covarMatrix[0][1] + modelMatrix[1][1]*covarMatrix[1][1];
		
//		fprintf(stderr,"FPmatrix:  %f, %f\n           %f, %f\n", FP[0][0], FP[0][1], FP[1][0], FP[1][1]);
//		fprintf(stderr,"Qmatrix:  %f, %f\n          %f, %f  -  readingDev: %f\n", Q[0][0], Q[0][1], Q[1][0], Q[1][1],readingDeviation);
		
		//(F*P)*F' + Q
		predictedP[0][0] = FP[0][0]*modelMatrix[0][0] + FP[0][1]*modelMatrix[0][1] + Q[0][0];
		predictedP[0][1] = FP[0][0]*modelMatrix[1][0] + FP[0][1]*modelMatrix[1][1] + Q[0][1];
		predictedP[1][0] = FP[1][0]*modelMatrix[0][0] + FP[1][1]*modelMatrix[0][1] + Q[1][0];
		predictedP[1][1] = FP[1][0]*modelMatrix[1][0] + FP[1][1]*modelMatrix[1][1] + Q[1][1];
		
//		fprintf(stderr,"predictedP:  %f, %f\n             %f, %f\n", predictedP[0][0], predictedP[0][1], predictedP[1][0], predictedP[1][1]);
		
		onlyPredictionCount++;
	}

	//Increment the lifetime of the instance
	cyclesAlive++;

	//set the time instant
	lastTime = instant;
}


void ObstaclePositionKalman::update_ObservationPhase( Vec readPosition )
{
	if (firstExecution)
	{
		lastMeasure = readPosition;
		firstExecution = false;
		return;
	}
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
	
	//protection so that speed doesn't go out of hand
//	if ( fabs(lastVel.x) > 10.0 || fabs(lastVel.y) > 10.0 )
//		resetFilter( readPosition );

	//The observation phase was executed this cycle, reset the only prediction count
	onlyPredictionCount = 0;
}


void ObstaclePositionKalman::resetFilter( Vec setAsLast, unsigned long instant )
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

	onlyPredictionCount = 0;
}

int ObstaclePositionKalman::getOnlyPredictionCount()
{
	return onlyPredictionCount;
}

double ObstaclePositionKalman::getPositionVariance()
{
	return covarMatrix[0][0];
}

double ObstaclePositionKalman::getVelocityVariance()
{
	return covarMatrix[1][1];
}

Vec ObstaclePositionKalman::getFilterPosition()
{
	return lastMeasure;
}

Vec ObstaclePositionKalman::getFilterVelocity()
{
	return lastVel;
}

void ObstaclePositionKalman::setID()
{
	unsigned char charID = instanceCount % 255;
	if ( charID < 10 )
		instanceCount += 10;

	obstacleID = instanceCount;
	instanceCount++;
}

unsigned int ObstaclePositionKalman::getID()
{
	return obstacleID;
}

vector<double> ObstaclePositionKalman::getCovarMatrix()
{
	vector<double> result;

	result.push_back(covarMatrix[0][0]);
	result.push_back(covarMatrix[0][1]);
	result.push_back(covarMatrix[1][0]);
	result.push_back(covarMatrix[1][1]);

	return result;
}

vector<Vec> ObstaclePositionKalman::getStateVector()
{
	vector<Vec> result;

	result.push_back(lastMeasure);
	result.push_back(lastVel);

	return result;
}

}//Close namespace
