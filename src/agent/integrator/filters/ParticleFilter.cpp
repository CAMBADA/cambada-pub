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

#include "ParticleFilter.h"

#define sqrt2pi 2.506628274631000

using namespace cambada;
using namespace cambada::geom;

namespace cambada {
namespace util {
using namespace geom;

ParticleFilter::ParticleFilter( double readingDeviation, int squareBase, struct timeval instant )
{
	lastTime = instant.tv_sec*1000 + instant.tv_usec/1000;		//initialize the last time instant as the creation time
	lastPosition = Vec::zero_vector;
	lastMeasure = Vec::zero_vector;

	setNoise(readingDeviation);		//set an initial noise
	setMParticles(squareBase);		//set the number of particles to use (the square of "squareBase"

	lastCycleVisible = false;

	hardDeviationCount = 0;

	createInitialSet();
}

ParticleFilter::~ParticleFilter()
{
}

void ParticleFilter::setNoise( double readingDeviation )
{
	this->readingDeviation = readingDeviation;
	velocityDeviation = readingDeviation;
}

void ParticleFilter::updateFilter( Vec readPosition, struct timeval instant )
{
	// readPosition = Vec(-1000.0,-1000.0); // default

	unsigned int m;
	double errorPos, errorVel, weightPos, weightVel, currentWeight=0.0, lastWeight=0.0, maxWeight=0.0;
	Vec measuredVelocity, posWsum=Vec::zero_vector, velWsum=Vec::zero_vector;
	vector<double> weights;
	weights.assign(M,M);			//create a weights vector with all elements with value 'M'
	bool onlyPrediction=false;

	Vec heavierPos, heavierVel;
	int zeroCount = 0;
	unsigned long instant_seconds = instant.tv_sec*1000 + instant.tv_usec/1000;

	if (readPosition == Vec(-1000.0,-1000.0))
	{
//		fprintf(stderr,"PARTICLE Only prediction\n");
		onlyPrediction=true;
	}

	bool veryLargeJump = ((readPosition - lastPosition).length() > (1.5));
	fprintf(stderr,"PARTICLE read: %f %f - last: %f %f, dist: %f - %d\n", readPosition.x, readPosition.y, lastPosition.x, lastPosition.y, (readPosition - lastPosition).length(), veryLargeJump);

	if ( !lastCycleVisible || (veryLargeJump && !onlyPrediction) )
	{
		fprintf(stderr,"PARTICLE RESET\n");
		resetFilter(readPosition, instant);
		lastCycleVisible = true;
	}

	 vector<Vec> tempPosParticles = positionParticles;
	 vector<Vec> tempVelParticles = velocityParticles;

	//calculate time variation between last and current cycle
	double deltaT;
	deltaT = (instant_seconds - lastTime)/1000.0;	//time in seconds
//	fprintf(stderr,"PARTICLE DeltaT: %f\n",deltaT);

//	fprintf(stderr,"CAPTURE %f %f %f %d %d\n", readPosition.x, readPosition.y, readingDeviation, -1, -1);
//	for (unsigned int a=0; a < tempPosParticles.size(); a++)
//		printf("PARTICLE X: %f, Y: %f\n",tempPosParticles.at(a).x,tempPosParticles.at(a).y);

	if (!onlyPrediction)
	{
		measuredVelocity = (readPosition-lastMeasure)/deltaT;		//estimate a velocity measure based on last cycle visible movement
		maxWeight=particlesWeight.at(0);
		for (unsigned int a=1; a<particlesWeight.size(); a++)
		{
			if (particlesWeight.at(a) > maxWeight)
			{
				maxWeight = particlesWeight.at(a);
			}
		}
	}

	for ( m=0; m<tempPosParticles.size(); m++ )
	{
		//add randomness to the particle velocity
		double factor=0.0;
		if ( particlesWeight.at(m) <= 0.5*maxWeight )
		{
//			fprintf(stderr,"PARTICLE 0.3\n");
			factor = 0.3;
		}
// 		else if ( particlesWeight.at(m) < 0.8*maxWeight )
// 		{
// 			fprintf(stderr,"PARTICLE 0.3\n");
// 			factor = 0.3;
// 		}
		else if ( particlesWeight.at(m) < 0.95*maxWeight )
		{
//			fprintf(stderr,"PARTICLE 0.1\n");
			factor = 0.1;
		}
// 		else
// 		{
// 			fprintf(stderr,"PARTICLE 0.0\n");
// 			factor = 0.0;
// 		}

		double xx = factor*r.randNorm();	//generate gaussian distributed random number with "factor" as standard deviation
		double yy = factor*r.randNorm();	//generate gaussian distributed random number with "factor" as standard deviation
//		double xx = factor*(-1.0 + rand() * (1.0 - -1.0) / RAND_MAX);		//generate uniform distributed random number between -"factor" and "factor"
//		double yy = factor*(-1.0 + rand() * (1.0 - -1.0) / RAND_MAX);		//generate uniform distributed random number between -"factor" and "factor"

		tempVelParticles.at(m).x += xx;
		tempVelParticles.at(m).y += yy;

		//calculate the predicted position of the particle
		tempPosParticles.at(m) = tempPosParticles.at(m) + deltaT*tempVelParticles.at(m);

		//velocity of the particles is not updated, since we use linear uniform movement model
//		fprintf(stderr,"PARTICLE TEMP X: %f, Y: %f --- vel: %f, %f\n",tempPosParticles.at(m).x, tempPosParticles.at(m).y, tempVelParticles.at(m).x, tempVelParticles.at(m).y);

		if (!onlyPrediction)
		{
			//estimate the residuals of the measured position and the measured velocity
			errorPos = (tempPosParticles.at(m)-readPosition).length();	//position residual
			errorVel = (tempVelParticles.at(m)-measuredVelocity).length();		//velocity residual

			//estimate the particle weight
			weightPos = (1 / (readingDeviation*sqrt2pi))  * exp( -(errorPos*errorPos) / (2*(readingDeviation*readingDeviation)));		//position weight of the current particle (normal distribution with "readingDeviation" as standard deviation)
			weightVel = (1 / (velocityDeviation*sqrt2pi))  * exp( -(errorVel*errorVel) / (2*(velocityDeviation*velocityDeviation)));		//velocity weight of the current particle (normal distribution with "velocityDeviation" as standard deviation)
			currentWeight = weightPos+weightPos*weightVel;		//estimate the total weight of the particle

//			fprintf(stderr,"PARTICLE Current weight: %f, rDev: %f, vDev: %f, errPos: %f, errVel: %f, sqrt2pi: %f\n",currentWeight, readingDeviation, velocityDeviation, errorPos, errorVel, sqrt2pi);
//			fprintf(stderr,"PARTICLE 1denom: %f - expFactor: %f - expResult: %f\n", (readingDeviation*sqrt2pi), -(errorPos*errorPos) / (2*(readingDeviation*readingDeviation)), exp( -(errorPos*errorPos) / (2*(readingDeviation*readingDeviation))));

			posWsum += tempPosParticles.at(m)*currentWeight;	//accumulate the position for weighted mean calculation
			velWsum += tempVelParticles.at(m)*currentWeight;	//accumulate the velocity for weighted mean calculation

			particlesWeight.at(m) = currentWeight;			//update the individual weight of the current particle
			weights.at(m) = lastWeight + currentWeight;		//update the cumulative weight of the current particle
			lastWeight = lastWeight + currentWeight;		//keep the cumulative weight for calculation of the next particle cumulative weight


			//FIXME
			if (currentWeight == 0.0)
				zeroCount++;
//			fprintf(stderr,"PARTICLE cumulative weight: %f --- posSum: %f,%f - velSum: %f,%f\n",weights.at(m), posWsum.x, posWsum.y, velWsum.x, velWsum.y);
		}
		else
		{
			posWsum += tempPosParticles.at(m);
			velWsum += tempVelParticles.at(m);
//			fprintf(stderr,"PARTICLE only prediction cumulative weight: %f --- posSum: %f,%f - velSum: %f,%f\n",weights.at(m), posWsum.x, posWsum.y, velWsum.x, velWsum.y);
		}
// 		%ADAPTIVE VELOCITY
 		if (currentWeight > maxWeight)
		{
 			maxWeight=currentWeight;
			//FIXME
 			heavierPos = tempPosParticles.at(m);
 			heavierVel = tempVelParticles.at(m);
		}

//		fprintf(stderr,"CAPTURE %f %f %f %f %512.512f\n",tempPosParticles.at(m).x, tempPosParticles.at(m).y, tempVelParticles.at(m).x, tempVelParticles.at(m).y, currentWeight);
	}

	lastPosition.x = posWsum.x / weights.back();
	lastPosition.y = posWsum.y / weights.back();
	lastVelocity.x = velWsum.x / weights.back();
	lastVelocity.y = velWsum.y / weights.back();

	fprintf(stderr,"PARTICLE LastPos: %f,%f, lastVel: %f,%f \n",lastPosition.x, lastPosition.y, lastVelocity.x, lastVelocity.y);//, weights.back(),maxWeight, heavierPos.x, heavierPos.y, heavierVel.x, heavierVel.y);    --- weight: %f, maxWeight: %32.30f Max pos: %f,%f, vel: %f,%f

	double p;
	unsigned int endBase,initVal,endVal,middle,index;

	endBase = weights.size()-1;
	for ( m=0; m<positionParticles.size(); m++ )
	{
//draw particles from the temporary with a probability equivalent to their weight (higher weight, higher probability to be chosen)
		p=(random() * (1.0 / RAND_MAX))*weights.back();		//rand on [0, 1]
//		fprintf(stderr,"Choosing %d th paticle for p %f\n",m,p);
		initVal = 0;
		endVal = endBase;

		while (true)
		{
			if ( (endVal-initVal) == 1 )
			{
				if ( p <= weights.at(initVal) )
				{
					index = initVal;
				}
				else
				{
					index = endVal;
				}
				break;
			}

			middle = floor((endVal+initVal)/2.0);

			if ( (p >= weights.at(initVal)) && (p < weights.at(middle)) )
			{
				endVal = middle;
			}
			else
			{
				initVal = middle;
			}
		}

//		fprintf(stderr,"PARTICLE p: %f, index: %d\n",p,index);

		positionParticles.at(m)=tempPosParticles.at(index);
		velocityParticles.at(m)=tempVelParticles.at(index);
	}

	lastTime = instant_seconds;

	//TODO Hard deviation detection was for velocity reset. Do I need it here??
	if ( fabs(lastPosition.length() - readPosition.length()) > (readingDeviation + 0.15) )
		hardDeviationCount++;
	else
		hardDeviationCount = 0;

	//FIXME
	fprintf(stderr,"PARTICLE zeroWeights: %d\n",zeroCount);
	fprintf(stderr,"PARTICLE count: %d\n\n",hardDeviationCount++);
}


void ParticleFilter::resetFilter( Vec initialPosition, struct timeval instant )
{
	lastTime = instant.tv_sec*1000 + instant.tv_usec/1000;
	lastPosition = Vec::zero_vector;
	lastMeasure = Vec::zero_vector;

	createVisualSet( initialPosition );
}

void ParticleFilter::setNotVisible()
{
	lastCycleVisible = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////// Specific Method's
void ParticleFilter::createInitialSet()
{
	double xIncrement, yIncrement;
	int VISIBLE_INTERVAL_X_MIN = -7;
	int VISIBLE_INTERVAL_X_MAX = 7;
	int VISIBLE_INTERVAL_Y_MIN = -10;
	int VISIBLE_INTERVAL_Y_MAX = 10;

	positionParticles.clear();
	velocityParticles.clear();
	particlesWeight.clear();

	xIncrement = (VISIBLE_INTERVAL_X_MAX - VISIBLE_INTERVAL_X_MIN) / sqrt((double)M);
	yIncrement = (VISIBLE_INTERVAL_Y_MAX - VISIBLE_INTERVAL_Y_MIN) / sqrt((double)M);

	//create the initial set of particles, equally spaced on a grid over the field (oficial dimensions), and with initial velocity 0
	for (double y = VISIBLE_INTERVAL_Y_MIN + yIncrement/2; y <= VISIBLE_INTERVAL_Y_MAX; y += yIncrement)
	{
		for (double x = VISIBLE_INTERVAL_X_MIN + xIncrement/2; x <= VISIBLE_INTERVAL_X_MAX; x += xIncrement)
		{
			positionParticles.push_back(Vec(x,y));
			velocityParticles.push_back(Vec(0.0,0.0));
			particlesWeight.push_back(-1.0);
		}
	}
}

void ParticleFilter::createVisualSet( Vec initialPosition )
{
	double spreadFactorX, spreadFactorY;

	positionParticles.clear();
	velocityParticles.clear();
	particlesWeight.clear();

	for (unsigned int m=0; m<M; m++)
	{
		spreadFactorX = 2*readingDeviation*r.randNorm();
		spreadFactorY = 2*readingDeviation*r.randNorm();

		Vec particle = Vec(initialPosition.x+spreadFactorX, initialPosition.y+spreadFactorY);

		positionParticles.push_back(particle);
		velocityParticles.push_back(Vec(0.0,0.0));
		particlesWeight.push_back(-1.0);
	}
}

void ParticleFilter::setMParticles( int squareBase )
{
	this->M = squareBase*squareBase;
}

vector<Vec> ParticleFilter::getPositionParticles()
{
	return positionParticles;
}

vector<Vec> ParticleFilter::getVelocityParticles()
{
	return velocityParticles;
}

}/* namespace util */
}/* namespace cambada */
