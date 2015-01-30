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

#include "EgoMotionEstimator.h"
#include <stdio.h>
#include "Clock.h"
#include "Angle.h"

using namespace cambada::geom;

namespace cambada {
namespace util {

EgoMotionEstimator::EgoMotionEstimator(int nSamples)
	: xLinReg(), yLinReg(), thetaLinReg(), xWindow(nSamples),
	  yWindow(nSamples), thetaWindow(nSamples), timeWindow(nSamples), timer(),
	  linVel(Vec()), angVel(0.0f)
{
}

EgoMotionEstimator::~EgoMotionEstimator()
{
}

void EgoMotionEstimator::update()
{
	//TODO adaptive window resize policy

	thetaLinReg.calculateParameters(timeWindow.getSamples(), thetaWindow.getSamples());
	if(thetaLinReg.getSlope() < 10e-3) //straight line
	{
		angVel = 0.0f;
		xLinReg.calculateParameters(timeWindow.getSamples(), xWindow.getSamples());
		yLinReg.calculateParameters(timeWindow.getSamples(), yWindow.getSamples());
		linVel.x = xLinReg.getSlope();
		linVel.y = yLinReg.getSlope();
	}
	else //arc movement
	{
		angVel = thetaLinReg.getSlope();

		float n = (float) timeWindow.getNumSamples();
		float sumX = 0.0f;
		float sumY = 0.0f;
		float sumS = 0.0f;
		float sumC = 0.0f;
		float sumSS = 0.0f;
		float sumCC = 0.0f;
		float sumSX = 0.0f;
		float sumCX = 0.0f;
		float sumSY = 0.0f;
		float sumCY = 0.0f;
		float timeRef = timeWindow.getSample(n-1);

		for (unsigned int i = 0; i < timeWindow.getNumSamples(); ++i)
		{
			float t = timeWindow.getSample(i) - timeRef;
			float x = xWindow.getSample(i);
			float y = yWindow.getSample(i);
			sumX += x;
			sumY += y;
			float s = sin(angVel*t);
			float c = (cos(angVel*t)-1);
			sumS += s;
			sumC += c;
			sumSS += s*s;
			sumCC += c*c;
			sumSX += s*x;
			sumCX += c*x;
			sumSY += s*y;
			sumCY += c*y;

		}
		sumS /= angVel;
		sumC /= angVel;
		sumSS /= (angVel*angVel);
		sumCC /= (angVel*angVel);
		sumSX /= angVel;
		sumCX /= angVel;
		sumSY /= angVel;
		sumCY /= angVel;


		float d = n*(sumSS+sumCC) - sumS*sumS - sumC*sumC;
		if(d < 10e-2)
		{
			linVel.x = 0.0f;
			linVel.y = 0.0f;
		}
		else
		{
			linVel.x = (-sumS*sumX + sumC*sumY + n*(sumSX -sumCY))/d;
			linVel.y = (-sumC*sumX - sumS*sumY + n*(sumCX + sumSY))/d;
		}

	}
}

void EgoMotionEstimator::addValue(Vec pos, float theta)
{
	this->addValue(pos.x,pos.y,theta);
}

void EgoMotionEstimator::addValue(float x, float y, float theta)
{
	xWindow.addValue(x);
	yWindow.addValue(y);

	//deal with angle discontinuity by angle unrolling
	if(thetaWindow.getNumSamples() > 1)
	{
		float lastTheta = thetaWindow.getSample(thetaWindow.getNumSamples()-1);
		float diffTheta;
		while(fabs(diffTheta = (theta - lastTheta)) > M_PI)
		{
			//Mapping angle to real value
			float sign = (diffTheta >= 0) ? 1.0f : -1.0f;
			theta -= sign * 2*M_PI;
		}
	}
	thetaWindow.addValue(theta);

	timeWindow.addValue(timer.elapsed()/1000.0f); //in seconds

	//fprintf(stderr,"EGOMOTION_DATA %f, %f, %f, %f, %d\n", timeWindow.getSample(timeWindow.getNumSamples()-1), xWindow.getSample(timeWindow.getNumSamples()-1), yWindow.getSample(timeWindow.getNumSamples()-1), thetaWindow.getSample(timeWindow.getNumSamples()-1), timeWindow.getNumSamples());

}

Vec EgoMotionEstimator::getLinearVelocity(void)
{
	return linVel;
}

float EgoMotionEstimator::getAngularVelocity(void)
{
	return angVel;
}

void EgoMotionEstimator::reset()
{
	xWindow.clear();
	yWindow.clear();
	thetaWindow.clear();
	timeWindow.clear();
	xLinReg.reset();
	yLinReg.reset();
	thetaLinReg.reset();
	timer.restart();
}

}}

