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

#include "LinRegression.h"
#include <cassert>

cambada::util::LinRegression::LinRegression()
	: slope(0), origin(0)
{
}

cambada::util::LinRegression::~LinRegression()
{
}

//void cambada::util::LinRegression::addPair(Vec vec)
//{
//	xSamples.addValue(vec.x);
//	ySamples.addValue(vec.y);
//}
//
//Vec cambada::util::LinRegression::getPair(int index)
//{
//	assert(index < getNumPairs());
//	return Vec(xSamples.getSample(index), ySamples.getSample(index));
//}
//
//unsigned int cambada::util::LinRegression::getNumPairs()
//{
//	return xSamples.getNumSamples();
//}

void cambada::util::LinRegression::calculateParameters(std::deque<float> xSamples, std::deque<float> ySamples)
{
	assert(xSamples.size() == ySamples.size());
	if(xSamples.size() < 2)
	{
		slope = 0.0f;
		origin = 0.0f;
	}
	else
	{
		float M = xSamples.size();
		float sumY = 0.0f;
		float sumX = 0.0f;
		float sumXY = 0.0f;
		float sumXX = 0.0f;

		for (int i = 0; i < M; ++i)
		{
			float x = xSamples[i];
			float y = ySamples[i];
			sumX += x;
			sumY += y;
			sumXY += x*y;
			sumXX += x*x;
		}

		//TODO zero if denominator very small
		slope = (M*sumXY - sumX*sumY)/(M*sumXX - sumX*sumX);
		origin = (1/M) * sumY - (slope/M) * sumX;
	}
}

float cambada::util::LinRegression::getSlope(void)
{
	return slope;
}

float cambada::util::LinRegression::getOrigin(void)
{
	return origin;
}

void cambada::util::LinRegression::reset()
{
	origin = 0.0f;
	slope = 0.0f;
}


