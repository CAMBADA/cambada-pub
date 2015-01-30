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

#ifndef LINREGRESSION_H_
#define LINREGRESSION_H_

#include <deque>

namespace cambada
{

namespace util
{

class LinRegression
{
public:
	LinRegression();
	virtual ~LinRegression();
//	void addPair(Vec);
//	Vec getPair(int index);
//	unsigned int getNumPairs();
	void calculateParameters(std::deque<float> xSamples, std::deque<float> ySamples);
	float getSlope(void);
	float getOrigin(void);
	void reset();

private:
	float slope, origin;
};

}  // namespace util

}  // namespace cambada

#endif /* LINREGRESSION_H_ */
