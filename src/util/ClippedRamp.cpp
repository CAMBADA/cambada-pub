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

#include "ClippedRamp.h"

namespace cambada {
namespace util {

ClippedRamp::ClippedRamp() {
	calcInternal(0,0,1,1);
	clipHighY = 1.0;
	clipLowY = 0.0;
}

ClippedRamp::ClippedRamp(float x1, float y1, float x2, float y2, float clipLowY, float clipHighY){
	this->clipLowY = clipLowY;
	this->clipHighY = clipHighY;
	calcInternal(x1,y1, x2,y2);
}

ClippedRamp::~ClippedRamp() {
	// TODO Auto-generated destructor stub
}

void ClippedRamp::calcInternal(float x1, float y1, float x2, float y2)
{
	m = (y2-y1) / (x2-x1);
	b = y1 - m*x1;
}

float ClippedRamp::getValue(float x)
{
	float ret = m*x + b;
	ret = (ret > clipHighY ? clipHighY : ret);
	ret = (ret < clipLowY ? clipLowY : ret);
	return ret;
}

}
} /* namespace cambada */
