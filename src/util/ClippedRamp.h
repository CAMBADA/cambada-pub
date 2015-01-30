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

#ifndef CLIPPEDRAMP_H_
#define CLIPPEDRAMP_H_

namespace cambada {
namespace util {

/**
 * Creates a clipped ramp profile, given 2 points of the line and maximum and minimum clipping values
 */
class ClippedRamp {
public:
	ClippedRamp();
	ClippedRamp(float x1, float y1, float x2, float y2, float clipLowY, float clipHighY);
	virtual ~ClippedRamp();

	float getValue(float x);

private:
	void calcInternal(float x1, float y1, float x2, float y2);
	float m; // slope
	float b; // offset
	float clipLowY, clipHighY; // clip values
};

}
} /* namespace cambada */
#endif /* CLIPPEDRAMP_H_ */
