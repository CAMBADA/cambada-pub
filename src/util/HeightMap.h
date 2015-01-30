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

#ifndef HEIGHTMAP_H_
#define HEIGHTMAP_H_

#include <stdio.h>
#include "Vec.h"
#include "geometry.h"
#include "GridView.h"
#include "libtcod.hpp"
#include "rtdb_api.h"
#include "rtdb_user.h"

#define MAP_RTDB 0
#define SCALE 0.25
#define OFFSETX 7.125
#define OFFSETY 10.125

#define SAMPLE_SCREEN_WIDTH (OFFSETX * 2 / SCALE) //57
#define SAMPLE_SCREEN_LENGTH (OFFSETY * 2 / SCALE) //81

namespace cambada {
namespace util {

class HeightMap {
public:
	HeightMap();
	virtual ~HeightMap();

	geom::Vec grid2world(int x,int y);
	void world2grid(geom::Vec pos, int &px, int &py);
	float world2grid(float val);
	void fillRtdb();
	void calculate();
	void addHill(geom::Vec point, float radius, float height);
	void digHill(geom::Vec point, float radius, float height);
	void clear();
	void addOffset(float value);
	void addOffset(geom::Circle circle, float value, bool inside);
	void addOffset(geom::XYRectangle rect, float value, bool inside);
	void addOffset(geom::Line line, float value, bool right);

	void add(HeightMap* map2);

	void cloneTo(HeightMap* destination);

	geom::Vec getMaxPos();
	geom::Vec getMinPos();
	float getMaxVal();
	float getMinVal();
	float getVal(geom::Vec pos);

	void normalize(float valMin = 0.0, float valMax = 1.0);

	void kernelTransform(int kernelsize, const int *dx,
			const int *dy, const float *weight, float minLevel, float maxLevel);

	TCODHeightMap* map;
};

}
} /* namespace cambada */
#endif /* HEIGHTMAP_H_ */
