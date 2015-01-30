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

#include "HeightMap.h"

using namespace cambada::geom;

namespace cambada {
namespace util {

HeightMap::HeightMap() {
	map = new TCODHeightMap(SAMPLE_SCREEN_WIDTH,SAMPLE_SCREEN_LENGTH);
	map->clear();
}

HeightMap::~HeightMap() {
	delete map;
}

void HeightMap::calculate()
{
	//map->clear();
}

void HeightMap::addHill(geom::Vec point, float radius, float height)
{
	int px, py;
	world2grid(point, px, py);
	map->addHill(px,py,world2grid(radius),height);
}

void HeightMap::digHill(geom::Vec point, float radius, float height)
{
	int px, py;
	world2grid(point, px, py);
	map->digHill(px,py,world2grid(radius),height);
}

Vec HeightMap::grid2world(int x,int y)
{
    Vec pos;
    pos.x = x * SCALE - OFFSETX + SCALE/2.0;
    pos.y = y * SCALE - OFFSETY + SCALE/2.0;
    return pos;
}

void HeightMap::world2grid(Vec pos, int &px, int &py)
{
    px=(pos.x + OFFSETX)/SCALE;
    py=(pos.y + OFFSETY)/SCALE;
}

float HeightMap::world2grid(float val)
{
	return val/SCALE;
}

void HeightMap::fillRtdb()
{
	if(Whoami() == 0)
	{
		GridView gv;
		int count= 0;

		for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
			for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
				gv.grid[count].pos = grid2world(x, y);
				//fprintf(stderr, "FILL x %.2f %.2f\n", gv.grid[count].pos.x, gv.grid[count].pos.y);
				gv.grid[count].val = map->getValue(x, y);
				count++;
			}
		}

		gv.count = count;
		DB_put(GRIDVIEW, (void*)&gv);
	}
}

void HeightMap::clear() {
	map->clear();
}

void HeightMap::addOffset(float value) {
	map->add(value);
}

void HeightMap::addOffset(geom::Circle circle, float value, bool inside) {
	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			Vec realPt = grid2world(x,y);
			if(inside && circle.is_inside(realPt))
				map->setValue(x,y, map->getValue(x,y) + value);
			else if(!inside && !circle.is_inside(realPt))
				map->setValue(x,y, map->getValue(x,y) + value);
		}
	}
}

void HeightMap::addOffset(geom::XYRectangle rect, float value, bool inside) {
	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			Vec realPt = grid2world(x,y);
			if(inside && rect.is_inside(realPt))
				map->setValue(x,y, map->getValue(x,y) + value);
			else if(!inside && !rect.is_inside(realPt))
				map->setValue(x,y, map->getValue(x,y) + value);
		}
	}
}

void HeightMap::addOffset(geom::Line line, float value, bool right) {
	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			Vec realPt = grid2world(x,y);
			if(right && line.side(realPt) > 0)
				map->setValue(x,y, map->getValue(x,y) + value);
			else if(!right && line.side(realPt) <= 0)
				map->setValue(x,y, map->getValue(x,y) + value);
		}
	}
}

Vec HeightMap::getMaxPos() {
	Vec ret = Vec::zero_vector;
	float min, max;
	map->getMinMax(&min, &max);

	// Search value in the map

	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {

			if(map->getValue(x,y) == max)
			{
				return grid2world(x,y);
			}
		}
	}
	return ret;
}

float HeightMap::getMaxVal() {
	float min, max;
	map->getMinMax(&min, &max);
	return max;
}

Vec HeightMap::getMinPos() {
	Vec ret = Vec::zero_vector;
	float min, max;
	map->getMinMax(&min, &max);

	// Search value in the map

	for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {

			if(map->getValue(x,y) == min)
			{
				return grid2world(x,y);
			}
		}
	}
	return ret;
}

float HeightMap::getMinVal() {
	float min, max;
	map->getMinMax(&min, &max);
	return min;
}

void HeightMap::add(HeightMap* map2){
	map->add(map, map2->map);
}

void HeightMap::cloneTo(HeightMap* destination) {
	destination->map->copy(map);
	/*for (int x=0; x < SAMPLE_SCREEN_WIDTH; x++ ) {
		for (int y=0; y < SAMPLE_SCREEN_LENGTH; y++ ) {
			destination->map->setValue(x,y,map->getValue(x,y));
		}
	}*/
}

float HeightMap::getVal(Vec pos) {
	int px, py;
	world2grid(pos, px, py);
	return map->getValue(px, py);
}

void HeightMap::normalize(float valMin, float valMax) {
	map->normalize(valMin, valMax);
}

void HeightMap::kernelTransform(int kernelsize, const int *dx,
		const int *dy, const float *weight, float minLevel, float maxLevel) {
	TCODHeightMap* result = new TCODHeightMap(SAMPLE_SCREEN_LENGTH,
			SAMPLE_SCREEN_WIDTH);
	result->copy(map);
	int x, y;
	for (x = 0; x < map->w; x++) {
		int offset = x;
		for (y = 0; y < map->h; y++) {
			if (map->values[offset] >= minLevel
					&& map->values[offset] <= maxLevel) {
				float val = 0.0f;
				float totalWeight = 0.0f;
				int i;
				for (i = 0; i < kernelsize; i++) {
					int nx = x + dx[i];
					int ny = y + dy[i];
					if (nx >= 0 && nx < map->w && ny >= 0 && ny < map->h) {
						val += weight[i] * map->getValue(nx, ny);
						totalWeight += weight[i];
					}
				}
				result->values[offset] = val / totalWeight;
			}
			offset += map->w;
		}
	}
	map->copy(result);
	delete result;
}

} /* namespace util */

} /* namespace cambada */
