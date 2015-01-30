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

#include "Clock.h"

#include <sys/time.h>
#include <time.h>

cambada::util::Clock::Clock()
{
	// TODO Auto-generated constructor stub

}

cambada::util::Clock::~Clock()
{
	// TODO Auto-generated destructor stub
}

float cambada::util::Clock::now()
{
	struct timeval tv;
	gettimeofday( &tv , NULL );
	return tv.tv_sec*1000+tv.tv_usec/1000;
}



