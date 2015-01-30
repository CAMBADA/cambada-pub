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

#include "SlidingWindow.h"
#include <cassert>




cambada::util::SlidingWindow::SlidingWindow(int n)
	: maxSamples(n)
{
}

cambada::util::SlidingWindow::SlidingWindow(std::deque<float> samples,unsigned int n)
	: maxSamples(n)
{
	assert(n >= samples.size());
	for (unsigned int i = 0; i < samples.size(); ++i)
		this->samples.push_back(samples[i]);
}

cambada::util::SlidingWindow::~SlidingWindow()
{
}

void cambada::util::SlidingWindow::addValue(float v)
{
	samples.push_back(v);
	if(getNumSamples() > maxSamples)
		samples.pop_front();
}

void cambada::util::SlidingWindow::resetSamples(unsigned int n)
{
	assert(n < maxSamples);
	if(getNumSamples() > n)
		samples.erase(samples.begin(), samples.begin() + samples.size() - n);
	//while(samples.size() > n)
	//	samples.pop_front();
}

float cambada::util::SlidingWindow::sum(void)
{
	float sum = 0.0f;
	for (unsigned int i = 0; i < samples.size(); ++i)
	{
		sum += samples[i];
	}
	return sum;
}

float cambada::util::SlidingWindow::mean(void)
{
	return sum()/getNumSamples();
}

unsigned int cambada::util::SlidingWindow::getNumSamples()
{
	return samples.size();
}

float cambada::util::SlidingWindow::getSample(unsigned int index)
{
	assert(index < getNumSamples());
	return samples[index];
}

void cambada::util::SlidingWindow::setSample(unsigned int index, float v)
{
	assert(index < getNumSamples());
	samples[index] = v;
}

std::deque<float> cambada::util::SlidingWindow::getSamples()
{
	return samples;
}

void cambada::util::SlidingWindow::clear(void)
{
	samples.clear();
}

unsigned int cambada::util::SlidingWindow::getMaxSamples() const
{
    return maxSamples;
}

bool cambada::util::SlidingWindow::isFull()
{
	return getNumSamples() == getMaxSamples();
}
