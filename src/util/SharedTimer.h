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

#ifndef _SHARED_TIMER_
#define _SHARED_TIMER_

#include <sys/time.h>

#define KEY 0x777777

namespace cambada
{
namespace util
{
class SharedTimer
{
private:
    struct shmData
    {
        int cnt;
        struct timeval baseTime;
    };

    struct shmData* data;
    struct timeval savedTime;

    int shmid;
    int semid;

    void adquireAcess();
    void releaseAccess();

public:
    SharedTimer();
    ~SharedTimer();

    void resetTime();
    void saveCurrentTime();
    int getLongTermTime();
    inline int elapsed() { return getLongTermTime(); }
    int getShortTermTime();
};
}
}
#endif
