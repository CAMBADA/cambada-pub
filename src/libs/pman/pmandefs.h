/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA PMAN
 *
 * CAMBADA PMAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA PMAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAMDEFS_H_INCLUDED
#define CAMDEFS_H_INCLUDED


#define SHMEM_FCAM_PMAN_KEY 0x9010 /* Shared memory key, PMAN, front camera */
#define SHMEM_OCAM_PMAN_KEY 0x9011 /* Shared memory key, PMAN, omnidirectional camera */
#define SEM_FCAM_PMAN_KEY   0x9010 /* Semaphore key for PMAN primitives, front camera */
#define SEM_OCAM_PMAN_KEY   0x9011 /* Semaphore key for PMAN primitives, omni camera */

// SHMEM and SEM can have the same key
//#define PMAN_SHARED_KEY(_seed)   (0x9100 + 2*_seed)

#endif
