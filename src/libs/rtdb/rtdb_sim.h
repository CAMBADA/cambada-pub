/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA RTDB
 *
 * CAMBADA RTDB is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA RTDB is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __RTDB_SIM_H
#define __RTDB_SIM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtdbdefs.h"

//	*************************
//	DB_init_all: Aloca acesso a base de dados
//
//	Saida:
//		0 = OK
//		-1 = erro
//
int DB_init_all (int _second_rtdb);

void DB_free_all (int _second_rtdb);

int DB_put_in (int _agent, int _to_agent, int _id, void *_value, int life);

int DB_get_from (int _agent, int _from_agent, int _id, void *_value);

void DB_set_config_file(const char* cf);

#ifdef __cplusplus
}
#endif

#endif
