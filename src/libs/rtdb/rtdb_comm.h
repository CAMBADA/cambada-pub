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

#ifndef __RTDB_COMM_H
#define __RTDB_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtdb_api.h"


//	*************************
//	DB_comm_put: Escreve na base de dados - apenas para outros agentes!
//
//	Entrada:
//		int _agent = numero do agente
//		int _id = identificador da 'variavel'
//		void *_value = ponteiro com os dados
//		int life = tempo de vida da 'variavel' em ms
//	Saida:
//		0 = OK
//		-1 = erro
//
int DB_comm_put (int _agent, int _id, int _size, void *_value, int life);


//	*************************
//	DB_comm_ini: 
//
//	Entrada:
//		int _id = array com identificadores da 'variavel'
//		int _size = array com temanhos da 'variavel'
//		int _period = array com periodos da 'variavel'
//	Saida:
//		int n_shared_recs = numero de 'variaveis' shared
//		-1 = erro
//
int DB_comm_ini(RTDBconf_var *rec);

#ifdef __cplusplus
}
#endif


#endif
