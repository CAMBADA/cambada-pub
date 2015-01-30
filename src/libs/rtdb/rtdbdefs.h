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

#ifndef __RTDBDEFS_H
#define __RTDBDEFS_H

// #define DEBUG

#ifdef __cplusplus
extern "C" {
#endif

#define SELF 3333

#define CONFIG_FILE	"../config/rtdb.ini"

#define SHMEM_KEY 0x2000
#define SHMEM_SECOND_TEAM_KEY 0x3000

// definicoes hard-coded
// alterar de acordo com a utilizacao pretendida

#define MAX_AGENTS 7	// numero maximo de agentes
#define MAX_RECS 100	// numero maximo de 'variaveis' (shared + local)

// fim das definicoes hard-coded

typedef struct
{
	int id;				// identificador da 'variavel'
	int size;			// tamanho de dados
	int period;			// periodicidade de refrescamento via wireless
} RTDBconf_var;

#ifdef __cplusplus
}
#endif

#endif
