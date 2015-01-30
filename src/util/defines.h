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

#ifndef DEFINES_H
#define DEFINES_H

//#define OUR_FIELD
//defines sizes px
//field
#ifdef OUR_FIELD
	#define FIELD_LENGTH 13600
	#define FIELD_WIDTH 5000
	#define PENALTYAREA_LENGTH 2250
	#define PENALTYAREA_WIDTH 3800
	#define GOALAREA_LENGTH 750
	#define GOALAREA_WIDTH 2750
	#define MIDFIELD_CIRCLE 2000
	#define PENALTY_OFFSET 600
	#define GOAL_OFFSET 1125
#else
	#define FIELD_LENGTH 18000
	#define FIELD_WIDTH 12000
	#define PENALTYAREA_LENGTH 2250
	#define PENALTYAREA_WIDTH 6500
	#define GOALAREA_LENGTH 750
	#define GOALAREA_WIDTH 3500
	#define MIDFIELD_CIRCLE 4000
	#define PENALTY_OFFSET 2750
	#define GOAL_OFFSET 4250
#endif

//ball
#define BALL_SIZE 220
#define TWO_METER 4000
#define THREE_METER 6000

//box
#define BOX_LENGTH (FIELD_LENGTH+2000)
#define BOX_WIDTH (FIELD_WIDTH+2000)

//robot
#define ROBOT_SIZE 500
#define NROBOTS 5

//misc
#define BORDERDIFF_L	((BOX_LENGTH-FIELD_LENGTH)*0.5)
#define BORDERDIFF_W	((BOX_WIDTH-FIELD_WIDTH)*0.5)
#define NZONES 10

//zones
#ifdef OUR_FIELD // 13600 x 5000 -> L: 2000+3500+8000, W: 1000+1500+1500+1000
	#define ZONE_1_L 2000
	#define ZONE_1_W 2500
	#define ZONE_2_L 3500
	#define ZONE_2_W 1000
	#define ZONE_3_L 8000
	#define ZONE_3_W 1000
	#define ZONE_4_L 3500
	#define ZONE_4_W 1500
	#define ZONE_5_L 8000
	#define ZONE_5_W 1500
	#define ZONE_6_L 3500
	#define ZONE_6_W 1500
	#define ZONE_7_L 8000
	#define ZONE_7_W 1500
	#define ZONE_8_L 2000
	#define ZONE_8_W 2500
	#define ZONE_9_L 3500
	#define ZONE_9_W 1000
	#define ZONE_10_L 8000
	#define ZONE_10_W 1000
#else
	// comprimento 10750 + 5000 + 2250
	// largura 2750 + 3250 + 3250 + 2750
	#define ZONE_1_L 2250
	#define ZONE_1_W 6000
	#define ZONE_2_L 5000
	#define ZONE_2_W 2750
	#define ZONE_3_L 10750
	#define ZONE_3_W 2750
	#define ZONE_4_L 5000
	#define ZONE_4_W 3250
	#define ZONE_5_L 10750
	#define ZONE_5_W 3250
	#define ZONE_6_L 5000
	#define ZONE_6_W 3250
	#define ZONE_7_L 10750
	#define ZONE_7_W 3250
	#define ZONE_8_L 2250
	#define ZONE_8_W 6000
	#define ZONE_9_L 5000
	#define ZONE_9_W 2750
	#define ZONE_10_L 10750
	#define ZONE_10_W 2750
#endif

enum zoneList{Zone_1=0,Zone_2,Zone_3,Zone_4,Zone_5,Zone_6,Zone_7,Zone_8,Zone_9,Zone_10};
enum signalList{kickOff=0,goalKick,cornerKick,freeKick,throwIn,all};
enum tipo{tReplacer=0,tReceiver,tDefault};
enum actionList{pass=0,cross,none};

#endif // DEFINES_H
