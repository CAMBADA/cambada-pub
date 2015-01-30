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

#ifndef HWCOMM_H
#define HWCOMM_H

#include <assert.h>
#include <stdio.h>
#include "rtdb_user.h"
#include "rtdb_api.h"
#include "rtdbdefs.h"
#include "common.h"

/**
 * \file HWcomm_rtdb.h
 *
 * \brief RTDB items related to robot base
 *
 * The following RTDB items are defined:
 *   CMD_VEL, CMD_KICKER, CMD_GRABBER, 
 *   CMD_POS, CMD_COMPASS, CMD_INFO, CMD_HWERROS, and
 *   LAST_CMD_VEL
 */

////////////////////////////////////////////////////////////////////////////////////
struct CMD_Grabber_Info
{
	unsigned int leftArm;
	unsigned int rightArm;
	bool leftHandicapped;
	bool rightHandicapped;
};

inline int CMD_Grabber_Info_GET(unsigned int *lArm, unsigned int *rArm, bool *lHandicap, bool *rHandicap)
{
	CMD_Grabber_Info gInfo;
	int lt = DB_get( Whoami(), CMD_GRABBER_INFO, (void *)&gInfo);
	*lArm = gInfo.leftArm;
	*rArm = gInfo.rightArm;
	if (lHandicap!=NULL) *lHandicap = gInfo.leftHandicapped;
	if (rHandicap!=NULL) *rHandicap = gInfo.rightHandicapped;
	return lt;
}

struct CMD_Grabber_Config
{
	int configId;	//0 is not config, 1 is to config low, 2 is to config high 
};

struct CMD_Vel
{
//	CMD_Vel(float velX=0.0,float velY=0.0, float velA=0.0 ) : vx(velX+0.001),vy(velY+0.001),va(velA+0.001) { }
	
	float vx, vy, va;
	bool smallAdjustment;
}; //size 12

inline void CMD_Vel_SET( float vx, float vy, float vr, bool smallAdjustment)
{
	CMD_Vel vel;
	vel.vx = vx;
	vel.vy = vy;
	vel.va = vr;
	vel.smallAdjustment = smallAdjustment;
	DB_put( CMD_VEL, (void *)&vel );
}

inline int HWcomm_Get_Vel( float *vx, float *vy, float *vr)
{
	/* Fetch data from the RTDB to the CAN bus */
	CMD_Vel vel;

	int lt;
	if ( ( lt = DB_get( Whoami(), CMD_VEL, (void *)&vel ) ) < 0){
		if ( ( lt = DB_get( Whoami(), CMD_VEL, (void *)&vel ) ) < 0){
			vel.vx=0.01;
			vel.vy=0.01;
			vel.va=0.01;
		}
	}

	*vx = vel.vx;
	*vy = vel.vy;
	*vr = vel.va;
	return lt;
}

inline void LAST_CMD_Vel_SET( float vx, float vy, float vr)
{
	CMD_Vel vel;
	vel.vx = vx;
	vel.vy = vy;
	vel.va = vr;
	DB_put( LAST_CMD_VEL, (void *)&vel );
}

////////////////////////////////////////////////////////////////////////////////////

struct CMD_Kicker
{
	unsigned char power; // if power==0  do not kick
	                     // else kick
						 // 
						 // After the first kick command transmission this var
						 // is reseted to 0
	unsigned char engLvlHigh;
	unsigned char engLvlLow;
};//size 1


inline void CMD_Kicker_SET(unsigned char power,
		unsigned char highThreshold = ENGAGED_THRESHOLD_HIGH,
		unsigned char lowThreshold = ENGAGED_THRESHOLD_LOW)
{
	CMD_Kicker kicker;
	kicker.power=power;
	kicker.engLvlHigh = highThreshold;
	kicker.engLvlLow = lowThreshold;
	DB_put( CMD_KICKER, (void *)&kicker );
}

inline int HWcomm_Get_Kicker(unsigned char *kick)
{
	CMD_Kicker kicker;
	int lt = DB_get( Whoami(), CMD_KICKER, (void *)&kicker);
	*kick = kicker.power;
	return lt;
}

////////////////////////////////////////////////////////////////////////////////////

struct CMD_Grabber
{
	unsigned char mode; //0-100
};//size 1

inline void CMD_Grabber_SET(unsigned char mode)
{
	CMD_Grabber grabber;
	grabber.mode=mode;
	DB_put( CMD_GRABBER, (void *)&grabber );
}

inline void HWcomm_Get_Grabber(unsigned char *mode)
{
	CMD_Grabber grabber;
	DB_get( Whoami(), CMD_GRABBER, (void *)&grabber);
	*mode = grabber.mode;
}


////////////////////////////////////////////////////////////////////////////////////

struct CMD_Pos
{
	float px,py,pa;
	float dx,dy,da;
}; //size 12 each triple, 24 total

inline int CMD_Pos_GET(float *px, float *py, float *pr, float *dx, float *dy, float *da)
{
	CMD_Pos pos;
	int lf = DB_get( Whoami(), CMD_POS, (void *)&pos );
	*px = pos.px;
	*py = pos.py;
	*pr = pos.pa;
	*dx = pos.dx;
	*dy = pos.dy;
	*da = pos.da;
	return lf;
}

inline void HWcomm_Set_Hodometry(float px, float py, float pr, float dx, float dy, float dr)
{
	CMD_Pos pos;
	DB_get( Whoami(), CMD_POS, (void *)&pos );
	pos.px = px;
	pos.py = py;
	pos.pa = pr;
	pos.dx = dx;
	pos.dy = dy;
	pos.da = dr;
	DB_put( CMD_POS, (void *)&pos );
}


////////////////////////////////////////////////////////////////////////////////////

struct CMD_Imu
{
	int compass, yaw, rawYaw;
}; //size 12

inline void HWcomm_Set_IMU(int compass, int yaw, int rawYaw)
{
	CMD_Imu info;
	DB_get( Whoami(), CMD_IMU, (void *)&info );
	info.compass = compass;
	info.yaw = yaw;
	info.rawYaw = rawYaw;
	DB_put( CMD_IMU , (void*)(&info) );
}

inline void HWcomm_Get_IMU(int *compass, int *yaw, int *rawYaw)
{
	CMD_Imu info;
	DB_get( Whoami(), CMD_IMU, (void *)&info );
	*compass = info.compass;
	*yaw = info.yaw;
	*rawYaw = info.rawYaw;
}

inline void CMD_set_orientation( int orientation)
{
	DB_put( CMD_SYNCIMU, (void *)&orientation );
}

////////////////////////////////////////////////////////////////////////////////////

struct CMD_Info
{
	unsigned short Voltage_logic;
	unsigned short Voltage_power12;
	unsigned short Voltage_power24;
	unsigned short Kicker_voltage;
	bool justKicked;
};

inline int CMD_Info_GET(
		unsigned short *Voltage_logic,
		unsigned short *Voltage_power12,
		unsigned short *Voltage_power24,
		unsigned short *Kicker_voltage,
		bool *justKicked)
{
	CMD_Info info;

	int lf = DB_get( Whoami(), CMD_INFO, (void *)&info );
	
	if (Voltage_logic!=NULL) *Voltage_logic = info.Voltage_logic;
	if (Voltage_power12!=NULL) *Voltage_power12 = info.Voltage_power12;
	if (Voltage_power24!=NULL) *Voltage_power24 = info.Voltage_power24;
	if (Kicker_voltage!=NULL) *Kicker_voltage = info.Kicker_voltage;
	if (justKicked!=NULL) *justKicked = info.justKicked;

	return lf;
}

inline void HWcomm_Set_RobotStatus(
		unsigned short Voltage_logic,
		unsigned short Voltage_power12,
		unsigned short Voltage_power24,
		unsigned short Kicker_voltage,
		bool justKicked)
{
	CMD_Info info;

	info.Voltage_logic = Voltage_logic;
	info.Voltage_power12 = Voltage_power12;
	info.Voltage_power24 = Voltage_power24;
	info.Kicker_voltage = Kicker_voltage;
	info.justKicked = justKicked;

	DB_put( CMD_INFO, (void *)&info );
}


////////////////////////////////////////////////////////////////////////////////////

struct CMD_HWerrors
{
	unsigned short (error_count[5])[8]; //5 - number of error types
	                                  //8 - number of modules
};

////////////////////////////////////////////////////////////////////////////////////

inline void HWcomm_rtdb_init(void)
{
	assert( DB_init() != -1 );
}

inline void HWcomm_rtdb_close(void)
{
	DB_free();
}


////////////////////////////////////////////////////////////////////////////////////

struct CMD_Robot_connection
{
	bool connection; //determines if there is connection with the hardware
};//size 1


#endif


