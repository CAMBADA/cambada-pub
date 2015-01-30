/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: CAMABADA barrier sensor
 * Author: Eurico F. Pedrosa <efp@ua.pt>
 * Date: 4 March 2010
 */

#include <sstream>
#include <cmath>
#include "Global.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "Model.hh"
#include "Body.hh"

#include "SensorFactory.hh"
#include "BarrierSensor.hh"

#include "rtdb_sim.h"
#include "WorldStateDefs.h"

using namespace csim;
using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("barrier", BarrierSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
BarrierSensor::BarrierSensor(Body *body)
    : Sensor(body)
{
  this->typeName = "barrier";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BarrierSensor::~BarrierSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
// Load the camera
void BarrierSensor::LoadChild( XMLConfigNode *node )
{
  this->ballModelName = node->GetString("ballModel", std::string(), 1);
  this->angle     = node->GetFloat("angle", 30.0, 0);
  this->distance  = node->GetFloat("distance", 0.28, 0);
  
  this->selfID = this->GetParentModel()->GetSelfID();
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void BarrierSensor::InitChild()
{
  // Without ID define it will not init
  if ( this->selfID < 1 ) { 
    gzerr(0) << "Agent ID is missing, Barrier will not update\n";
    return;
  }
  
  Model* ballModel = World::Instance()->GetModelByName( this->ballModelName );
  
  if ( ballModel == NULL ){
    this->ball = NULL;
    gzerr(0) << "Ball model not found. Barrier disabled" << std::endl;
    
  }else{
    // I assume there is only one body, makes no sense otherwise.
    this->ball = ballModel->GetBody();
  }
  
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void BarrierSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void BarrierSensor::UpdateChild()
{
  if ( this->selfID < 1  ) return;
  if ( this->ball == NULL )return;
  
  // Send info..
	CMD_Info info;
  
  // Batteries voltage..
	info.Voltage_logic    = 74;  // 12V
	info.Voltage_power12  = 150; // 12V
	info.Voltage_power24  = 150; // 12V
	info.Kicker_voltage	  = 400; // 
	
	// Parent pose
  Pose3d ppose = this->body->GetAbsPose();
  // Ball pose
  Pose3d bpose = this->ball->GetAbsPose();
  float  ballAltitude = bpose.pos.z;
  
  // for now, z = 0
  ppose.pos.z = 0;
  bpose.pos.z = 0;
  
  //Resolve ball position
  Pose3d ballRelPosition = bpose - ppose;
  
  float ang = std::atan2(ballRelPosition.pos.x, ballRelPosition.pos.y);

  this->closed_circuit  = ( std::fabs( RTOD(ang) ) < this->angle ) && 
                          ( ballRelPosition.pos.GetLength() < (this->distance) );

  CMD_Grabber_Info ginfo;
  if ( this->closed_circuit && (ballAltitude < 0.15) ){
    ginfo.leftArm = 100; ginfo.rightArm = 100;
  }else{
    ginfo.leftArm = 0; ginfo.rightArm = 0;
  }
  //info.Grabber_barrier  = this->closed_circuit && (ballAltitude < 0.15);
	

  DB_put_in(this->selfID, this->selfID, CMD_INFO, (void *)&info , 0);
  DB_put_in(this->selfID, this->selfID, CMD_GRABBER_INFO, (void *)&ginfo , 0);
  
}

