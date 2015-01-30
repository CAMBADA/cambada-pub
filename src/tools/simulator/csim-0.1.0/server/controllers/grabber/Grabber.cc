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

#include <cmath>

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Body.hh"

#include "BarrierSensor.hh"
#include "Grabber.hh"

#include "rtdb_sim.h"
#include "HWcomm_rtdb.h"

using namespace csim;
using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("grabber", Grabber);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Grabber::Grabber(Entity *parent )
    : Controller(parent)
{
  this->barrier = dynamic_cast<BarrierSensor*>(this->parent);

  if (!this->barrier)
    gzthrow("Grabber controller requires a BarrierSensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Grabber
Grabber::~Grabber()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Grabber::LoadChild(XMLConfigNode *node)
{   
  // Force "Always on"
  this->alwaysOnP->SetValue( true );
  
  this->angle     = node->GetFloat("angle", 5.0, 0);
  this->distance  = node->GetFloat("distance", 0.35, 0);
    
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Grabber::InitChild()
{
  this->selfID = this->barrier->GetParentModel()->GetSelfID();
  // Without ID define it will not init
  if ( this->selfID < 1 ) { 
    gzerr(0) << "Agent ID is missing, Grabber will not work\n";
    return;
  }
  
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Grabber::UpdateChild()
{

  if ( this->selfID < 1   ) return;
  
  CMD_Grabber grabber;
  DB_get_from( this->selfID, this->selfID, CMD_GRABBER, (void *)&grabber);
  if ( grabber.mode == 0 ) return;
  
  float powa = 1.0;
  
  Body* ball = this->barrier->GetBallBody();
  if ( ball == NULL ) return;
  
  // Parent parent body pose
  Pose3d ppose = this->parent->GetParent()->GetAbsPose();
  // Ball pose
  Pose3d bpose = ball->GetAbsPose();
  if (bpose.pos.z > 0.15 ) return;
  // for now, z = 0
  ppose.pos.z = 0;
  bpose.pos.z = 0;
  
  //Resolve ball position
  Pose3d  ballRelPosition = bpose - ppose;
  
  float ang      = std::atan2(ballRelPosition.pos.x, ballRelPosition.pos.y);
  float distance = ballRelPosition.pos.GetLength();
  
  bool inrange = ( std::fabs( RTOD(ang) ) < this->angle ) && ( distance < (this->distance) );
  if ( !inrange ) return;
  
  // In world coordinates
  ang =  - ang + M_PI*0.5 + ppose.rot.GetYaw();
  Vector3 attractionForce( -std::cos(ang)*distance, -std::sin(ang)*distance, 0.0 );
  attractionForce.Normalize();
  attractionForce * 100.0 * powa;
  
  // IF the body is disabled it will accumulate the force that is set
  ball->SetEnabled(true);
  ball->SetForce( attractionForce );
  
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Grabber::FiniChild()
{
}
