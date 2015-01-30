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
#include "Shape.hh"
#include "SphereShape.hh"

#include "BarrierSensor.hh"
#include "Kicker.hh"

#include "rtdb_sim.h"
#include "HWcomm_rtdb.h"

using namespace csim;
using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("kicker", Kicker);

////////////////////////////////////////////////////////////////////////////////
// Constructor
Kicker::Kicker(Entity *parent )
    : Controller(parent)
{
  this->barrier = dynamic_cast<BarrierSensor*>(this->parent);

  if (!this->barrier)
    gzthrow("Kicker controller requires a BarrierSensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Kicker::~Kicker()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Kicker::LoadChild(XMLConfigNode *node)
{   
  // Force "Always on"
  this->alwaysOnP->SetValue( true );
  
  this->kickerTilt = DTOR (node->GetFloat("tilt", 45.0, 0 ) );
  this->highDistance = node->GetFloat("highDistance", 12, 0);
  this->lowVel  = node->GetFloat("lowVel", 11.5, 0);
    
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Kicker::InitChild()
{
  this->selfID = this->barrier->GetParentModel()->GetSelfID();
  // Without ID define it will not init
  if ( this->selfID < 1 ) { 
    gzerr(0) << "Agent ID is missing, Kicker will not work\n";
    return;
  }
  
  // Calculate high kick max velocity
  this->highVel = std::sqrt( (this->highDistance * 9.8) / std::sin(2*this->kickerTilt) ); 
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Kicker::UpdateChild()
{

  if ( this->selfID < 1   ) return;
  
  // Ball not in barrier area..
  if ( this->barrier->BallOnBarrierArea() != true ) return;
  
  Body* ball = this->barrier->GetBallBody();
  if ( ball == NULL ) return;
  
  CMD_Kicker kicker;
  DB_get_from(this->selfID, this->selfID, CMD_KICKER, (void *)&kicker);

  unsigned char kickPowa = kicker.power;
  
  // Only kick the ball when the power > 0
  if ( kickPowa == 0 ) return;
  
  float swivel;
  float tilt;
  float speed;
  
  const int maxPowa = 50;
  
  // Parent parent body pose
  Pose3d ppose = this->parent->GetParent()->GetAbsPose();
  // Ball pose
  Pose3d bpose = ball->GetAbsPose();
  // for now, z = 0
  ppose.pos.z = 0;
  bpose.pos.z = 0;
  
  //Resolve ball position
  Pose3d  ballRelPosition = bpose - ppose;
  Vector3 velToApply;
  
  // Robbie direction
  swivel = M_PI*0.5 + ppose.rot.GetYaw();

  if ( kickPowa < 128 ){
    // High kick
    kickPowa = kickPowa > 50 ? 50 : kickPowa;
    speed = 2 * this->highVel * kickPowa / maxPowa;
    tilt  = this->kickerTilt;
    
  }else{
    // Low kick
    kickPowa -= 128;
    kickPowa = kickPowa > 50 ? 50 : kickPowa;

    //HACK
    kickPowa = 1;

    speed = 20 * this->highVel * kickPowa / maxPowa;
    tilt  = 0.0;
  }
   
  float dirx = std::cos(tilt) * std::cos(swivel);
  float diry = std::cos(tilt) * std::sin(swivel);
  float dirz = std::sin(tilt);
  
  velToApply.x = dirx * speed; 
  velToApply.y = diry * speed;
  velToApply.z = dirz * speed;
  
  this->barrier->GetBallBody()->SetLinearVel( velToApply );
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Kicker::FiniChild()
{
}
