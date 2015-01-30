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
/*
 * Desc: Position2d controller for a Holonome3Sw drive.
 * Author: Christian Gagneraud (ch'Gans), based on Differential_Position2d.cc
 * Date: 21 Feb 2007
 * SVN info: $Id: Holonome3Sw_Position2d.cc 8433 2009-11-22 20:15:37Z natepak $
 */

#include "XMLConfig.hh"
#include "Model.hh"
#include "Body.hh"
#include "Global.hh"
#include "Joint.hh"
#include "World.hh"
#include "Simulator.hh"
#include "gazebo.h"
#include "PhysicsEngine.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Holonome3Sw_Position2d.hh"

#include "rtdb_sim.h"
#include "HWcomm_rtdb.h"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("holonome3sw_position2d", Holonome3Sw_Position2d);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
  Holonome3Sw_Position2d::Holonome3Sw_Position2d(Entity *parent )
: Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Holonome3Sxw_Position2d controller requires a Model as its parent");

  ResetData();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Holonome3Sw_Position2d::~Holonome3Sw_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Holonome3Sw_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<PositionIface*>(this->GetIface("position"));

  this->selfID = this->myParent->GetSelfID();  ;//node->GetInt("selfID", -1, 1);
  
  // Force "Always on"
  this->alwaysOnP->SetValue( true );
    

  this->userVelocities = node->GetBool("uservelocities", false, 0);
  this->xya = node->GetVector3("velocities", Vector3());

  // Get wheels child
  node = node->GetChild("wheels");
  if (!node)
    gzthrow("Holonome3Sw_Position2d controller requires a <wheels> node");

  // Get default params
  float distdef = node->GetFloat("distance", 0.0, 0);
  float radiusdef = node->GetFloat("radius", 0.0, 0);
  float maxtorquedef = node->GetFloat("torque", 0.0, 0);
  float betadef = node->GetFloat("beta", 0.0, 0);
  float gammadef = node->GetFloat("gamma", 0.0, 0);

  // Get params for each wheels
  for (size_t i = 0; i < 3; ++i)
  {
    std::ostringstream sw; sw << "swedish" << i;
    XMLConfigNode *lnode = node->GetChild(sw.str());
    if (!lnode)
      gzthrow("The controller couldn't get swedish wheels " + sw.str());

    std::string joint =  lnode->GetString("joint", "", 1);
    this->joint[i] = this->myParent->GetJoint(joint);

    if (!this->joint[i])
      gzthrow("The controller couldn't get hinge joint for " + sw.str());

    this->ALPHA[i] = lnode->GetFloat("alpha", 1000000, 1);
    if (this->ALPHA[i] == 1000000)
      gzthrow("The controller could't get alpha param for " + sw.str());

    this->DIST[i] = lnode->GetFloat("distance", distdef, 0);
    if (!this->DIST[i])
      gzthrow("The controller could't get distance param for " + sw.str());

    this->BETA[i] = lnode->GetFloat("beta", betadef, 0);

    this->GAMMA[i] = lnode->GetFloat("gamma", gammadef, 0);

    this->RADIUS[i] = lnode->GetFloat("radius", radiusdef, 0);
    if (!this->RADIUS[i])
      gzthrow("The controller could't get radius param for " + sw.str());

    this->MAXTORQUE[i] = lnode->GetFloat("torque", maxtorquedef, 0);
    if (!this->MAXTORQUE[i])
      gzthrow("The controller could't get torque param for " + sw.str());

    // Pre-compute some constants
    A[i] = (2*3.14159265358979)*fmodf(ALPHA[i]+BETA[i]+GAMMA[i], 360)/360;
    L[i] = DIST[i]*cos((2*3.14159265358979)*fmodf(BETA[i]+GAMMA[i], 360)/360);
    R[i] = RADIUS[i]*cos((2*3.14159265358979)*fmodf(GAMMA[i], 360)/360);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
/*void Holonome3Sw_Position2d::SaveChild(XMLConfigNode *node)
  {
  node = node->GetChild("wheels");
  if (!node)
  gzthrow("No such child node: wheels");
  for (size_t i = 0; i < 3; ++i)
  {
  std::ostringstream sw; sw << "swedish" << i;
  XMLConfigNode *lnode = node->GetChild(sw.str());
  if (!lnode)
  gzthrow("No such child node wheels/" + sw.str());
  lnode->SetValue("alpha", this->ALPHA[i]);
  lnode->SetValue("distance", this->DIST[i]);
  lnode->SetValue("beta", this->BETA[i]);
  lnode->SetValue("gamma", this->GAMMA[i]);
  lnode->SetValue("radius", this->RADIUS[i]);
  lnode->SetValue("max-torque", this->MAXTORQUE[i]);
// "joint" ???
}
}*/

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Holonome3Sw_Position2d::InitChild()
{
  ResetData();
}

void Holonome3Sw_Position2d::ResetChild()
{
  ResetData();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Holonome3Sw_Position2d::UpdateChild()
{
  if ( this->selfID == - 1 ) return;
  
  this->GetPositionCmd();
  this->UpdateOdometry();

  // I don't know if this is enough for... for now it works
  ((Model*)(this->parent))->GetBody()->SetEnabled( true );

  for (size_t i = 0; i < 3; ++i)
  {
    if (this->enableMotors)
    {
      this->joint[i]->SetVelocity( 0, this->PhiP[i]);
      this->joint[i]->SetMaxForce( 0, this->MAXTORQUE[i] );
    }
    else
    {
      this->joint[i]->SetVelocity( 0, 0 ); 
      this->joint[i]->SetMaxForce( 0, 0 );
    }
  }
  //std::cout << "}" << std::endl;
  this->PutPositionData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Holonome3Sw_Position2d::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset internal data to 0
void Holonome3Sw_Position2d::ResetData()
{

  this->odomX = 0.0;
  this->odomY = 0.0;
  this->odomA = 0.0;

  for (size_t i = 0; i < 3; ++i)
  {
    Xi[i] = 0;
    XiP[i] = 0;
    PhiP[i] = 0;
  }
  this->enableMotors = true;
}

//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Holonome3Sw_Position2d::GetPositionCmd(){
  
    /* Fetch data from the RTDB to the CAN bus */
    CMD_Vel vel;

    int lt = DB_get_from( this->selfID, this->selfID, CMD_VEL, (void *)&vel );

    if ( (lt < 0) || (lt > 500) ){
        vel.vx=0.0;
        vel.vy=0.0;
        vel.va=0.0;
    }

    float vx, vy, va;

    if ( this->userVelocities ){
        vx = this->xya.x;
        vy = this->xya.y;
        va = this->xya.z;
    }else{
        vx = vel.vx;
        vy = vel.vy;
        va = vel.va;
    }

  vel.vx = hm.velX;
  vel.vy = hm.velY;
  vel.va = -hm.w_rot;
  DB_put_in(this->selfID, this->selfID, LAST_CMD_VEL, (void *)&vel, 0);
  
  int s1, s2, s3;  
  float inv_factor = iM_2_PI / ( N_CONT*DESMUL*TIME_TICK*K_HCTL);

  
  // s1 == RODA 3; s2 == RODA 1; s3 == RODA 2
  this->hm.GetSetPoints( vx, vy, va, &s1, &s2, &s3 );
  
  this->PhiP[0] = s2*inv_factor;
  this->PhiP[1] = s3*inv_factor;
  this->PhiP[2] = s1*inv_factor; 
}


void Holonome3Sw_Position2d::UpdateOdometry(){

  // Update
  Time stepTime;
  stepTime = Simulator::Instance()->GetSimTime() - this->prevUpdateTime;
  this->prevUpdateTime = Simulator::Instance()->GetSimTime();

  this->odomX += (hm.velX *std::cos(this->odomA) - hm.velY*sin(this->odomA)) * stepTime.Double();
  this->odomY += (hm.velX *std::sin(this->odomA) + hm.velY*cos(this->odomA)) * stepTime.Double();
  this->odomA += -hm.w_rot * stepTime.Double();

  fprintf(stderr,"Sim: odomA %.3f\n",this->odomA);

  // Normalize..
  if ( this->odomA < 0.0  ){
    double a = std::ceil(- this->odomA / (2*M_PI) );
    this->odomA += a*(2*M_PI);
  }
  if ( this->odomA > 2*M_PI ){
    double a = std::floor(this->odomA / (2*M_PI) );
    this->odomA -=  a*(2*M_PI);
  }

  // Put the odometry information in the rtdb
  CMD_Pos odom;
  odom.px = this->odomX;
  odom.py = this->odomY;
  odom.pa = this->odomA;
  odom.da = 0.0;
  odom.dx = 0.0;
  odom.dy = 0.0;

  DB_put_in( this->selfID, this->selfID, CMD_POS, (void*)&odom, 0);
}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Holonome3Sw_Position2d::PutPositionData()
{
  if (this->myIface->Lock(1))
  {
    // TODO: Data timestamp
    this->myIface->data->head.time = Simulator::Instance()->GetSimTime().Double();

    this->myIface->data->pose.pos.x = this->Xi[0];
    this->myIface->data->pose.pos.y = this->Xi[1];
    this->myIface->data->pose.yaw = NORMALIZE(this->Xi[2]);

    this->myIface->data->velocity.pos.x = 0;
    this->myIface->data->velocity.pos.y = 0;
    this->myIface->data->velocity.yaw = 0;

    // TODO
    this->myIface->data->stall = 0;

    this->myIface->Unlock();
  }
}
