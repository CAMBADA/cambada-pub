/*
 *  CSim - CAMBADA Simulator
 *  Copyright (C) 2010  Universidade de Aveiro
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
 *  @Author Eurico F. Pedrosa <efp@ua.pt>
 *  @Date   2 Jun 2010
 *  @Desc   Automatic Referee
 *
 */
 
#include "GazeboMessage.hh"
#include "GazeboError.hh"

#include "World.hh"
#include "Model.hh"
#include "Geom.hh"
#include "Field.hh"
#include "XMLConfig.hh"
#include "ContactSensor.hh"
#include "Referee.hh"
#include "Socket.hh"
#include "OldRefBoxProtocol.hh"


using namespace gazebo;
using namespace csim;


void Referee::Load(XMLConfigNode* node){

  XMLConfigNode *refereeNode = node->GetChildByNSPrefix("referee");
  
  if ( refereeNode == NULL ){
    this->enabled = false;
    return;
  }

  this->ballModelName     = refereeNode->GetString("ballModel",     std::string(), 1);
  this->contactSensorName = refereeNode->GetString("contactSensor", std::string(), 1);

  this->socketPort = refereeNode->GetInt("port", 28097, 0);
  
  // pre-activate the Referee
  this->enabled = true;
}

void Referee::Init(){

  if ( this->enabled == false )
    return;
  
  // 
  this->enabled = false;  
  
  Model* ballModel = World::Instance()->GetModelByName( this->ballModelName );
  if ( ballModel == NULL ){
    gzerr(0) << "Ball model not found. Referee disabled" << std::endl;
    return;
  }
  
  this->ball = ballModel->GetBody();
  
  this->contactSensor = dynamic_cast<ContactSensor*>( ballModel->GetSensor( this->contactSensorName ) );
  if ( this->contactSensor == NULL ){
    gzerr(0) << "Ball contact sensor not found. Referee disabled" << std::endl;
    return;
  }
  
  // The contact must have only one geom, not less or more..
  if ( this->contactSensor->GetGeomCount() != 1 ){
    gzerr(0) << "Ball contact sensor must have [only] one geometry. Referee disabled" << std::endl;
    return;
  }
  
  // Keep a reference to the game field
  this->field = World::Instance()->GetField();
  // Set last contact to none
  this->lastContactTeam = TEAM_NONE;
  
  // RefereeBox protocol
  this->protocol = new OldRefBoxProtocol();
  // Start socket listening thread..
  this->socketLoop = new boost::thread( boost::bind( &Referee::SocketLoop, this )  );
}

void Referee::SocketLoop(){


  while( !this->shouldQuit ){

    try {
    
      ServerSocket refereeServer( this->socketPort );

      // First Connection
      this->conn1 = new ServerSocket();
      refereeServer.accept( *this->conn1 );
      
      // Welcome the client
      this->protocol->Welcome( *this->conn1 );
      std::cout << "Client 1 connected" << std::endl;
      
      // Second Connection
      //this->conn2 = new SocketServer();
      //refereeServer.accept( this->conn2 );
      
      
    }catch ( SocketException& e ){
        std::cout << "Exception was caught:" << e.description() << "\nExiting.\n";
    }
    
    // Enable referee
    this->enabled = true;
    
    // This will wait until new connections are needed
    {// scoped mutex ??
      boost::unique_lock<boost::mutex> lock(this->mut_lc);
      while( !this->newClients ){
          this->listenClients.wait(lock);
      }
    }
    
    this->newClients = false;
  
  }// End WHILE


}

void Referee::Fini(){

}


void Referee::ApplyRules(){

  if ( this->enabled == false ) return;

  // check for contacts
  unsigned int contactCount = this->contactSensor->GetGeomContactCount(0);
  unsigned int i;
  for ( i = 0; i < contactCount; i++ ){
    Contact contact = this->contactSensor->GetGeomContact(0, i);
    // contact::geom1 -- is allways the ball geometry.
    
    if ( contact.geom2->GetType() == Shape::PLANE )
      continue; // Skip plane contact, not usefull..
      
    Model* model = contact.geom2->GetModel();
    if ( model->GetTeam().empty() ) continue; // no team assigned
    
    std::string team = model->GetTeam();
    if ( team.compare("magenta") == 0 ){
      this->lastContactTeam = TEAM_MAGENTA;
    }else if( team.compare("cyan") == 0 ){
      this->lastContactTeam = TEAM_CYAN;
    }
  }  
  
    Vector3 ballPos = this->ball->GetAbsPose().pos;
  // Is ball within field limits ?
  if ( ( std::fabs(ballPos.x) <= this->field->fieldWidth*0.5 ) &&
       ( std::fabs(ballPos.y) <= this->field->fieldLength*0.5)   ){
       
    // The ball is inside the field
    return;
  }
  
  // The ball is outside the field limits
  // Check for throw-ins
  if ( std::fabs( ballPos.x ) > this->field->fieldWidth*0.5 ){
    std::cout << "The ball is out, last contact " << this->lastContactTeam << std::endl; 
  }

}

/* Private functions ..*/

Referee::Referee(){
  this->enabled    = false;
  this->socketLoop = NULL;
  this->shouldQuit = false;
  
  this->conn1 = NULL;
  this->conn2 = NULL;
  
  this->newClients = false;
}

Referee::~Referee(){

}

