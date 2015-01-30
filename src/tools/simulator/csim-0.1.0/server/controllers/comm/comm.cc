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
 *  @Date   31 Jan 2010
 *  @Desc   CAMBADA comm module
 *
 */

//#define Vec VecGZ
#include "gazebo.h"
//#undef  Vec

#include "Global.hh"
#include "XMLConfig.hh"
#include "Model.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "IfaceFactory.hh"
#include "comm.hh"

#include "rtdb.h"
#include "rtdb_sim.h"
#include "HWcomm_rtdb.h"

#include "Robot.h"
#include "CoachInfo.h"
#include "KickCalibData.h"
#include "GridView.h"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("comm", Comm);

int Comm::commLoaded = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Comm::Comm(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if ( Comm::commLoaded == 2 )
    gzthrow("Only two instance of Comm controller are allowed");

  if (!this->myParent)
    gzthrow("Comm controller requires a Model as its parent");
    
  if ( this->myParent->GetType() != "empty")
    gzthrow("Comm controller requires an empty Model as its parent");
  
  this->rtdbNum = Comm::commLoaded;
  Comm::commLoaded += 1;
  this->fRTDB = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Comm::~Comm()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Comm::LoadChild(XMLConfigNode *node)
{
  // Force "Always on"
  this->alwaysOnP->SetValue( true );
  this->rtdbConfigFile = node->GetFilename("rtdbConf", std::string(), 0);
  
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Comm::InitChild()
{
  if ( this->rtdbConfigFile != "" )
    DB_set_config_file( this->rtdbConfigFile.c_str() );
  
  int _second_rtdb;
  if ( this->rtdbNum == 1)
    _second_rtdb = 1;
  else
    _second_rtdb = 0;
    
  if ( DB_init_all( _second_rtdb ) == -1 )
      gzthrow("Unable to init rtdb");
  
  // Flag rtdb clean up
  this->fRTDB = true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Comm::UpdateChild()
{

  // Share robots World State
  Robot rws;
	CoachInfo cInfo;
  FormationInfo fInfo;
  KickCalibAppData kcAppData;
  KickCalibRobData kcRobData;
  GridView gv;
  
  int offset;
  unsigned int FINFO_Lifetime;
  if ( this->rtdbNum == 1 )
    offset = MAX_AGENTS;
  else
    offset = 0;
  
  // Fetch coach data
  DB_get_from(0 + offset, 0, COACH_INFO, (void*)&cInfo);
  FINFO_Lifetime = DB_get_from(0 + offset, 0, FORMATION_INFO, (void*)&fInfo);
  DB_get_from(0 + offset, 0, KICKCALIB_APP, (void*)&kcAppData);

//  fprintf(stderr,"csim_comm finfo size %d fid %d msg %s\n", sizeof(fInfo), fInfo.formationID, fInfo.setplayMessage);

  for(int ag1=1; ag1 < N_AGENTS; ag1++) {
    DB_put_in(ag1 + offset , 0, COACH_INFO, (void*)&cInfo,0);
    if(FINFO_Lifetime < 1000){    
	DB_put_in(ag1 + offset, 0, FORMATION_INFO, (void*)&fInfo,0);
    }
    DB_put_in(ag1 + offset, 0, KICKCALIB_APP, (void*)&kcAppData,0);

    int lt = DB_get_from(ag1 + offset, ag1, ROBOT_WS, (void*)&rws);
    int lt2 = DB_get_from(ag1 + offset, ag1, KICKCALIB_ROB, (void*)&kcRobData);
    //int lt3 = DB_get_from(ag1 + offset, ag1, GRIDVIEW, (void*)&gv);
    for(int ag2=0; ag2 < N_AGENTS; ag2++) {
      if(ag1!=ag2) {
        DB_put_in(ag2 + offset, ag1, ROBOT_WS, (void*)&rws, lt );
        DB_put_in(ag2 + offset, ag1, KICKCALIB_ROB, (void*)&kcRobData, lt2 );
        //DB_put_in(ag2 + offset, ag1, GRIDVIEW, (void*)&gv, lt3 );
      }
    } // end for( ag2 )
  } // end for( ag1 )

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Comm::FiniChild()
{
  if ( this->fRTDB ) {
    int _second_rtdb;
    if ( this->rtdbNum == 1)
      _second_rtdb = 1;
    else
      _second_rtdb = 0;
      
    DB_free_all(_second_rtdb);
  }
}
