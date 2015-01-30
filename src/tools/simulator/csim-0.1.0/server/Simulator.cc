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
/* Desc: The Simulator; Top level managing object
 * Author: Jordi Polo
 * Date: 3 Jan 2008
 *
 * Modified by: Eurico Pedrosa <efp@ua.p>
 * Date: 10 Fev 2010
 *
 * Modification Notes
 *
 *  The modifications presented by me, have the purpose of
 *  removing the 'rendering' and 'gui' modules from the code base.
 *  The reasons behind this decision are simple, allow gazebo to run
 *  on computers with less gpu capabilities and lessen the the coupling
 *  between simulation and visualization.
 *
 */

#include <assert.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "Timer.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Model.hh"
#include "Entity.hh"
#include "World.hh"
#include "XMLConfig.hh"
#include "GazeboConfig.hh"
#include "gazebo.h"
#include "PhysicsEngine.hh"
#include "GazeboMessage.hh"
#include "Global.hh"
#include "Referee.hh"
#include "Visual.hh"
#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Simulator::Simulator()
: xmlFile(NULL),
  gazeboConfig(NULL),
  loaded(false),
  pause(false),
  simTime(0.0),
  pauseTime(0.0),
  startTime(0.0),
  physicsUpdates(0),
  checkpoint(0.0),
  stepInc(false),
  userQuit(false),
  physicsEnabled(true),
  timeout(-1),
  selectedEntity(NULL),
  selectedBody(NULL)
{
  this->mutex = new boost::recursive_mutex();
  this->startTime = this->GetWallTime();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Simulator::~Simulator()
{

  if (this->gazeboConfig)
  {
    delete this->gazeboConfig;
    this->gazeboConfig = NULL;
  }

  if (this->xmlFile)
  {
    delete this->xmlFile;
    this->xmlFile = NULL;
  }

  if (this->mutex)
  {
    delete this->mutex;
    this->mutex = NULL;
  }


  if (this->physicsThread)
  {
    delete this->physicsThread;
    this->physicsThread = NULL;
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Closes the Simulator and frees everything
void Simulator::Close()
{
  if (!this->loaded)
    return;

  gazebo::World::Instance()->Close();
}

////////////////////////////////////////////////////////////////////////////////
/// Load the world configuration file
/// Any error that reach this level must make the simulator exit
void Simulator::Load(const std::string &worldFileName, unsigned int serverId )
{
  this->state = LOAD;

  if (loaded)
  {
    this->Close();
    loaded=false;
  }

  // Load the world file
  this->xmlFile=new gazebo::XMLConfig();
  try
  {
    this->xmlFile->Load(worldFileName);
  }
  catch (GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  XMLConfigNode *rootNode(xmlFile->GetRootNode());

  // Load the messaging system
  gazebo::GazeboMessage::Instance()->Load(rootNode);

  // load the configuration options 
  this->gazeboConfig=new gazebo::GazeboConfig();
  try
  {
    this->gazeboConfig->Load();
  }
  catch (GazeboError e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  //Create the world
  try
  {
    gazebo::World::Instance()->Load(rootNode, serverId);
    //if ( optRenderEngineEnabled )
      visual::VisualApp::Instance()->Load( rootNode );
      
    gazebo::Referee::Instance()->Load( rootNode );
    
  }
  catch (GazeboError e)
  {
    gzthrow("Failed to load the World\n"  << e);
  }

  this->loaded=true;
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the simulation
void Simulator::Init()
{
  this->state = INIT;

  //Initialize the world
  try
  {
    gazebo::World::Instance()->Init();
  }
  catch (GazeboError e)
  {
    gzthrow("Failed to Initialize the World\n"  << e);
  }

  // This is not a debug line. This is useful for external programs that 
  // launch Gazebo and wait till it is ready   
  std::cout << "Gazebo successfully initialized" << std::endl
            << "control+c to Quit" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
/// Save the world configuration file
void Simulator::Save(const std::string& filename)
{
  std::fstream output;

  output.open(filename.c_str(), std::ios::out);

  // Write out the xml header
  output << "<?xml version=\"1.0\"?>\n";
  output << "<gazebo:world\n\
    xmlns:xi=\"http://www.w3.org/2001/XInclude\"\n\
    xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\"\n\
    xmlns:model=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#model\"\n\
    xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"\n\
    xmlns:param=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#param\"\n\
    xmlns:body=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#body\"\n\
    xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\"\n\
    xmlns:joint=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#joint\"\n\
    xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"\n\
    xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"\n\
    xmlns:physics=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#physics\">\n\n";

  std::string prefix = "  ";

  if (output.is_open())
  {
    GazeboMessage::Instance()->Save(prefix, output);
    output << "\n";

    World::Instance()->GetPhysicsEngine()->Save(prefix, output);
    output << "\n";

    World::Instance()->Save(prefix, output);
    output << "\n";

    output << "</gazebo:world>\n";
    output.close();
  }
  else
  {
    gzerr(0) << "Unable to save XML file to file[" << filename << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the simulation
void Simulator::Fini( )
{
  gazebo::World::Instance()->Fini();

  this->Close();
}

////////////////////////////////////////////////////////////////////////////////
/// Main simulation loop, when this loop ends the simulation finish
void Simulator::MainLoop()
{
  this->state = RUN;

  //DiagnosticTimer timer("--------------------------- START Simulator::MainLoop() --------------------------");
  Time currTime = 0;
  Time lastTime = 0;
  struct timespec timeSpec;
  double freq = 50.0; // used to be 80

  this->physicsThread = new boost::thread( 
                         boost::bind(&Simulator::PhysicsLoop, this));


  // Update the gui
  while (!this->userQuit)
  {
    currTime = this->GetWallTime();
    if ( currTime - lastTime > 1.0/freq)
    {
      lastTime = this->GetWallTime();
      
      visual::VisualApp::Instance()->ProcessEvents();
      this->userQuit = ( this->userQuit || visual::VisualApp::Instance()->UserQuit() );
      
      currTime = this->GetWallTime();

      World::Instance()->ProcessEntitiesToLoad();

      if (currTime - lastTime < 1/freq)
      {
        Time sleepTime = ( Time(1.0/freq) - (currTime - lastTime));
        timeSpec.tv_sec = sleepTime.sec;
        timeSpec.tv_nsec = sleepTime.nsec;

        nanosleep(&timeSpec, NULL);
      }
    }
    else
    {
      Time sleepTime = ( Time(1.0/freq) - (currTime - lastTime));
      timeSpec.tv_sec = sleepTime.sec;
      timeSpec.tv_nsec = sleepTime.nsec;
      nanosleep(&timeSpec, NULL);
    }
  }

  this->physicsThread->join();
}

////////////////////////////////////////////////////////////////////////////////
/// Gets local configuration for this computer
GazeboConfig *Simulator::GetGazeboConfig() const
{
  return this->gazeboConfig;
}


////////////////////////////////////////////////////////////////////////////////
// Return when this simulator is paused
bool Simulator::IsPaused() const
{
  return this->pause;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether the simulation is paused
void Simulator::SetPaused(bool p)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if (this->pause == p)
    return;

  this->pauseSignal(p);
  this->pause = p;
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulation time
gazebo::Time Simulator::GetSimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the sim time
void Simulator::SetSimTime(Time t)
{
  this->simTime = t;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pause time
gazebo::Time Simulator::GetPauseTime() const
{
  return this->pauseTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the start time
gazebo::Time Simulator::GetStartTime() const
{
  return this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the real time (elapsed time)
gazebo::Time Simulator::GetRealTime() const
{
  return this->GetWallTime() - this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the wall clock time
gazebo::Time Simulator::GetWallTime() const
{
  Time t;
  t.SetToWallTime();
  return t;
}

////////////////////////////////////////////////////////////////////////////////
// Set the user quit flag
void Simulator::SetUserQuit()
{
  //  this->Save("test.xml");
  this->userQuit = true;
}

////////////////////////////////////////////////////////////////////////////////
bool Simulator::GetStepInc() const
{
  return this->stepInc;
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::SetStepInc(bool step)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->stepInc = step;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the length of time the simulation should run.
void Simulator::SetTimeout(double time)
{
  this->timeout = time;
}

////////////////////////////////////////////////////////////////////////////////
// Set the physics enabled/disabled
void Simulator::SetPhysicsEnabled( bool enabled )
{
  this->physicsEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
// Get the physics enabled/disabled
bool Simulator::GetPhysicsEnabled() const
{
  return this->physicsEnabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the selected entity
void Simulator::SetSelectedEntity( Entity *ent )
{
    this->selectedEntity = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the selected entity
Entity *Simulator::GetSelectedEntity() const
{
  return this->selectedEntity;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model that contains the entity
Model *Simulator::GetParentModel( Entity *entity ) const
{
  Model *model = NULL;

  if (entity == NULL)
    return NULL;

  do 
  {
    model = dynamic_cast<Model*>(entity);
    entity = entity->GetParent();
  } while (model == NULL);

  return model;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the body that contains the entity
Body *Simulator::GetParentBody( Entity *entity ) const
{
  Body *body = NULL;

  if (entity == NULL)
    return NULL;

  do 
  {
    body = dynamic_cast<Body*>(entity);
    entity = entity->GetParent();
  } while (body == NULL);

  return body;
}

////////////////////////////////////////////////////////////////////////////////
/// Function to run physics. Used by physicsThread
void Simulator::PhysicsLoop()
{
  World *world     = World::Instance();
  Referee* referee = Referee::Instance();

  world->GetPhysicsEngine()->InitForThread();

  Time step = world->GetPhysicsEngine()->GetStepTime();
  double physicsUpdateRate = world->GetPhysicsEngine()->GetUpdateRate();
  Time physicsUpdatePeriod = 1.0 / physicsUpdateRate;

  bool userStepped;
  Time diffTime;
  Time currTime;
  Time lastTime = this->GetRealTime();
  struct timespec req, rem;


  while (!this->userQuit)
  {
    //DiagnosticTimer timer("PhysicsLoop Timer ");

    currTime = this->GetRealTime();
    userStepped = false;

    // Update the physics engine
    //if (!this->GetUserPause()  && !this->IsPaused() ||
    //   (this->GetUserPause() && this->GetUserStepInc()))
    if (!this->IsPaused() || this->GetStepInc())
    {
      this->simTime += step;

      if (this->GetStepInc())
          userStepped = true;
    }
    else
      this->pauseTime += step;

    lastTime = this->GetRealTime();

    {
      boost::recursive_mutex::scoped_lock lock(*this->mutex);
      world->Update();
      //referee->ApplyRules();
    }

    currTime = this->GetRealTime();

    // Set a default sleep time
    req.tv_sec  = 0;
    req.tv_nsec = 10000;

    // If the physicsUpdateRate < 0, then we should try to match the
    // update rate to real time
    if ( physicsUpdateRate < 0 &&
        (this->GetSimTime() + this->GetPauseTime()) > 
        this->GetRealTime()) 
    {
      diffTime = (this->GetSimTime() + this->GetPauseTime()) - 
                 this->GetRealTime();
      req.tv_sec  = diffTime.sec;
      req.tv_nsec = diffTime.nsec;
    }
    // Otherwise try to match the update rate to the one specified in
    // the xml file
    else if (physicsUpdateRate > 0 && 
        currTime - lastTime < physicsUpdatePeriod)
    {
      diffTime = physicsUpdatePeriod - (currTime - lastTime);

      req.tv_sec  = diffTime.sec;
      req.tv_nsec = diffTime.nsec;
    }

    nanosleep(&req, &rem);

    {
      //DiagnosticTimer timer("PhysicsLoop UpdateSimIfaces ");

      // Process all incoming messages from simiface
      world->UpdateSimulationIface();
    }

    if (this->timeout > 0 && this->GetRealTime() > this->timeout)
    {
      this->userQuit = true;
      break;
    }

    if (userStepped)
    {
      this->SetStepInc(false);
      this->SetPaused(true);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the simulator mutex
boost::recursive_mutex *Simulator::GetMRMutex()
{
  return this->mutex;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the state of the simulation
Simulator::State Simulator::GetState() const
{
  return this->state;
}
