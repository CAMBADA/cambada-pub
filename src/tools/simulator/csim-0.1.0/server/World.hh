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
/* Desc: The world; all models are collected here
 * Author: Andrew Howard and Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id: World.hh 8480 2009-12-22 18:08:45Z natepak $
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

#ifndef WORLD_HH
#define WORLD_HH

#include <iostream>
#include <vector>
#include <deque>
#include <map>
#include <string>

#include <boost/tuple/tuple.hpp>
#include <boost/signal.hpp>

#ifdef USE_THREADPOOL
#include "boost/threadpool.hpp"
#include "boost/thread/mutex.hpp"
#endif

#include "SingletonT.hh"
#include "Vector2.hh"
#include "Vector3.hh"
#include "Pose3d.hh"
#include "Field.hh"
#include "Entity.hh"
#include "Global.hh"
#include "Timer.hh"

#include "json/json.h"

namespace gazebo
{
/// \addtogroup gazebo_server
/// \brief The World
/// \{

// Forward declarations
  class Server;
  class SimulationIface;
  class Model;
  class Geom;
  class PhysicsEngine;
  class XMLConfigNode;
  class Factory;
  class WorldState;
  class Timer;
  class Time;
   
/// \brief The World
/*
 * The world class keps a list of all models, handles loading and saving,
 * object dynamics and collision detection for contact joints
 */
class World : public SingletonT<World>
{
  /// Private constructor
  private: World();

  /// Private destructor
  private: ~World();

  ///Closes the present world, frees the resources and closes the interfaces   
  public: void Close();

  /// Load the world
  /// \param node XMLConfig node point
  /// \param serverId Id of the gazebo server
  public: void Load(XMLConfigNode *rootNode, unsigned int serverId);

  /// Save the world
  /// \param stream Output stream
  public: void Save(std::string &prefix, std::ostream &stream);

  /// Initialize the world
  public: void Init();

  /// Update the world
  public: void Update();

  /// Finilize the world
  public: void Fini();

  /// Retun the libgazebo server
  /// \return Pointer the the libgazebo server
  public: Server *GetGzServer() const;

  /// Return the physics engine
  /// \return Pointer to the physics engine
  public: PhysicsEngine *GetPhysicsEngine() const;

  /// Get the simulation time
  /// \return The simulation time
  public: double GetSimTime() const;

  /// Get the pause time
  /// \return The pause time
  public: double GetPauseTime() const;

  /// Get the start time
  /// \return The start time
  public: double GetStartTime() const;

  /// Get the real time (elapsed time)
  /// \return The real time
  public: double GetRealTime() const;

  /// \brief Get the wall clock time
  /// \return The wall clock time
  public: double GetWallTime() const;

  /// \brief Load all entities
  /// \param node XMLConfg node pointer
  /// \param parent Parent of the model to load
  /// \param removeDuplicate Remove existing model of same name
  public: void LoadEntities(XMLConfigNode *node, Model *parent, 
                            bool removeDuplicate);

  /// \brief Insert an entity into the world. This function pushes the model
  //  (encoded as an XML string) onto a list. The Graphics Thread will then
  //  call the ProcessEntitiesToLoad function to actually make the new
  //  entities. This Producer-Consumer model is necessary for thread safety.
  public: void InsertEntity(std::string xmlString);

  /// \brief Load all the entities that have been queued
  public: void ProcessEntitiesToLoad();

  /// \brief Delete an entity by name
  /// \param name The name of the entity to delete
  public: void DeleteEntity(const char *name);

  /// \brief Get a pointer to a model based on a name
  public: Model *GetModelByName(std::string modelName);

  /// \brief Get an iterator over the models
  public: std::vector<Model*> &GetModels();

  /// \brief Reset the simulation to the initial settings
  public: void Reset();

  /// \brief register a geom
  public: void RegisterGeom(Geom *geom);

  /// \brief Register a body 
  public: void RegisterBody(Body *body);

  /// \brief Goto a position in time
  public: void GotoTime(double pos);

  /// \brief Save the state of the world
  public: void SaveState();

  /// \brief Return a json object with the state of all dynamic models.
  public: Json::Value GetDynamicModelsState();

  /// \brief Set the state of the dynamic models given a json root.
  public: void SetDynamicModelsState(const Json::Value& root);

  /// \brief Save the state of a specific model
  public: void SaveModelState(Model* model);
  /// \brief Restore the state of the world
  public: void RestoreState();

  /// \breif Set the state of the world to the pos pointed to by the iterator
  public: void SetState(std::deque<WorldState>::iterator iter);
  

  /// \brief Pause callback
  private: void PauseSlot(bool p);

  /// \brief Load a model
  /// \param node Pointer to the XMLConfig node
  /// \param parent The parent model
  /// \param removeDuplicate Remove existing model of same name
  /// \return The model that was created
  private: Model *LoadModel(XMLConfigNode *node, Model *parent, bool removeDuplicate);

  /// \brief Set the model pose and the pose of it's attached children 
  /// \param model The model to set
  /// \param pose The pose to set the model to
  public: void  SetModelPose(Model *model, Pose3d pose);
  public: Model* GetModelAt(const Vector3& pos);
  /// \brief Update the simulation iface
  public: void UpdateSimulationIface();

  /// \brief Connect a boost::slot the the add entity signal
  public: template<typename T>
          void ConnectAddEntitySignal( T subscriber )
          {
            addEntitySignal.connect(subscriber);
          }

  /// \brief Get the names of interfaces defined in the tree of a model
  private: void GetInterfaceNames(Entity* m, std::vector<std::string>& list);

  /// Pointer the physics engine
  private: PhysicsEngine *physicsEngine;
  
  /// List of all the models
  private: std::vector< Model* > models;

  /// List of all the registered geometries
  private: std::vector< Geom* > geometries;

  /// List of all the registered bodies
  private: std::vector< Body* > bodies;

  /// List of models to delete from the world
  private: std::vector< Model* > toDeleteModels;

  private: std::vector< std::string > toLoadEntities;
  
  // return field
  public: csim::Field* GetField(){ return this->field; };
  // World field
  private: csim::Field* field;

  /// Simulator control interface
  private: Server *server;

  /// Simulation interface
  private: SimulationIface *simIface;

  private: Factory *factory;

  /// Length of time to run before receiving a "go" command
  private: Time simPauseTime;

  /// List of all the parameters
  protected: std::vector<Param*> parameters;

  private: ParamT<int>* threadsP;

#ifdef USE_THREADPOOL
  public: boost::threadpool::pool* threadPool;
#endif

  private: friend class DestroyerT<World>;
  private: friend class SingletonT<World>;

  private: boost::signal<void (Entity*)> addEntitySignal;

  // SAVE WORLD STATE
  // TODO: Create structure
  private: std::map<std::string, Pose3d> modelAbsPose;
  private: std::map<std::string, Vector3> modelLinearVel;
  private: std::map<std::string, Vector3> modelLinearAccel;
  private: std::map<std::string, Vector3> modelAngularVel;
  private: std::map<std::string, Vector3> modelAngularAccel;
  

  private: std::deque<WorldState> worldStates;
  private: std::deque<WorldState>::iterator worldStatesInsertIter;
  private: std::deque<WorldState>::iterator worldStatesEndIter;
  private: std::deque<WorldState>::iterator worldStatesCurrentIter;

  private: Timer saveStateTimer;
  private: ParamT<Time> *saveStateTimeoutP;
  private: ParamT<unsigned int> *saveStateBufferSizeP;
};

class WorldState
{
  public: std::map<std::string, Pose3d> modelPoses;
  public: std::map<std::string, Pose3d> bodyPoses;
  public: std::map<std::string, Pose3d> geomPoses;
};

/// \}
}

#endif
