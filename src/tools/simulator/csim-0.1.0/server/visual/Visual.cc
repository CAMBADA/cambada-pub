
#include <iostream>

#include <boost/thread/recursive_mutex.hpp>

#include "Model.hh"
#include "Simulator.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "Visual.hh"
#include "MainWindow.h"

#include <QApplication>

using namespace gazebo;
using namespace visual;

void VisualApp::Load(XMLConfigNode *node){
  
  XMLConfigNode *visualNode = node->GetChildByNSPrefix("visual");
  if ( visualNode == NULL )
    return;
    
  this->ballModelName = visualNode->GetString("ballModel", std::string(), 0);
  
}

void VisualApp::Init( ){

  if (this->enabled == false)
    return;

  static int argc = 1;
  static char* argv[] = { (char*)"csim" };
  
  Model* ballModel = World::Instance()->GetModelByName( this->ballModelName );
  
  if ( ballModel == NULL ){
    this->ball = NULL;
    gzerr(5) << "Ball model not found. Ball view disabled" << std::endl;
  }else{
    // I assume there is only one body, makes no sense otherwise.
    this->ball = ballModel->GetBody();
  }
  
  this->QTapp = new QApplication(argc, argv);
  this->mw    = new MainWindow();
  
  this->mw->show();
  this->enabled = true;
  
}

void VisualApp::Fini( ){

}

void VisualApp::SetEnabled(bool enabled){
  this->enabled = enabled;
}

void VisualApp::ProcessEvents( ){
  
  if ( this->enabled == false)
    return;
    
  // maybe try try_lock here instead
  boost::recursive_mutex::scoped_lock lock(
        *Simulator::Instance()->GetMRMutex());

  if ( this->QTapp->hasPendingEvents() )
    this->QTapp->processEvents();
    
  // Render
  this->mw->updateVisual();
  
}

bool VisualApp::UserQuit(){
  
  if ( this->enabled == false)
    return false;
    
  return this->mw->UserClosedWindow();
}

/*********** Private functions ***********/

// Constructor
VisualApp::VisualApp( ){
  QTapp = NULL;
  mw    = NULL;
  ball  = NULL;
}

// Destructor
VisualApp::~VisualApp( ){

}
