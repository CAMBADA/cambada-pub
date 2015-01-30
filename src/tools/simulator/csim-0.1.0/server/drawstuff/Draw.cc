

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <iostream>
#include <iomanip>

#include <vector>
#include <map>
#include <string>
#include <sstream> 

#include "Simulator.hh"
#include "World.hh"
#include "Field.hh"

#include "Model.hh"
#include "Geom.hh"
#include "Body.hh"
#include "Shape.hh"

#include "SphereShape.hh"
#include "BoxShape.hh"
#include "CylinderShape.hh"

#include "drawstuff.h"
#include "Draw.hh"

#include "Vector2.hh"

#include <ode/ode.h>

// some constants

#define BALL_RADIUS 0.22	// ball radius
#define LENGTH 0.40		// box length & width
#define HEIGHT 0.4		// box height
#define MASS 20			// mass of box[i][j] = (i+1) * MASS
#define FORCE 0.1		// force applied to box[i][j] = (j+1) * FORCE
#define MU 0.1			// the global mu to use
#define GRAVITY 2.9		// the global gravity to use
#define N MAX_CLIENTS			// number of robots
#define MAX_CONTACTS 3		// maximum number of contact points per body

float *camsmooth;
float *camlooksmooth;

void static commands( int cmd );
void static drawstuff( int pause );

static const char* getObstacle(int id)
{
	return "obstable_0";
}

using namespace gazebo;

//////////////////////////////////////
///
Draw::Draw( ){
  this->drawThread = NULL;
}

//////////////////////////////////////
///
Draw::~Draw( ){
  
  if ( this->drawThread != NULL)
    delete this->drawThread;
}


void Draw::Load(XMLConfigNode *node){
  
  XMLConfigNode *renderNode = node->GetChildByNSPrefix("render");
  
  if ( renderNode == NULL )
    return;
  
  this->ballModelName = renderNode->GetString("ballModel", std::string(), 0);
  
}

//////////////////////////////////////
///
void Draw::Init(){

  this->viewmode = kFreeView;
  this->toggleMoveBall = 0;
  // keep a reference to the field.
  this->field = World::Instance()->GetField();
  
  Model* ballModel = World::Instance()->GetModelByName( this->ballModelName );
  
  if ( ballModel == NULL ){
    this->ball = NULL;
    gzerr(5) << "Ball model not found. Ball view disabled" << std::endl;
    
  }else{
    // I assume there is only one body, makes no sense otherwise.
    this->ball = ballModel->GetBody();
  }
  
  // Get Robots list
  std::vector<Model*> worldModels = World::Instance()->GetModels();
  std::vector<Model*>::iterator it= worldModels.begin();
  
  for( ; it != worldModels.end(); it++ ){
    
    //if (  (*it)->GetScopedName().start_with("robbie") )
      //this->robbies.push_back( (*it)->GetCanonicalBody()  );
  }
  
}

//////////////////////////////////////
///
void Draw::Fini(){
  
  if ( this->drawThread != NULL ){
    dsStop();
    this->drawThread->join(); 
  }
  
}

//////////////////////////////////////
///
void Draw::StartDraw( ){

  // Main loop thread
  dsSetSphereQuality(2);
  this->drawThread = new boost::thread( boost::bind( &Draw::DrawLoop, this )  );
  
}

//////////////////////////////////////
///
void Draw::DrawLoop(){

  dsFunctions fn;
  
  fn.start   = NULL;
  fn.command = commands;
  fn.stop    = NULL;
  fn.path_to_textures = "textures";
  fn.step = drawstuff;
  fn.version = DS_VERSION;
  
  camsmooth     = new float[3];
  camlooksmooth = new float[3];
  
  camsmooth[0] = 0;
  camlooksmooth[0] = 0;
  camsmooth[1] = 0;
  camlooksmooth[1] = 0;
  camsmooth[2] = 0;
  camlooksmooth[1] = 0;
  
  char* argv[] = { (char*)"-notex", (char*)"-noshadow",(char*)"-noshadows"};
  int argc = 3;
  dsSimulationLoop( argc, argv, 800, 600, &fn );
  
  gazebo::Simulator::Instance()->SetUserQuit();
}


///////////////////////
void Draw::DrawField()
{

  float lh = 0.01;
  float a[3];
  float b[3];
  
  /*
  // quick & dirty axis
  a[0] = a[1] = a[2] = 0;
  b[0] = a[1] = b[2] = 0;
  
  // X axis - RED
  b[0] = 1; 
  dsSetColor (1, 0, 0);
  dsDrawLine(a, b);
  // Y axis - GREEN
  b[0] = 0; b[1] = 1;
  dsSetColor (0, 1, 0);
  dsDrawLine(a, b);
  // Z axis - BLUE
  b[1] = 0; b[2] = 1;
  dsSetColor (0, 0, 1);
  dsDrawLine(a, b);
  */
  // White lines
  dsSetColor (1, 1, 1);

  std::vector<csim::LineSegment>::iterator iter = this->field->GetSegments()->begin();
  
  // not quite on the ground
  a[2] = b[2] = lh;
  
  for ( ; iter != this->field->GetSegments()->end(); ++iter )
  {
    a[0] = (*iter).pointA.x; a[1] = (*iter).pointA.y;
    b[0] = (*iter).pointB.x; b[1] = (*iter).pointB.y;
    
    dsDrawLine(a, b);
  }
  
  return;
}

void Draw::HandleView(){
  
  float camspeedpos = 0.2;
  float camspeedlook = 0.2;
  static float pos[3];
  static float look[4];

  switch ( this->viewmode ){
  
  case Draw::kFreeView: return; // Free 
  case Draw::kRobotView:
  {
        dReal dpos[3];
        dReal dlook[4];
        
        Model* m = World::Instance()->GetModelByName("robbie_2");
        Body* body = m->GetCanonicalBody();
        Pose3d bpose = body->GetAbsPose();
        
        dpos[0] = bpose.pos.x;
        dpos[1] = bpose.pos.y;
        dpos[2] = bpose.pos.z;

        dlook[0] = bpose.rot.u;
        dlook[1] = bpose.rot.x;
        dlook[2] = bpose.rot.y;
        dlook[3] = bpose.rot.z;
        
        pos[0] = (float) dpos[0];
        pos[1] = (float) dpos[1];
        pos[2] = (float) dpos[2] + 0.4;
        
        look[0] = 90.0 + RTOD( bpose.rot.GetYaw() ) ;
        look[1] = -25;
        look[2] = 0.0;
        look[3] = 0.0;

        //look[1] = -25;
        camspeedpos = 0.4;
        camspeedlook = 0.9;
        break;
  }        
  
  case Draw::kBallView:
  { 
    // Ball position
    Body* ball = Draw::Instance()->GetBall();
    if ( ball != NULL ){
      dReal dpos[3];
      dReal dlook[4];
      Pose3d bpose = ball->GetAbsPose();
      
      dpos[0] = bpose.pos.x;
      dpos[1] = bpose.pos.y;
      dpos[2] = bpose.pos.z;
      
      dlook[0] = bpose.rot.u;
      dlook[1] = bpose.rot.x;
      dlook[2] = bpose.rot.y;
      dlook[3] = bpose.rot.z;

      pos[0] = (float) dpos[0];
      pos[1] = (float) dpos[1];
      pos[2] = (float) +4.4;
      look[0] = (float) 180;
      look[1] = (float) 0;
      look[2] = (float) 0;

      look[3] = (float) 0;
      look[1] = -90;
      camspeedpos = 0.05;
      camspeedlook = 0.05;
      break;
    }
  }
  
  case Draw::kTopView:  
    pos[0] = 0; pos[1] = 0; 
    pos[2] = 10 * World::Instance()->GetField()->fieldLength /12.0;
    look[0] = 180.0; look[1] = -90.0; look[2] = 0.0; look[3] = 0.0;
    break;  
  }
      
  for (int i = 0; i < 3; i++){
    camsmooth[i] = camsmooth[i] * (1 - camspeedpos) + pos[i] * camspeedpos;
    camlooksmooth[i] = camlooksmooth[i] * (1 - camspeedlook) + look[i] * camspeedlook;      
  }
  
  dsSetViewpoint (camsmooth, camlooksmooth);
}

static void commands( int cmd ){
  
  switch( cmd ){
    case '?': { Draw::Instance()->viewmode = Draw::kFreeView;  return; }
    case '+': { Draw::Instance()->viewmode = Draw::kBallView;  return; }
    case '0': { Draw::Instance()->viewmode = Draw::kTopView;   return; }
    case '1': { Draw::Instance()->viewmode = Draw::kRobotView; return; }
    
    case 'W':
    case 'w':
    {
      Simulator* sim = Simulator::Instance();
      boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
     
      Body* ball = Draw::Instance()->GetBall();
      ball->SetEnabled(true);
      if( Draw::Instance()->toggleMoveBall == 0 )
	   ball->SetForce( Vector3(-1,0,0) );
      else if( Draw::Instance()->toggleMoveBall == 1 )
	  {
			Pose3d po = ball->GetAbsPose();
			po.pos += Vector3(-0.20,0.0,0.0);
			ball->SetAbsPose(po);
			ball->SetLinearVel( Vector3(0,0,0) );
			ball->SetAngularVel( Vector3(0,0,0) );
      }
      else
      {
		Model* m = World::Instance()->GetModelByName("obstacle_0");
		Body* body = m->GetCanonicalBody();
		Pose3d bpose = body->GetAbsPose();
		bpose.pos.x -= 0.1;
		body->SetAbsPose( bpose );
      }

      return;
    }
    
    
    case 'S':
    case 's':
    {
      Simulator* sim = Simulator::Instance();
      boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
      
      Body* ball = Draw::Instance()->GetBall();
      ball->SetEnabled(true);
      if(Draw::Instance()->toggleMoveBall == 0 )
	   ball->SetForce( Vector3(1,0,0) );
      else if( Draw::Instance()->toggleMoveBall == 1 )
	  {
			Pose3d po = ball->GetAbsPose();
			po.pos += Vector3(0.20,0.0,0.0);
			ball->SetAbsPose(po);
			ball->SetLinearVel( Vector3(0,0,0) );
			ball->SetAngularVel( Vector3(0,0,0) );
      }
      else
      {
		Model* m = World::Instance()->GetModelByName("obstacle_0");
		Body* body = m->GetCanonicalBody();
		Pose3d bpose = body->GetAbsPose();
		bpose.pos.x += 0.1;
		body->SetAbsPose( bpose );
      }

	return;
    }

    
    case 'D':
    case 'd':
    {
      Simulator* sim = Simulator::Instance();
      boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
      
      Body* ball = Draw::Instance()->GetBall();
      ball->SetEnabled(true);
      if(Draw::Instance()->toggleMoveBall == 0 )
	   ball->SetForce( Vector3(0,1,0) );
      else if( Draw::Instance()->toggleMoveBall == 1 )
	  {
			Pose3d po = ball->GetAbsPose();
			po.pos += Vector3(0.00,0.20,0.0);
			ball->SetAbsPose(po);
			ball->SetLinearVel( Vector3(0,0,0) );
			ball->SetAngularVel( Vector3(0,0,0) );
      }
      else
      {
		Model* m = World::Instance()->GetModelByName("obstacle_0");
		Body* body = m->GetCanonicalBody();
		Pose3d bpose = body->GetAbsPose();
		bpose.pos.y += 0.1;
		body->SetAbsPose( bpose );
      }

	return;
    }
    
    case 'A':
    case 'a':
    {
      Simulator* sim = Simulator::Instance();
      boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
      
      Body* ball = Draw::Instance()->GetBall();
      ball->SetEnabled(true);
      if(Draw::Instance()->toggleMoveBall == 0 )
			ball->SetForce( Vector3(0,-1,0) );
      else if( Draw::Instance()->toggleMoveBall == 1 )
	  {
			Pose3d po = ball->GetAbsPose();
			po.pos += Vector3(0.0,-0.20,0.0);
			ball->SetAbsPose(po);
			ball->SetLinearVel( Vector3(0,0,0) );
			ball->SetAngularVel( Vector3(0,0,0) );
	  }
      else
      {
		Model* m = World::Instance()->GetModelByName("obstacle_0");
		Body* body = m->GetCanonicalBody();
		Pose3d bpose = body->GetAbsPose();
		bpose.pos.y -= 0.1;
		body->SetAbsPose( bpose );
      }

	return;
    }
 
	case 'X':
    case 'x':
    {
      Simulator* sim = Simulator::Instance();
      boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
      
      Body* ball = Draw::Instance()->GetBall();
      ball->SetEnabled(true);
      ball->SetLinearVel( Vector3(0,0,0) );
      ball->SetAngularVel( Vector3(0,0,0) );
      return;
    }
    
    case 'B':
    case 'b':
    {
	  Draw::Instance()->toggleMoveBall++;
     
      if( Draw::Instance()->toggleMoveBall > 1 )
		Draw::Instance()->toggleMoveBall = 0;
      
      return;
    }

	case 'R':
    case 'r':
    {
      Simulator* sim = Simulator::Instance();
      boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
      
      Body* ball = Draw::Instance()->GetBall();
      ball->SetEnabled(true);
      ball->SetLinearVel( Vector3(0,0,0) );
      ball->SetAngularVel( Vector3(0,0,0) );
      Pose3d posi;
      posi.pos.z = 0.11;
	  ball->SetAbsPose(posi);
		return;
	} 
  }  
}

static void drawstuff( int pause ){

  Simulator* sim = Simulator::Instance();
  boost::recursive_mutex::scoped_lock lock( *sim->GetMRMutex() );
  
  Draw::Instance()->HandleView();
  Draw::Instance()->DrawField();
  
  // Ball position
  Body* ball = Draw::Instance()->GetBall();
  if ( ball ){
    Pose3d bpose = ball->GetAbsPose();
    std::stringstream info;
    
    info << "Ball : ( moveMode: " << Draw::Instance()->toggleMoveBall << " )   "<< std::setiosflags(std::ios::fixed) << std::setprecision(2)<< bpose.pos ;
    
    print(10,40, info.str().c_str()  );
  }
  
  switch ( Draw::Instance()->viewmode ){
    
    case Draw::kFreeView: print(10,20, "ViewMode: Free" );break;
    case Draw::kBallView: print(10,20, "ViewMode: Ball" );break;
    case Draw::kTopView: print(10,20, "ViewMode: Top" );break;
    case Draw::kRobotView: print(10,20, "ViewMode: Robot" );break;
    
  }
  
  std::vector<Model*> models = World::Instance()->GetModels();
  std::vector<Model*>::const_iterator m_iter = models.begin();
  
  const std::map< std::string, Body* > *bodies;
  const std::map<std::string, Geom*> *geoms;
  Model* m     = NULL;
  Geom* geom   = NULL;
  Shape* shape = NULL;
  
  dReal p[3];
  dReal r[12];
  dReal q[4];
  
  for ( ; m_iter != models.end(); m_iter++ ){
    
    m = *m_iter;
    if ( m->GetType().compare("empty") == 0 ) continue;
    
    // Find all Bodies
    bodies = m->GetBodies();
    std::map< std::string, Body* >::const_iterator b_iter = bodies->begin();
    
    for ( ; b_iter != bodies->end(); b_iter++ ){
      geoms = b_iter->second->GetGeoms();
      std::map<std::string, Geom*>::const_iterator g_iter = geoms->begin();
      
      for ( ; g_iter != geoms->end(); g_iter++ ){
        geom = g_iter->second;
        
        Vector3 rgb = geom->GetPigment();
        dsSetColor( rgb.x, rgb.y, rgb.z );
        
        shape= geom->GetShape();
        // Body Pose
        Pose3d pose = geom->GetAbsPose();
        
        p[0] = pose.pos.x;
        p[1] = pose.pos.y;
        p[2] = pose.pos.z;
        
        q[0] = pose.rot.u;
        q[1] = pose.rot.x;
        q[2] = pose.rot.y;
        q[3] = pose.rot.z;
        
        dQtoR( q, r );
        
        switch ( shape->GetType() ){
          case Shape::SPHERE:
            dsDrawSphere( p, r , ((SphereShape*)shape)->GetSize());
            break;
            
          case Shape::CYLINDER:
          {
            //cylinder dimension
            Vector2<double>  cyldim = ((CylinderShape*)shape)->GetSize();
            dsDrawCylinder( p, r, cyldim.y, cyldim.x );
          }
            break;
            
          case Shape::BOX:
          {
            float s[3];
            Vector3 v = ((BoxShape*)shape)->GetSize();
            s[0] = v.x; s[1] = v.y; s[2] = v.z;
            dsDrawBox( p, r, s);
          }
            break;
          
          default:
            break;
        }// end Switch geom
        
      } // end for geom..
      
    } // end for bodies
    
  } // end for models 
  
}

