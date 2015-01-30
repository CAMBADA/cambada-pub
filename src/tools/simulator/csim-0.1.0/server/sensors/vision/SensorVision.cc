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
 *  @Date   30 Jan 2010
 *  @Desc   Vision sensor
 *
 */

#include <sstream>
#include "XMLConfig.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "Simulator.hh"
#include "World.hh"
#include "Model.hh"
#include "Body.hh"
#include "Pose3d.hh"
#include "Rand.hh"

#include <vector>
#include <list>
#include <cmath>

#include "Timer.hh"
#include "SensorFactory.hh"
#include "SensorVision.hh"


// CAMBADA include
#include "rtdb_sim.h"
#include "pman.h"
#include "pmandefs.h"
#include "WorldStateDefs.h"
#include "Vec.h"
//#include "Integrator.h"

#include "geometry.h"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("vision", SensorVision);


static int dummyShed(int pint){ /* empty */ };

static bool comparePosByAngle(Vec first, Vec second ){

  float a1 = std::atan2(first.y, first.x );
  float a2 = std::atan2(second.y, second.x );

  if ( a1 < 0.0 ) a1 = 2*M_PI + a1;
  if ( a2 < 0.0 ) a2 = 2*M_PI + a2;

  return a1 < a2;

}


//////////////////////////////////////////////////////////////////////////////
// Constructor
SensorVision::SensorVision(Body *body)
    : Sensor(body)
{
  this->omniQueue = NULL;
  this->typeName  = "vision";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SensorVision::~SensorVision()
{
  
  if ( this->omniQueue != NULL)
    delete omniQueue;
}

//////////////////////////////////////////////////////////////////////////////
// Load the vision
void SensorVision::LoadChild( XMLConfigNode *node )
{
  this->ballModelName = node->GetString("ballModel", std::string(), 1);
  this->ballRadius    = node->GetFloat("ballRadius", 0.11, 0);
  this->maxWhitePoints= node->GetInt("maxWhitePoints", 500, 0);
  this->seeDistance   = node->GetFloat("seeDistance", 5, 0);
  this->height        = node->GetFloat("height", 0.8, 0);
  this->cycleDelay    = node->GetInt("delay", 2, 0);
  this->radialSensors = node->GetInt("sensors", 20, 0);
  this->radialPasses  = node->GetInt("passes", 2, 0);

  this->noisyWhite    = node->GetBool("noisyWhite",     false, 0);
  this->noisyBall     = node->GetBool("noisyBall",      false, 0);
  this->noisyObstacles= node->GetBool("noisyObstacles", false, 0);

  this->PMANConfigFile = node->GetFilename("PMANConf", std::string(), 0);
  if ( this->PMANConfigFile == "" ) this->PMANConfigFile = std::string("../config/pman.conf"); 

  this->selfID = this->GetParentModel()->GetSelfID();
  
  // never more than MAX_POINTS
  if ( this->maxWhitePoints > MAX_POINTS )
    this->maxWhitePoints = MAX_POINTS;
  
  //
  if ( this->radialSensors <= 0 )
    this->radialSensors = 20;
    
  if ( this->radialPasses <= 0 )
    this->radialPasses = 2;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize the camera
void SensorVision::InitChild()
{ 
  
  // Withou ID define it will not init
  if ( this->selfID < 1 ) { 
    gzerr(0) << "Agent ID is missing, Vision will not update\n";
    return;
  }
  
  Model* ballModel = World::Instance()->GetModelByName( this->ballModelName );
  
  if ( ballModel == NULL ){
    this->ball = NULL;
    gzerr(0) << "Ball model not found. Ball detection disabled" << std::endl;
    
  }else{
    // I assume there is only one body, makes no sense otherwise.
    this->ball = ballModel->GetBody();
  }
  
  // Keep a reference of the field
  this->field = World::Instance()->GetField();
  // Get all black models
  this->FillObstacleList();
  
  // prepare delay queue (fifo)
  if ( this->cycleDelay > 0 ){
    this->visionInfo.lines.nPoints = 0;
    this->visionInfo.obstacles.nPoints = 0;
	this->visionInfo.nBalls = 0;
    this->omniQueue = new std::deque<VisionInfo>(this->cycleDelay , this->visionInfo );
  }else{
    this->omniQueue = NULL;
  }
  
  // Init PMAN
  FILE *fpPman = NULL;
	int pmanstat, pprio;

  this->fPMAN = false;
	// PMAN initializations
  pmanstat = PMAN_init2(SHMEM_OCAM_PMAN_KEY + 2*this->selfID, SEM_OCAM_PMAN_KEY + 2*this->selfID,
                       (void *)dummyShed, sizeof(pprio), PMAN_NEW);

	if( pmanstat < 0 ) {
	 /* 
	  * PMAN_init return values:
	  *    0 : Success
    *   -1 : Error creating shared memory region 
    *   -2 : Failed to create semaphore 
    *   -3 : Error setting the priority
    *   -4 : Error allocating memory (QoS data)
    */
    switch ( -pmanstat ){
      case 1: gzthrow("PMAN_init: Error creating shared memory region");
      case 2: gzthrow("PMAN_init: Failed to create semaphore ");
      case 3: gzthrow("PMAN_init: Error setting the priority");
      case 4: gzthrow("PMAN_init: Error allocating memory (QoS data)");
    }
		
	} else {
    
    this->fPMAN = true;
    pman_save_ids( this->selfID );
    
		if(!(fpPman = fopen( this->PMANConfigFile.c_str() , "r"))) {
  		gzthrow("Couldn't open PMAN configuration file");
		}
		else {
			do {
				char pname[32];
				int pper, ppha, pddln, pprio;
				// process- {name, period, phase, deadline, priority}
				int nv = fscanf(fpPman,"%s %d %d %d %d",pname,&pper,&ppha,&pddln,&pprio);
				if( nv == 5 ) {
					PMAN_procadd(pname,PMAN_NOPID,pper,ppha,pddln,&pprio,sizeof(pprio));
				}
			} while( !feof(fpPman) );

			fclose(fpPman);
		}
	}
	// end PAMN init
	
}

//////////////////////////////////////////////////////////////////////////////
// Finalize the camera
void SensorVision::FiniChild(){
  
  if ( this->fPMAN ){
    pman_switch_id( this->selfID );
    PMAN_close(PMAN_CLFREE);
  }

}

//////////////////////////////////////////////////////////////////////////////
// Update the drawing
void SensorVision::UpdateChild()
{
  DiagnosticTimer dt("Vision update");
  dt.Start();
  
  // Withour ID define it will not update
  if ( this->selfID < 1 ) return;

  // Detect obstacles before anything else
  // as it is needed to do Ball and White-points occlusion.
  this->DetectOcclusions();
  this->DetectObstacle();
  this->DetectBall();
  this->DetectWhite();
  
  if ( this->omniQueue != NULL ){
    
    omniQueue->push_back( this->visionInfo );
    this->visionInfo = omniQueue->front();
    omniQueue->pop_front();
  }
  
  // Send data to RTDB
  DB_put_in( this->selfID, this->selfID, VISION_INFO, &(this->visionInfo), 0 );
  
  // awake Agent
  pman_switch_id( this->selfID );
  PMAN_tick();
}

//////////////////////////////////////////////////////////////////////////////
// Detect occlusions
void SensorVision::DetectOcclusions(){

  std::vector<Body*>::iterator it = this->obstacleList.begin();
  
  Vector3 aabb_min, aabb_max;
  Vector3 location;
  float radius;
  
  Pose3d opose; // Obstacle pose
  Pose3d rpose; // Relative pose
  // Parent pose
  Pose3d ppose = this->body->GetAbsPose();
  ppose.pos.z = 0;
  
  this->occlusion.clear();
  for( ; it != this->obstacleList.end(); it++){
    
    opose = (*it)->GetAbsPose();
    opose.pos.z = 0; // ground level ...
    // Get the relative position
    rpose = opose - ppose;
    
    // I don't know the dimensions of the obstacle but i can
    // assume that every obstacle has the shape of a cylinder,
    // therefore i can use the axis-align bounding box to obtain
    // the obstacle radius.
    (*it)->GetBoundingBox( aabb_min, aabb_max );
    radius = (aabb_max.x - aabb_min.x) * 0.5;
    radius = (aabb_max.y - aabb_min.y) * 0.5 > radius ? 
             (aabb_max.y - aabb_min.y) * 0.5 : radius ;
    
    // Subtract obstacle radius
    location = rpose.pos * ( (rpose.pos.GetLength() - radius) / rpose.pos.GetLength() );
    // Find left and right limits using circle intersection
    Circle obstCircle( Vec(rpose.pos.x, rpose.pos.y), (double) radius );
    location = rpose.pos * 0.5; // half way beetwen origin and obstacle center
    Circle usefullCircle( Vec(location.x, location.y), location.GetLength() );
    
    std::vector<Vec> limitPoints = intersect( obstCircle, usefullCircle );
    // limitPoints will always be size 2, unless something wrong happens
    // I think the size should be asserted..
    
    // Save Occlusion area of the obstacle.
    float a,b;
    a = std::atan2( limitPoints[1].x, limitPoints[1].y );
    b = std::atan2( limitPoints[0].x, limitPoints[0].y );
    
    OcclusionArea oa( rpose.pos.GetLength(), aabb_max.z - aabb_min.z, BetweenAngles(a,b) );
    this->occlusion.push_back( oa );
  }  

}

//////////////////////////////////////////////////////////////////////////////
// Detect ball
void SensorVision::DetectBall(){
  
  if ( this->ball == NULL )
    return;
  
  // Front Vision does not exist
  //this->frontVisionInfo.ball.cyclesNotVisible = 10000;
  
  // Parent pose
  Pose3d ppose = this->body->GetAbsPose();
  // Ball pose
  Pose3d bpose = this->ball->GetAbsPose();
  // Save ball distance from the ground 
  float ballAltitude = std::max(bpose.pos.z - this->ballRadius, 0.0);
  
  if ( ballAltitude >= this->height ){
    // Ball above visual detection
    this->visionInfo.nBalls = 0;
    return;
  }
  
  // make it ground level
  ppose.pos.z  = 0.0f;
  bpose.pos.z  = 0.0f;
  
  // Resolve ball position
  Pose3d ballRelPosition = bpose - ppose;
  float distance = ballRelPosition.pos.GetLength();
  
  if ( (this->height-ballAltitude) < (this->height*distance) / this->seeDistance ){
    // Ball out of visual reach
    this->visionInfo.nBalls = 0;
    return;
  }
  
  // Check for occlusion
  bool ballNotVisible;
  Vector3 pos = Vector3( ballRelPosition.pos.x, ballRelPosition.pos.y, ballAltitude );
  ballNotVisible = this->OnOcclusionArea( pos );
  
  if ( ballNotVisible ){
    // Ball out of visual reach
    this->visionInfo.nBalls = 0;
    return;
  }

  // Update ball relative position by projecting the ball to the ground
  float groundDistance = (this->height * distance) / ( this->height - ballAltitude );
  ballRelPosition.pos *= (groundDistance / distance) ;
  
  // noisy or not...
  if (this->noisyBall)
      this->NoisyPosition(ballRelPosition.pos);

  // Update vision info -- Ball section
  this->visionInfo.ball[0].position.x = ballRelPosition.pos.x;
  this->visionInfo.ball[0].position.y = ballRelPosition.pos.y;
  this->visionInfo.nBalls = 1;

/*  std::cout << "BALL " << Simulator::Instance()->GetSimTime().Double()
            << " "     << (ballRelPosition + ppose).pos
            << std::endl;
*/
}

//////////////////////////////////////////////////////////////////////////////
// Detect white points
void SensorVision::DetectWhite(){

  unsigned int i;
  int k;
        
  // Resolve White Points...
  float angle;
  float angleStep;
  unsigned int   totalWhite;
  std::vector<csim::Point> allWhite;
  
  // Parent pose
  Pose3d ppose = this->body->GetAbsPose();
  ppose.pos.z = 0;

  csim::Point robotPosition( ppose.pos.x , ppose.pos.y);
  
  // Radial sensors 
  totalWhite = 0;
  angleStep = M_PI / (float) this->radialSensors;
  
  for ( k = 0; k < this->radialPasses ; k++ )
  for ( angle = k*angleStep ; angle < (M_PI - angleStep*0.5) ; angle += angleStep*this->radialPasses ){

    csim::Point prot( std::cos(angle), std::sin(angle) );
    csim::Ray ray( robotPosition, robotPosition + prot );
    std::vector<csim::Point> white = this->field->Intersect( ray );
    
    // Check for occlusion and max distance
    std::vector<csim::Point>::iterator it = white.begin();
    for( ; it != white.end(); it++ ){
      
      Vector3 pos = Vector3( (*it).x, (*it).y, 0.0 );
      //Vector3 rel = pos - Vector3( robotPosition.x, robotPosition.y, 0 );
      // Calculate points relative to robot position..
      Pose3d white( pos, Quatern() );
      Pose3d relPosition = white - ppose;

      if( relPosition.pos.GetLength() >= 12.0 )
          continue; // too far away

      if ( this->OnOcclusionArea( relPosition.pos )  )
        continue; // White point is not visible
      
      allWhite.push_back( *it );
      totalWhite++;
      
      if ( totalWhite == this->maxWhitePoints )
        break;
    }
    
    if ( totalWhite == this->maxWhitePoints )
      break;

  } // for "angle"
  
  for ( i = 0; i < totalWhite; i++ ){
    
    // Calculate points relative to robot position..
    Pose3d white( Vector3( allWhite[i].x  , allWhite[i].y , 0.0), Quatern() );
    Pose3d relPosition = white - ppose;

    // noisy or not...
    if (this->noisyWhite)
        this->NoisyPosition(relPosition.pos, true);

    this->visionInfo.lines.point[i] = Vec( relPosition.pos.x * 1000, relPosition.pos.y * 1000 );
  }
  
  this->visionInfo.lines.nPoints = totalWhite;
}

//////////////////////////////////////////////////////////////////////////////
// Detect obstacles - new algorithm
void SensorVision::DetectObstacle(void){

  std::list<Vec> black;
  float angleStep = DTOR( 1.5 );

  // Parent pose
  Pose3d ppose = this->body->GetAbsPose();
  ppose.pos.z = 0;

  std::vector<Body*>::iterator oit = this->obstacleList.begin();
  for( ; oit != this->obstacleList.end(); oit++){

    Vector3 aabb_min, aabb_max;
    Pose3d opose; // Obstacle pose
    Pose3d rpose; // Relative pose

    opose = (*oit)->GetAbsPose();
    opose.pos.z = 0; // ground level ...
    // Get the relative position
    rpose = opose - ppose;

    float radius;
    // I don't know the dimensions of the obstacle but i can
    // assume that every obstacle has the shape of a cylinder,
    // therefore i can use the axis-align bounding box to obtain
    // the obstacle radius.
    (*oit)->GetBoundingBox( aabb_min, aabb_max );
    radius = (aabb_max.x - aabb_min.x) * 0.5;
    radius = (aabb_max.y - aabb_min.y) * 0.5 > radius ?
             (aabb_max.y - aabb_min.y) * 0.5 : radius;


    // Find if the obstacle is detected by a sensor.
    float angle = std::atan2(  rpose.pos.y, rpose.pos.x );
    float angleDelta = std::atan(radius / rpose.pos.GetLength() );

    // Find the 'start' and 'end' angles that are multiples of angleStep.
    float startAt = ((int)((angle + angleDelta) / angleStep)) * angleStep;
    float endAt   = ((int)((angle - angleDelta) / angleStep)) * angleStep;

    if ( startAt < endAt ){
      angle   = startAt;
      startAt = endAt;
      endAt   = angle;
    }

    for ( angle = startAt; angle >= endAt; angle -= angleStep ){
      Vec  prot( std::cos(angle), std::sin(angle) );
      Line ray( Vec(0,0), Vec(0,0) + prot );

      // Obstacle representation
      Circle orep( Vec(rpose.pos.x, rpose.pos.y), radius);
      // intersection points
      std::vector<Vec> ip = intersect( ray, orep );
      if ( !ip.empty() ){
        // As the obstacle is represented as a circle, most of the times
        // 2 points are returned, but I only want the closest one.
        Vec maybeBlack;
        if ( ip.size() == 1 )
          maybeBlack= ip[0];
        else
          if( ip[0].length() < ip[1].length() )
            maybeBlack = ip[0];
          else
            maybeBlack = ip[1];

        Vector3 pos = Vector3( maybeBlack.x, maybeBlack.y, 0.0 );
        if (! this->OnOcclusionArea( pos )  ){
            if(this->noisyObstacles){
                this->NoisyPosition(pos);
                maybeBlack.x = pos.x;
                maybeBlack.y = pos.y;
            }

            black.push_back( maybeBlack );
        }

      } // end if ( ip.size() == 1 )
    } // end for

  }// end for all obstacles

  black.sort( comparePosByAngle );

  unsigned int i = 0;
  std::list<Vec>::iterator it;

  for ( it = black.begin() ; it != black.end(); it++ ){
    this->visionInfo.obstacles.point[i] = (*it);
    i++;
  }

  this->visionInfo.obstacles.nPoints = i;

}

// Search for obstacles
void SensorVision::FillObstacleList(){
  
  std::vector<Model*> worldModels = World::Instance()->GetModels();
  std::vector<Model*>::iterator it= worldModels.begin();
  
  Body* canonical;
  Vector3 pigment;
  
  for( ; it != worldModels.end(); it++ ){
    
    canonical = (*it)->GetCanonicalBody();
    if ( canonical == NULL ) continue;
    
    // I cant see my self ...
    if ( canonical == this->body ) continue;
    
    // In search for black(ish) bodies..
    pigment = canonical->GetPigment();
    if ( pigment.x > 0.2 || pigment.y > 0.2 || pigment.z > 0.2 )
      continue;
    
    // you my friend, are an obstacle.
    this->obstacleList.push_back( canonical );
    
    gzmsg(8) << "Add " << canonical->GetScopedName()  << " to obstacle list"<< std::endl;
  }
  
}

bool SensorVision::OnOcclusionArea(Vector3 position){
 
  // Check for occlusion
  //float partial = 1.0;
  float distance = Vector3( position.x, position.y, 0.0).GetLength();
  float height   = position.z;
  
  std::vector<OcclusionArea>::iterator it = this->occlusion.begin();
  for ( ; it != this->occlusion.end(); it++ ){
  
    if ( distance < (*it).distance )
      continue; // The ball is between the robot and the obstacle.
    
    if ( height > (*it).height )
      continue; // the ball is above the obstacle, so it is visible
    
    cambada::geom::Angle angle;

    cambada::geom::Angle a;
    cambada::geom::Angle b;

    angle.set_rad( std::atan2( position.x, position.y ) );
    a.set_rad( (*it).angles.first  );
    b.set_rad( (*it).angles.second );

    if ( angle.in_between( a, b ) ){
        return true;
    }

//    float angle = std::atan2( position.x, position.y ) ;
//    float a = (*it).angles.first;
//    float b = (*it).angles.second;
//    if ( b >= a ){
//      // a <= angle <= b
//      if ( ( a <= angle ) && ( angle <= b ) ){
//          std::cout << "OCLUDED a " <<  (*it).distance << " a1 " << a << " a2 " << b << std::endl;
//        return true;
//      } // end if ( a <= angle <= b )
    
//    }else{
//      //  a <= angle OR angle <= b
//      if ( ( a <= angle ) || ( angle <= b ) ){
//          std::cout << "OCLUDED b " <<  (*it).distance << " a1 " << a << " a2 " << b << std::endl;
//        return true;
//      } // end if ( a <= angle OR angle <= b )
//    }
  
//    // Side partials...
  }
  
  return false;

}

void SensorVision::NoisyPosition(Vector3 &pos, bool noTheta){

    // X polyfit 0.0017563   0.0100797   0.0141694
    // Y polyfit 0.0084655  -0.0203064   0.0244877

    Vec helper;
    double var, rho, theta;
    double distance;

    helper.x = pos.x; helper.y = pos.y;
    distance = helper.length();

    // 0.0082489  -0.0033775   0.007199
    var = 0.0082489*distance*distance - 0.0033775*distance +   0.007199;
    rho = distance + Rand::GetDblNormal( 0.0, var );

    var   = (6.5933e-04)*distance*distance - (5.3490e-03)*distance + 2.4949e-02;
    theta = std::atan2( pos.y, pos.x ) +  Rand::GetDblNormal( 0.0, var);

    pos.x = rho * std::cos( theta );
    pos.y = rho * std::sin( theta );

//    double dX = helper.x; double dY = helper.y;
//    double XSigma = dX * dX * 0.0017563 + dX *  0.0100797 + 0.0141694;
//    double YSigma = dY * dY * 0.0084655 + dY * -0.0203064 + 0.0244877;

//    if ( noTheta == true ){
//        pos.x += Rand::GetDblNormal( 0.0, XSigma);
//        pos.y += Rand::GetDblNormal( 0.0, YSigma);
//    }
}

