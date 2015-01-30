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

#ifndef SENSORVISION_HH
#define SENSORVISION_HH

#include <string>
#include <vector>
#include <deque>
#include <utility>

#include "Body.hh"
#include "Sensor.hh"
#include "Field.hh"

// CAMBADA include
#include "VisionInfo.h"

namespace gazebo
{
  
  //
  typedef std::pair<float,float> BetweenAngles;
  struct OcclusionArea {    
  public:
    OcclusionArea(float d, float h,const BetweenAngles& ang) 
                 : distance(d), height(h), angles(ang)   { } 
    
    float         distance; // this is the distance between the robot and the obstacle
    float         height;   // the height of the obstacle
    BetweenAngles angles;   // pair of angles that confines the occlusion area

  };
  
/// \addtogroup gazebo_sensor
/// \brief Stubbed out sensor
/// \{
/// \defgroup gazebo_sensor_stub Sensor Stub
/// \brief Stubbed out sensor
// \{

/// \brief Stubbed out  sensor
///
/// Copy this sensor to create your own
class SensorVision : public Sensor
{ 
  /// \brief Constructor
  public: SensorVision(Body *body);

  /// \brief Destructor
  public: virtual ~SensorVision();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild();

  /// Finalize the camera
  protected: virtual void FiniChild();

  //
  private:
    
    void DetectOcclusions();
    void DetectBall();
    void DetectWhite();
    void DetectObstacle();
    
    void FillObstacleList();
    bool OnOcclusionArea(Vector3 position);
    void NoisyPosition(Vector3& pos, bool noTheta = false);
    
    int cycleDelay;
    float seeDistance;
    float height;

    bool noisyWhite;
    bool noisyBall;
    bool noisyObstacles;
    
    int radialSensors;  // Number of radial sensors
    int radialPasses;   // Number of passes.
    unsigned int maxWhitePoints;
    // Robot ID
    int selfID;

    // Vision information..
    VisionInfo      visionInfo;
    
    std::string ballModelName;
    Body* ball;
    float ballRadius;
    csim::Field* field;
    // Keep a list of all obstacles
    std::vector<Body*> obstacleList;
    // Occlusion area using "in between angles"
    std::vector< OcclusionArea > occlusion;
    
    // Omni vision Update queue
    std::deque<VisionInfo> *omniQueue;
    
    // PMAN config file
    std::string PMANConfigFile;
    bool fPMAN;
};

/// \}
/// \}
}
#endif

