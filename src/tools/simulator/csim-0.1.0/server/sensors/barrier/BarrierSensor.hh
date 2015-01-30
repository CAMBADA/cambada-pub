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

#ifndef BARRIER_SENSOR_HH
#define BARRIER_SENSOR_HH

#include "Sensor.hh"

namespace csim
{
/// \addtogroup cambada_sensors
/// \brief CAMABADA sensors
/// \{
/// \defgroup cambada_barrier Barrier Sensor
/// \brief CAMBADA barrier sensor
// \{

  using namespace gazebo;

/// \brief Barrier sensor
///
class BarrierSensor : public Sensor
{
  /// \brief Constructor
  public: BarrierSensor(Body *body);

  /// \brief Destructor
  public: virtual ~BarrierSensor();
  
  /// \brief the body of the ball that the sensor is tracking
  public: Body* GetBallBody(){ return this->ball; };
  
  public: bool BallOnBarrierArea(){ return this->closed_circuit; };

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild();

  /// Finalize the camera
  protected: virtual void FiniChild();
  
private:
  std::string ballModelName;
  Body* ball;
  
  // Barrier area
  float angle; 
  float distance;
  
  bool  closed_circuit;
  
  int selfID;

};

/// \}
/// \}
}
#endif

