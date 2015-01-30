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
 
/* Desc: CAMABADA compass sensor
 * Author: Eurico F. Pedrosa <efp@ua.pt>
 * Date: 25 Fev 2010
 */

#ifndef SENSORSTUB_HH
#define SENSORSTUB_HH

#include "Sensor.hh"

namespace csim
{
/// \addtogroup cambada_sensors
/// \brief CAMABADA sensors
/// \{
/// \defgroup cambada_compass Compass Sensor
/// \brief CAMBADA compass sensor
// \{

  using namespace gazebo;

/// \brief Compass Sensor
///
/// Copy this sensor to create your own
class CompassSensor : public Sensor
{
  /// \brief Constructor
  public: CompassSensor(Body *body);

  /// \brief Destructor
  public: virtual ~CompassSensor();

  /// \brief Load the compass using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the compass
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild();

  /// Finalize the compass
  protected: virtual void FiniChild();
  
private:
  float theNorth;
  float adjustNorth;
  int   selfID;

  bool noisyCompass;

};

/// \}
/// \}
}
#endif

