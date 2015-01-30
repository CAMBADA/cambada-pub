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
 
#ifndef _COMM_HH_
#define _COMM_HH_

#include "gazebo.h"
#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{ 
  
/// \addtogroup gazebo_controller
/// \{
/** \defgroup controller_stub controller_stub

  \brief A stubbed out controller.

  Copy this example code when creating a new controller
  \{
*/

/// \brief A stubbed out controller.
class Comm : public Controller
{
  /// Constructor
  public: Comm(Entity *parent );

  /// Destructor
  public: virtual ~Comm();

  /// Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  private:
    /// The parent Model
    Model *myParent;
    std::string rtdbConfigFile;
    std::string PMANConfigFile;
    
    int rtdbNum;
    
    /// Allow only one instance
    static int commLoaded;
    
    // Control resources to be released0
    bool fRTDB;
};

/** \} */
/// \}

}

#endif /* !_COMM_HH_ */

