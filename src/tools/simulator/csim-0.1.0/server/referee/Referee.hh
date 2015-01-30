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
 *  @Date   2 Jun 2010
 *  @Desc   Automatic Referee
 *
 */
 
#ifndef __REFEREE_H__
#define __REFEREE_H__

#include <iostream>
#include <boost/thread.hpp>

#include "Field.hh"
#include "SingletonT.hh"

#define TEAM_NONE     0
#define TEAM_MAGENTA  1
#define TEAM_CYAN     2


// Referee States
#define RS_

namespace csim{

  class IRefBoxProtocol;
  class ServerSocket;
}

namespace gazebo {

  using namespace csim;
  
  class XMLConfigNode;  
  class ContactSensor;
  class Body;

  class Referee : public SingletonT<Referee> {

  public:
  
    void Load(XMLConfigNode* node);
    void Init();
    void Fini();
    
    void ApplyRules();
  
  private:

    Referee();
    virtual ~Referee();
    
    void SocketLoop();

    std::string ballModelName;
    std::string contactSensorName;
    
    ContactSensor* contactSensor;
    Body* ball;
    csim::Field* field;
    bool enabled;
    
    int lastContactTeam;
    int refereeState;
    
    // Connections
    ServerSocket* conn1;
    ServerSocket* conn2;
    
    // Referee Protocol
    IRefBoxProtocol* protocol;
    
    // Socket options
    int socketPort;
    
    // Thread & Syncronization
    boost::thread* socketLoop;
    boost::condition_variable listenClients;
    boost::mutex              mut_lc;
    bool newClients;

    bool shouldQuit;

    friend class DestroyerT<Referee>;
    friend class SingletonT<Referee>;
  
  }; 

}

#endif /* __REFEREE_H__ */

