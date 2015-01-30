/*
 *  Gazebo - Outdoor Multi-Robot Simulator
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

// 
//  Drawstuff.hh
//  gazebo-0.10.0
//  
//  Created by Eurico Pedrosa on 2010-01-18.
//  Copyright 2010 __MyCompanyName__. All rights reserved.
// 

#ifndef _DRAW_HH_
#define _DRAW_HH_

#include <string>
#include <vector>
#include <boost/thread.hpp>

#include "SingletonT.hh"
#include "Field.hh"

namespace gazebo {
  
  class XMLConfigNode;
  class Body;
  
  class Draw : public SingletonT<Draw> {
    
    public: enum ViewMode { kFreeView = 0, kTopView, kBallView, kRobotView }; 
    
    private: Draw();
    private: ~Draw();
    
    public: void Load(XMLConfigNode *node);
    public: void Init();
    public: void Fini();
    
    public: void StartDraw();
    public: void DrawField();
    public: void HandleView();
    
    public: Body* GetBall(){ return this->ball; };
    public: int viewmode;
    public: int toggleMoveBall;

    private: void DrawLoop();
    private: boost::thread* drawThread;
    private: csim::Field* field;
    private: std::string ballModelName;
    private: Body* ball;
    private: std::vector<Body*> robbies;
    
    //Singleton implementation
    private: friend class DestroyerT<Draw>;
    private: friend class SingletonT<Draw>;

  };
  
}


#endif /* _DRAWSTUFF_HH_ */
