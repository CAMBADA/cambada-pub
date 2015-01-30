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
 */
/* Desc: Local Gazebo configuration 
 * Author: Jordi Polo
 * Date: 3 May 2008
 * SVN: $Id: GazeboConfig.cc 8279 2009-10-06 13:58:48Z natepak $
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
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>

#include "XMLConfig.hh"
#include "GazeboConfig.hh"
#include "GazeboMessage.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
GazeboConfig::GazeboConfig()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
GazeboConfig::~GazeboConfig()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Loads the configuration file 
void GazeboConfig::Load()
{
  std::ifstream cfgFile;

  std::string rcFilename = getenv("HOME");
  rcFilename += "/.gazeborc";

  cfgFile.open(rcFilename.c_str(), std::ios::in);

  std::string delim(":");

  char *gazebo_resource_path = getenv("GAZEBO_RESOURCE_PATH");
  if(gazebo_resource_path) 
  {
    std::string str(gazebo_resource_path);
    int pos1 = 0;
    int pos2 = str.find(delim);
    while (pos2 != (int)std::string::npos)
    {
      this->gazeboPaths.push_back(str.substr(pos1,pos2-pos1));
      pos1 = pos2+1;
      pos2 = str.find(delim,pos2+1);
    }
    this->gazeboPaths.push_back(str.substr(pos1,str.size()-pos1));
  }

  if (cfgFile.is_open())
  {
    XMLConfig rc;
    XMLConfigNode *node;
    rc.Load(rcFilename);

    // if gazebo path is set, skip reading from .gazeborc
    if(!gazebo_resource_path)
    {
      node = rc.GetRootNode()->GetChild("gazeboPath");
      while (node)
      {
        gzmsg(2) << "Gazebo Path[" << node->GetValue() << "]\n";
        this->gazeboPaths.push_back(node->GetValue());
        node = node->GetNext("gazeboPath");
      }
    }

  }
  else
  {
    gzmsg(0) << "Unable to find the file ~/.gazeborc. Using default paths.\n";

    if ( !gazebo_resource_path )
    {
      this->gazeboPaths.push_back("/Users/efp/Sandbox");
      this->gazeboPaths.push_back("/usr/local/share/gazebo");
    }

  }
}

std::list<std::string> &GazeboConfig::GetGazeboPaths() 
{
  return this->gazeboPaths;
}
