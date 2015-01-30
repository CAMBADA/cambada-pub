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
/* Desc: A ray shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
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

#include "RayShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
RayShape::RayShape( Geom *parent ) : Shape(parent)
{
  this->type = Shape::RAY;
  this->SetName("Ray");

  this->contactLen = DBL_MAX;
  this->contactRetro = 0.0;
  this->contactFiducial = -1;

  this->parent->SetSaveable(false);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
RayShape::~RayShape()
{

}
 
////////////////////////////////////////////////////////////////////////////////
/// Set the ray based on starting and ending points relative to the body
void RayShape::SetPoints(const Vector3 &posStart, const Vector3 &posEnd)
{
  Vector3 dir;

  this->relativeStartPos = posStart;
  this->relativeEndPos = posEnd;

  this->globalStartPos = this->parent->GetAbsPose().CoordPositionAdd(
      this->relativeStartPos);
  this->globalEndPos = this->parent->GetAbsPose().CoordPositionAdd(
      this->relativeEndPos);

  // Compute the direction of the ray
  dir = this->globalEndPos - this->globalStartPos;
  dir.Normalize();

}

////////////////////////////////////////////////////////////////////////////////
/// Get the relative starting and ending points
void RayShape::GetRelativePoints(Vector3 &posA, Vector3 &posB)
{
  posA = this->relativeStartPos;
  posB = this->relativeEndPos;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the global starting and ending points
void RayShape::GetGlobalPoints(Vector3 &posA, Vector3 &posB)
{
  posA = this->globalStartPos;
  posB = this->globalEndPos;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the length of the ray
void RayShape::SetLength( double len )
{
  this->contactLen=len;

  Vector3 dir = this->relativeEndPos - this->relativeStartPos;
  dir.Normalize();

  this->relativeEndPos = dir * len + this->relativeStartPos;

  //std::cout << "Len[" << len << "] St[" << this->relativeStartPos << "] End[" << this->relativeEndPos << "] dir[" << dir << "]\n";

}

////////////////////////////////////////////////////////////////////////////////
/// Get the length of the ray
double RayShape::GetLength() const
{
  return this->contactLen;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the retro-reflectivness detected by this ray
void RayShape::SetRetro( float retro )
{
  this->contactRetro = retro;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the retro-reflectivness detected by this ray
float RayShape::GetRetro() const
{
  return this->contactRetro;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the fiducial id detected by this ray
void RayShape::SetFiducial( int fid )
{
  this->contactFiducial = fid;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the fiducial id detected by this ray
int RayShape::GetFiducial() const
{
  return this->contactFiducial;
}

////////////////////////////////////////////////////////////////////////////////
/// Load thte ray
void RayShape::Load(XMLConfigNode *node) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void RayShape::Save(std::string &, std::ostream &) 
{
}
