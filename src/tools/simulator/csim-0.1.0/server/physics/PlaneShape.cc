/* *  Gazebo - Outdoor Multi-Robot Simulator
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

#include "Geom.hh"
#include "PlaneShape.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
PlaneShape::PlaneShape(Geom *parent) : Shape(parent)
{
  this->type = Shape::PLANE;

  Param::Begin(&this->parameters);
  this->normalP = new ParamT<Vector3>("normal",Vector3(0,0,1),0);
  this->normalP->Callback( &PlaneShape::SetNormal, this );
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
PlaneShape::~PlaneShape()
{
  delete this->normalP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the plane
void PlaneShape::Load(XMLConfigNode *node)
{
  Vector3 perp;

  this->normalP->Load(node);
  this->CreatePlane();
}

////////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void PlaneShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->normalP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Create the plane
void PlaneShape::CreatePlane()
{

}

////////////////////////////////////////////////////////////////////////////////
/// Set the altitude of the plane
void PlaneShape::SetAltitude(const Vector3 &pos) 
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the normal
void PlaneShape::SetNormal( const Vector3 &norm )
{
  this->normalP->SetValue( norm );
  this->CreatePlane();
}

