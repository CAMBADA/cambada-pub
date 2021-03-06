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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id: Geom.cc 8474 2009-12-18 17:26:23Z natepak $
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

#include <sstream>

#include "Shape.hh"
#include "Mass.hh"
#include "PhysicsEngine.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "SurfaceParams.hh"
#include "World.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( Body *body )
    : Entity(body->GetCoMEntity())
{
  this->physicsEngine = World::Instance()->GetPhysicsEngine();

  this->typeName = "unknown";

  this->body = body;

  // Create the contact parameters
  this->surface = new SurfaceParams();

  this->shape = NULL;

  this->contactsEnabled = false;

  Param::Begin(&this->parameters);
  this->massP = new ParamT<double>("mass",0.001,0);
  this->massP->Callback( &Geom::SetMass, this);

  this->xyzP = new ParamT<Vector3>("xyz", Vector3(), 0);
  this->xyzP->Callback( &Entity::SetRelativePosition, (Entity*)this);

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this);

  this->laserFiducialIdP = new ParamT<int>("laserFiducialId",-1,0);
  this->laserRetroP = new ParamT<float>("laserRetro",-1,0);
  
  this->pigmentP = new ParamT<Vector3>("pigment", this->body->GetPigment() , 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Geom::~Geom()
{
  delete this->massP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->laserFiducialIdP;
  delete this->laserRetroP;
  delete this->pigmentP;

  delete this->shape;
}

////////////////////////////////////////////////////////////////////////////////
// First step in the loading process
void Geom::Load(XMLConfigNode *node)
{
  this->xmlNode=node;

  this->typeName = node->GetName();

  this->nameP->Load(node);
  this->SetName(this->nameP->GetValue());
  this->nameP->Load(node);
  this->massP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->laserFiducialIdP->Load(node);
  this->laserRetroP->Load(node);
  this->pigmentP->Load(node);

  this->SetRelativePose( Pose3d( **this->xyzP, **this->rpyP ) );

  this->mass.SetMass( **this->massP );

  this->surface->Load(node);

  this->shape->Load(node);

  this->body->AttachGeom(this);

  if (this->GetType() != Shape::PLANE && this->GetType() != Shape::HEIGHTMAP)
  {
    World::Instance()->RegisterGeom(this);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Geom::Save(std::string &prefix, std::ostream &stream)
{
  if (!this->GetSaveable())
    return;

  std::string p = prefix + "  ";

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );

  stream << prefix << "<geom:" << this->typeName << " name=\"" 
         << this->nameP->GetValue() << "\">\n";

  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  this->shape->Save(p,stream);

  stream << prefix << "  " << *(this->massP) << "\n";

  stream << prefix << "  " << *(this->laserFiducialIdP) << "\n";
  stream << prefix << "  " << *(this->laserRetroP) << "\n";

  stream << prefix << "</geom:" << this->typeName << ">\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(bool placeable)
{
  this->physicsEngine->LockMutex();

  this->placeable = placeable;

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
  else
  {
    // collide with all
    this->SetCategoryBits(GZ_ALL_COLLIDE);
    this->SetCollideBits(GZ_ALL_COLLIDE);
  }

  this->physicsEngine->UnlockMutex();
}

////////////////////////////////////////////////////////////////////////////////
// Update
void Geom::Update()
{
  this->ClearContacts();
  this->shape->Update();
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser fiducial integer id
void Geom::SetLaserFiducialId(int id)
{
  this->laserFiducialIdP->SetValue( id );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser fiducial integer id
int Geom::GetLaserFiducialId() const
{
  return this->laserFiducialIdP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness
void Geom::SetLaserRetro(float retro)
{
  this->laserRetroP->SetValue(retro);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser retro reflectiveness
float Geom::GetLaserRetro() const
{
  return this->laserRetroP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Geom::SetMass(const Mass &_mass)
{
  this->mass = _mass;
  //this->body->UpdateCoM();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Geom::SetMass(const double &_mass)
{
  this->mass.SetMass( _mass );
  //this->body->UpdateCoM();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the body this geom belongs to
Body *Geom::GetBody() const
{
  return this->body;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model this geom belongs to
Model *Geom::GetModel() const
{
  return this->body->GetModel();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the friction mode of the geom
void Geom::SetFrictionMode( const bool &v )
{
  this->surface->enableFriction = v;
}


////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the mass
const Mass &Geom::GetMass() const
{
  return this->mass;
}

////////////////////////////////////////////////////////////////////////////////
// Get the shape type
Shape::Type Geom::GetType()
{
  return this->shape->GetType();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the shape for this geom
void Geom::SetShape(Shape *shape)
{
  this->shape = shape;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the attached shape
Shape *Geom::GetShape() const
{
  return this->shape;
}

const Vector3 Geom::GetPigment() const
{
  return Vector3( this->pigmentP->GetValue() );
}

////////////////////////////////////////////////////////////////////////////////
// Turn contact recording on or off
void Geom::SetContactsEnabled(bool enable)
{
  this->contactsEnabled = enable;
}

////////////////////////////////////////////////////////////////////////////////
// Return true of contact recording is on
bool Geom::GetContactsEnabled() const
{
  return this->contactsEnabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Add an occurance of a contact to this geom
void Geom::AddContact(const Contact &contact)
{
  if (!this->contactsEnabled)
    return;

  if (this->GetType() == Shape::RAY || this->GetType() == Shape::PLANE)
    return;

  this->contacts.push_back( contact.Clone() );
  this->contactSignal( contact );
}

////////////////////////////////////////////////////////////////////////////////
/// Clear all contact info
void Geom::ClearContacts()
{
  this->contacts.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of contacts
unsigned int Geom::GetContactCount() const
{
  return this->contacts.size();
}
            
////////////////////////////////////////////////////////////////////////////////
/// Get a specific contact
Contact Geom::GetContact(unsigned int i) const
{
  if (i < this->contacts.size())
    return this->contacts[i];
  else
    gzerr(0) << "Invalid contact index\n";

  return Contact();
}
