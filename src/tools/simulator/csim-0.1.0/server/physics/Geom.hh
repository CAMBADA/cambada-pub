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
 * SVN: $Id: Geom.hh 8474 2009-12-18 17:26:23Z natepak $
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

#ifndef GEOM_HH
#define GEOM_HH

#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include "Contact.hh"
#include "Shape.hh"
#include "Param.hh"
#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"
#include "Mass.hh"

namespace gazebo
{
  class Model;
  class Body;
  class SurfaceParams;
  class XMLConfigNode;
  class PhysicsEngine;

  /// \addtogroup gazebo_physics
  /// \brief Base class for all geoms
  /// \{

  /// \brief Base class for all geoms
  class Geom : public Entity
  {
    /// \brief Constructor
    //public: Geom(Body *body, const std::string &name);
    public: Geom(Body *body);
  
    /// \brief Destructor
    public: virtual ~Geom();

    /// \brief Load the geom
    public: void Load(XMLConfigNode *node);

    /// \brief Load the geom
    public: void Save(std::string &prefix, std::ostream &stream);
 
    /// \brief Set the encapsulated geometry object
    public: void SetGeom(bool placeable);
  
    /// \brief Update function for geoms
    public: void Update();
 
    /// \brief Return whether this geom is placeable
    public: bool IsPlaceable() const;
    
    /// \brief Set the category bits, used during collision detection
    /// \param bits The bits
    public: virtual void SetCategoryBits(unsigned int bits) = 0;
  
    /// \brief Set the collide bits, used during collision detection
    /// \param bits The bits
    public: virtual void SetCollideBits(unsigned int bits) = 0;
  
    /// \brief Get the mass of the geom
    public: virtual Mass GetBodyMassMatrix() = 0;
  
    /// \brief Set the laser fiducial integer id
    public: void SetLaserFiducialId(int id);
  
    /// \brief Get the laser fiducial integer id
    public: int GetLaserFiducialId() const;
  
    /// \brief Set the laser retro reflectiveness 
    public: void SetLaserRetro(float retro);
  
    /// \brief Get the laser retro reflectiveness 
    public: float GetLaserRetro() const;

    /// \brief Set the mass
    public: void SetMass(const double &mass);

    /// \brief Set the mass
    public: void SetMass(const Mass &mass);

    /// \brief Get the body this geom belongs to
    public: Body *GetBody() const;

    /// \brief Get the model this geom belongs to
    public: Model *GetModel() const;

    /// \brief Set the friction mode of the geom
    public: void SetFrictionMode( const bool &v );

    /// \brief Get the bounding box for this geom
    public: virtual void GetBoundingBox(Vector3 &min, Vector3 &max) const = 0;

    /// \brief Get a pointer to the mass
    public: const Mass &GetMass() const;

    /// \brief Get the shape type
    public: Shape::Type GetType();

    /// \brief Set the shape for this geom
    public: void SetShape(Shape *shape);
            
    /// \brief Get the attached shape
    public: Shape *GetShape() const;
    
    /// \brief Get geometry color
    public: const Vector3 GetPigment() const;

    /// \brief Turn contact recording on or off
    public: void SetContactsEnabled(bool enable);

    /// \brief Return true of contact recording is on
    public: bool GetContactsEnabled() const;

    /// \brief Add an occurance of a contact to this geom
    public: void AddContact(const Contact &contact);

    /// \brief Clear all contact info
    public: void ClearContacts();

    /// \brief Get the number of contacts
    public: unsigned int GetContactCount() const;
            
    /// \brief Get a specific contact
    public: Contact GetContact(unsigned int i) const;

    public: template< typename C>
            void ContactCallback( void (C::*func)(const Contact&), C *c )
            {
              contactSignal.connect( boost::bind(func, c, _1) );
            }

    ///  Contact parameters
    public: SurfaceParams *surface; 

    public: std::vector<Contact> contacts;
 
    /// The body this geom belongs to
    protected: Body *body;
  
    protected: bool placeable;

    protected: Mass mass;

    private: ParamT<int> *laserFiducialIdP;
    private: ParamT<float> *laserRetroP;

    ///  Mass as a double
    private: ParamT<double> *massP;

    protected: ParamT<Vector3> *xyzP;
    protected: ParamT<Quatern> *rpyP;
    
    protected: ParamT<Vector3> *pigmentP;

    ///our XML DATA
    private: XMLConfigNode *xmlNode;

    private: std::string typeName;

    protected: PhysicsEngine *physicsEngine;

    protected: Shape *shape;

    private: bool contactsEnabled;

    public: boost::signal< void (const Contact &)> contactSignal;
  };

  /// \}

}
#endif
