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
 *  @Desc   Game field representation
 *
 */

#ifndef _FIELD_HH_
#define _FIELD_HH_

#include "Vector2.hh"
#include "XMLConfig.hh"
#include <vector>

namespace csim {
  
  typedef gazebo::Vector2<float> Point;
  typedef gazebo::Vector2< Point > Line;
  typedef gazebo::Vector2< Point > Ray;
  /// Line segment utility
  class LineSegment {
    
  public:
    /// Constructor.
    /**
      To point are needed to define a line segmente.
      The order doesn't count.
     */
    LineSegment( Point& a, Point& b);
    /// Constructor.
    /**
      Creates a line segment defined by point a and point b.
    
      @param a  Must point to an array with at least 2 elements
      @param b  Must point to an array with at least 2 elements
     */
    LineSegment( float* a, float* b );
    

    Point pointA;
    Point pointB;
    
  };
  
  
  /// The game field.
  /**
      The game field is defined by its dimensions and rule lines. Those dimensions and
      rule lines are represented by Line Segments. 
  */
  class Field {
    
  public:
    /// Constructor.
    Field();
    /// Destructor.
    virtual ~Field();
    /// Load field options from configuration file
    void Load(gazebo::XMLConfigNode *node);
    /// Init field.
    /**
        A vector with all line segments that defines the field will be
        build.
     */
    void Init();
    
    /// Field Segments getter.
    std::vector<LineSegment>* GetSegments(){ return this->fieldSegments; };
    
    /// Ray casting on the field.
    /**
        Cast a ray on the field and see if it intersects with any line segment.
        It return a vector with all the points where an intersection occurs.
    */
    std::vector<Point> Intersect( Ray ray );
    
    /*** Field Properties ***/
    // Create Getters and Setters for all the properties would a PITA...
    
    /// Field length in meters (Oy in world coordinates).
    double fieldLength;
    /// Field width in meters (Ox in world coordinates).
    double fieldWidth;
    /// Center field circle radius in meters.
    double centerCircleRadius;
    /// Goalie width in meters (Ox in world coordinates).
    double goalieAreaWidth;
    /// Goalie length in meters (Oy in world coordinates).
    double goalieAreaLength;
    /// Penalty width in meters (Ox in world coordinates).
    double penaltyAreaWidth;
    /// Penalty length in meters (Ox in world coordinates).
    double penaltyAreaLength;
    /// World north
    double north;
    /************************/
    
  private:
    std::vector<LineSegment>* fieldSegments;
    
  }; /* @end of class */
  
}

#endif /* _FIELD_HH_ */


