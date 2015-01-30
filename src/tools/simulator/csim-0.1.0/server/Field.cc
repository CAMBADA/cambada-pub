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
 
#include "Field.hh"

#include "GazeboError.hh"
#include "XMLConfig.hh"

using namespace csim;
using namespace gazebo;

// Line segment constructor.
LineSegment::LineSegment( Point& a, Point& b) : pointA(a), pointB(b)
{ }

// Line segment constructor.
LineSegment::LineSegment( float a[2], float b[2] )
{ 
  pointA.x = a[0]; pointA.y = a[1];
  pointB.x = b[0]; pointB.y = b[1]; 
}

// Field constructor.
Field::Field()
{
  /* Nothing to do here, for now!!*/
  this->fieldSegments = new std::vector<LineSegment>();
}

// Field destructor.
Field::~Field()
{
  /* Nothing to do here, for now!!*/
  delete this->fieldSegments;
}

// Load options
void Field::Load(XMLConfigNode *node){
  
  if ( node == NULL )
    gzthrow("Field section is missing\n");
    
  // TODO: Use oficial robocup measurements as default
  this->fieldLength = node->GetDouble("fieldLength", 0.0, 1 );
  this->fieldWidth  = node->GetDouble("fieldWidth",  0.0, 1 );
  
  this->centerCircleRadius = node->GetDouble("centerCircleRadius",  0.0, 1 );
  
  this->goalieAreaWidth  = node->GetDouble("goalieAreaWidth",  0.0, 1 );
  this->goalieAreaLength = node->GetDouble("goalieAreaLength", 0.0, 1 );
  
  this->penaltyAreaWidth  = node->GetDouble("penaltyAreaWidth",  0.0, 1 );
  this->penaltyAreaLength = node->GetDouble("penaltyAreaLength", 0.0, 1 );
  
  this->north = node->GetDouble("north", 90.0, 0);
  
}

std::vector<Point> Field::Intersect( Ray ray ){
  
  std::vector<LineSegment>::iterator iter = this->fieldSegments->begin();
  std::vector<Point> intersections(0);
  
  // TODO: better understanding of this algorithm...
  
  // go through all line segments..
  for( ; iter != this->fieldSegments->end(); ++iter ){
    
    // ray equation:  p = R0 + t * Rdir, t >= 0
    // line equation: p = P0 + s * (P1 - P0), s any real number
    //
    // (R0x,R0y) + t * (Rdirx, Rdiry) = (P0x, P0y) + s * ( P1x-P0x, P1y-P0y )
    // { R0x + t * Tdirx = P0x + s * (P1x-P0x)
    // { R0y + t * Tdiry = P0y + s * (P1y-P0y)
    
    Point d1 = (*iter).pointB - (*iter).pointA;   //  Segment: PointB - PointA
    Point d2 = ray.y - ray.x;           //      Ray: PointB - PointA

    // Calculo do determinante..
    float det = d1.x*d2.y - d2.x*d1.y;

    Point dp = ray.x - (*iter).pointA;
    float tau= (d2.y*dp.x - d2.x*dp.y)/det;
    
    if ( fabs(det) < 1e-5f ) continue;



    // intersection point
    Point is =  (*iter).pointA * (1.0f-tau)  + (*iter).pointB * tau;
    float t;
    
    if ( (*iter).pointA.x != (*iter).pointB.x)
      t = ( is.x - (*iter).pointA.x ) / ( (*iter).pointB.x - (*iter).pointA.x );
    else
      t = ( is.y - (*iter).pointA.y ) / ( (*iter).pointB.y - (*iter).pointA.y );
  
    if ( tau< 0.0f || tau>1.0f) continue;
    
    intersections.push_back( is );
  }  

  return intersections;
}

// Init Field
void Field::Init(){
  
  float flh = this->fieldLength / 2.0;
  float fwh = this->fieldWidth / 2.0;
  float a[2];      // Point A
  float b[2];      // Point B
  

/*
  ====================================================
    DISCLAIMER: I just adapted the code, don't blame
  me if you can't *read* it.
  ====================================================
*/
  
  //
  //  World coordinates:
  //        ^  z
  //        | 
  //        |
  //        |-------> y
  //       /
  //     /   x
  
  //
  //  CAMBADA coordinates:
  //        ^  z
  //        | 
  //        |   / y
  //        | /
  //        |-------> x
  //
  
  // Lets draw the lines within CAMABADA coordinates
  
  /** Field Lines **/
  // bottom
  a[0] = fwh; a[1] =  flh;
  b[0] = fwh; b[1] = -flh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  // top
  a[0] = b[0] = -fwh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  // left
  a[1] = -flh;
  b[0] =  fwh; b[1] = -flh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  // right
  a[1] = flh;
  b[1] = flh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  // middle
  a[0] = -fwh; a[1] = 0;
  b[0] =  fwh; b[1] = 0;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
  // semi-circle
  const unsigned int kResolution = 12;
  const float kAngle = 3.1415/kResolution;
  unsigned int i;

  a[0] = -this->centerCircleRadius; a[1] = 0;
  for ( i = 1; i <= kResolution; i++ ){

    b[0] = -cos( kAngle*i ) * this->centerCircleRadius;
    b[1] =  sin( kAngle*i ) * this->centerCircleRadius;
    
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );

    // Draw symmetric then revert.
    a[1] = -a[1];
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
    b[1] = -b[1];
    a[1] = -a[1];
    //
    
    a[0] = b[0]; // Yes.. at the last cicle this is futile,
    a[1] = b[1]; // but, I can live with that : )
  }
  
  /** Goalie Area **/
  {
    // bottom line on the left side
    a[0] = this->goalieAreaWidth/2.0; a[1] = -flh;
    b[0] = this->goalieAreaWidth/2.0; b[1] = -flh + this->goalieAreaLength;
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // bottom line on the right side 
    a[1] = -a[1];
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // top line on the right side 
    a[0] = -a[0];
    b[0] = -b[0];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // top line on the left side 
    a[1] = -a[1]; 
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // parallel line to goal line on the left
    a[0] = -a[0];
    a[1] =  b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // parallel line to goal line on the right
    a[1] = -a[1]; 
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  }
  
  /** Penalty Area **/
  {
    // bottom line on the left side
    a[0] = this->penaltyAreaWidth/2.0; a[1] = -flh;
    b[0] = this->penaltyAreaWidth/2.0; b[1] = -flh + this->penaltyAreaLength;
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // bottom line on the right side 
    a[1] = -a[1];
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // top line on the right side 
    a[0] = -a[0];
    b[0] = -b[0];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // top line on the left side 
    a[1] = -a[1]; 
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // parallel line to goal line on the left
    a[0] = -a[0];
    a[1] =  b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  
    // parallel line to goal line on the right
    a[1] = -a[1]; 
    b[1] = -b[1];
    this->fieldSegments->push_back( csim::LineSegment( a, b ) );
  }
  
  /* Corners */
  // TOOD: clean this up, if possible
  a[1] = -flh;
  a[0] = -fwh + .5;
  b[1] = -(+flh - .3);
  b[0] = -fwh + .3;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = -(+flh - .5);
  a[0] = -fwh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = -flh;
  a[0] = +fwh - .5;
  b[1] = -(+flh - .3);
  b[0] = fwh - .3;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = -(+flh - .5);
  a[0] = fwh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = flh;
  a[0] = -fwh + .5;
  b[1] = -(-flh + .3);
  b[0] = -fwh + .3;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = -(-flh + .5);
  a[0] = -fwh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = flh;
  a[0] = +fwh - .5;
  b[1] = -(-flh + .3);
  b[0] = fwh - .3;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );

  a[1] = -(-flh + .5);
  a[0] = fwh;
  this->fieldSegments->push_back( csim::LineSegment( a, b ) );
}






