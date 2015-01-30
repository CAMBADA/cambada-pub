/*
 * Copyright (c) 2002-2005, Neuroinformatics research group, 
 * University of Osnabrueck <tribots@informatik.uni-osnabrueck.de>
 * Adapted in 2007 by CAMBADA <cambada@ua.pt>
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include "FieldLUT.h"
#include "ConfigXML.h"
	
#include "geometry.h"
#include <cmath>

#include <fstream>
#define TEST_FIELDLUT 0

using namespace std;

namespace cambada {
namespace loc {

FieldLUT::~FieldLUT () throw () {
  delete [] array;
  delete [] grad;
}

FieldLUT::FieldLUT ( ConfigXML* config, /*const FieldGeometry& fg,*/ unsigned int c)/* throw (std::bad_alloc)*/ : cell_size(c), array(NULL) {
  //const int igoal_band_length			= config->getField("goal_band_length");
  const int ifield_length				= config->getField("field_length");
  const int ifield_width				= config->getField("field_width");
  const int iside_band_width			= config->getField("side_band_width");
  const int igoal_band_width			= config->getField("goal_band_width");
  const int igoal_area_length			= config->getField("goal_area_length");
  const int igoal_area_width			= config->getField("goal_area_width");
  const int ipenalty_area_length		= config->getField("penalty_area_length");
  const int ipenalty_area_width			= config->getField("penalty_area_width");
  const int icenter_circle_radius		= config->getField("center_circle_radius");
  const int icorner_arc_radius			= config->getField("corner_arc_radius");
  const int ipenalty_marker_distance	= config->getField("penalty_marker_distance");
  //const int iline_thickness				= config->getField("line_thickness");
  //const int iborder_line_thickness		= config->getField("border_line_thickness");
  const int igoal_width					= config->getField("goal_width");
  const int igoal_length				= config->getField("goal_length");
  //const int igoal_height				= config->getField("goal_height");
	
  x_res = static_cast<unsigned int>(ceil((0.5*ifield_width+iside_band_width)/static_cast<double>(cell_size)));
  y_res = static_cast<unsigned int>(ceil((0.5*ifield_length+iside_band_width)/static_cast<double>(cell_size)));
  // GUS y_res = static_cast<unsigned int>(ceil((0.5*ifield_length+igoal_band_width)/static_cast<double>(cell_size)));
  array = new double [4*x_res*y_res];
  grad = new Vec [4*x_res*y_res];

  error_outside = (iside_band_width>igoal_band_width ? iside_band_width : igoal_band_width);

  // all entry to maximum values set 
  double max_val = 1e100;

  double* arr_ptr=array;
  const double* end_arr_ptr=array+4*x_res*y_res;
  while (arr_ptr<end_arr_ptr) 
    *(arr_ptr++) = max_val;

/*
  // attempt: “Posts” - effect model 
  draw_line_segment (Vec (0.5*igoal_width,-0.5*ifield_length), Vec (0.5*igoal_width+500,-0.5*ifield_length-500));
  draw_line_segment (Vec (0.5*igoal_width,0.5*ifield_length), Vec (0.5*igoal_width+500,0.5*ifield_length+500));
  draw_line_segment (Vec (-0.5*igoal_width,-0.5*ifield_length), Vec (-0.5*igoal_width-500,-0.5*ifield_length-500));
  draw_line_segment (Vec (-0.5*igoal_width,0.5*ifield_length), Vec (-0.5*igoal_width-500,0.5*ifield_length+500));
  */


  // GUS - goals drawing
  draw_line_segment( Vec(0.5*igoal_width,-0.5*ifield_length) , Vec(0.5*igoal_width,-0.5*ifield_length-igoal_length ) );
  draw_line_segment( Vec(-0.5*igoal_width,-0.5*ifield_length) , Vec(-0.5*igoal_width,-0.5*ifield_length-igoal_length ) );
  draw_line_segment( Vec(-0.5*igoal_width,-0.5*ifield_length-igoal_length) , Vec(0.5*igoal_width,-0.5*ifield_length-igoal_length ) );
  //
  draw_line_segment( Vec(0.5*igoal_width,0.5*ifield_length) , Vec(0.5*igoal_width,0.5*ifield_length+igoal_length ) );
  draw_line_segment( Vec(-0.5*igoal_width,0.5*ifield_length) , Vec(-0.5*igoal_width,0.5*ifield_length+igoal_length ) );
  draw_line_segment( Vec(-0.5*igoal_width,0.5*ifield_length+igoal_length) , Vec(0.5*igoal_width,0.5*ifield_length+igoal_length ) );

// gradually the lines draw in 
  draw_line_segment (Vec (0.5*ifield_width,-0.5*ifield_length), Vec (0.5*ifield_width,0.5*ifield_length));   // collateral line 
  draw_line_segment (Vec (-0.5*ifield_width,-0.5*ifield_length), Vec (-0.5*ifield_width,0.5*ifield_length));   // collateral line 
  draw_line_segment (Vec (0.5*ifield_width,0.5*ifield_length), Vec (-0.5*ifield_width,0.5*ifield_length));  // Torauslinie
  draw_line_segment (Vec (0.5*ifield_width,-0.5*ifield_length), Vec (-0.5*ifield_width,-0.5*ifield_length));  // gate from line 
  
  draw_line_segment (Vec (-0.5*ifield_width,0), Vec(0.5*ifield_width,0));     // center line 
  
  if ((igoal_area_width>0) && (igoal_area_length>0)) {
    // // gate area 
    draw_line_segment (Vec(-0.5*igoal_area_width, 0.5*ifield_length-igoal_area_length), Vec(0.5*igoal_area_width,0.5*ifield_length-igoal_area_length));
    draw_line_segment (Vec( 0.5*igoal_area_width, 0.5*ifield_length-igoal_area_length), Vec(0.5*igoal_area_width,0.5*ifield_length));
    draw_line_segment (Vec(-0.5*igoal_area_width, 0.5*ifield_length-igoal_area_length), Vec(-0.5*igoal_area_width,0.5*ifield_length));
    draw_line_segment (Vec(-0.5*igoal_area_width,-0.5*ifield_length+igoal_area_length), Vec(0.5*igoal_area_width,-0.5*ifield_length+igoal_area_length));
    draw_line_segment (Vec( 0.5*igoal_area_width,-0.5*ifield_length+igoal_area_length), Vec(0.5*igoal_area_width,-0.5*ifield_length));
    draw_line_segment (Vec(-0.5*igoal_area_width,-0.5*ifield_length+igoal_area_length), Vec(-0.5*igoal_area_width,-0.5*ifield_length));
  }
  if ((ipenalty_area_width>0) && (ipenalty_area_length>0)) {
    // Strafraum
    draw_line_segment (Vec(-0.5*ipenalty_area_width,0.5*ifield_length-ipenalty_area_length), Vec(0.5*ipenalty_area_width,0.5*ifield_length-ipenalty_area_length));
    draw_line_segment (Vec(0.5*ipenalty_area_width,0.5*ifield_length-ipenalty_area_length), Vec(0.5*ipenalty_area_width,0.5*ifield_length));
    draw_line_segment (Vec(-0.5*ipenalty_area_width,0.5*ifield_length-ipenalty_area_length), Vec(-0.5*ipenalty_area_width,0.5*ifield_length));
    draw_line_segment (Vec(-0.5*ipenalty_area_width,-0.5*ifield_length+ipenalty_area_length), Vec(0.5*ipenalty_area_width,-0.5*ifield_length+ipenalty_area_length));
    draw_line_segment (Vec(0.5*ipenalty_area_width,-0.5*ifield_length+ipenalty_area_length), Vec(0.5*ipenalty_area_width,-0.5*ifield_length));
    draw_line_segment (Vec(-0.5*ipenalty_area_width,-0.5*ifield_length+ipenalty_area_length), Vec(-0.5*ipenalty_area_width,-0.5*ifield_length));
  }
  if (icenter_circle_radius>0 ) {
    draw_arc (Vec(0,0), icenter_circle_radius, Angle::zero, Angle::half); // center circle segment 
    draw_arc (Vec(0,0), icenter_circle_radius, Angle::half, Angle::zero);  // center circle segment 
  }
  if (icorner_arc_radius>0) {
    draw_arc (Vec(0.5*ifield_width,0.5*ifield_length), icorner_arc_radius, Angle::half, Angle::three_quarters);  // corner
    draw_arc (Vec(0.5*ifield_width,-0.5*ifield_length), icorner_arc_radius, Angle::quarter, Angle::half);  // corner
    draw_arc (Vec(-0.5*ifield_width,0.5*ifield_length), icorner_arc_radius, Angle::three_quarters, Angle::zero);  // corner
    draw_arc (Vec(-0.5*ifield_width,-0.5*ifield_length), icorner_arc_radius, Angle::zero, Angle::quarter);  // corner
  }
  if (ipenalty_marker_distance>0) {
    draw_dot (Vec(0,0.5*ifield_length-ipenalty_marker_distance));  // point of punishing impact 
    draw_dot (Vec(0,-0.5*ifield_length+ipenalty_marker_distance));  // point of punishing impact 
  }

  // // gradient approximate with Sobelfilter 
  // first the edge ignore and the inside compute 
  for (unsigned int xi = 1; xi<2*x_res-1; xi++)
    for (unsigned int yi=1; yi<2*y_res-1; yi++) {  // Sobelfilter use 
      double lo = array[xi+2*x_res*yi-1+2*x_res];
      double lm = array[xi+2*x_res*yi-1];
      double lu = array[xi+2*x_res*yi-1-2*x_res];
      double ro = array[xi+2*x_res*yi+1+2*x_res];
      double rm = array[xi+2*x_res*yi+1];
      double ru = array[xi+2*x_res*yi+1-2*x_res];
      double mo = array[xi+2*x_res*yi+2*x_res];
      double mu = array[xi+2*x_res*yi-2*x_res];
      grad[xi+2*x_res*yi].x = 0.125*(-lo+ro-2*lm+2*rm-lu+ru)/static_cast<double>(cell_size);
      grad[xi+2*x_res*yi].y = 0.125*(lo-lu+2*mo-2*mu+ro-ru)/static_cast<double>(cell_size);
    }
  // at the edge simply the neighbouring gradients take over 
  for (unsigned int xi = 1; xi<2*x_res-1; xi++) {
    grad[xi] = grad[xi+2*x_res];
    grad[xi+2*x_res*(2*y_res-1)] = grad[xi+2*x_res*(2*y_res-2)];
  }
  for (unsigned int yi= 1; yi<2*y_res-1; yi++) {
    grad[2*x_res*yi] = grad[2*x_res*yi+1];
    grad[2*x_res*yi+2*x_res-1] = grad[2*x_res*yi+2*x_res-2];
  }
  // corners by averaging neighbouring peripheral points 
  grad[0]=0.5*(grad[1]+grad[2*x_res]);
  grad[2*x_res-1]=0.5*(grad[2*x_res-2]+grad[4*x_res-1]);
  grad[2*x_res*(y_res-1)]=0.5*(grad[2*x_res*(y_res-2)]+grad[2*x_res*(y_res-1)+1]);
  grad[4*x_res*y_res-1]=0.5*(grad[4*x_res*y_res-2]+grad[4*x_res*y_res-2*x_res-1]);

#if TEST_FIELDLUT
  // only to test purposes, graphic expenditure of the spacer array as PGM Frame 
  {
    ofstream foo ("fieldlut_f.pgm");
    foo << "P5\n" << 2*y_res << ' ' << 2*x_res << " 255\n";
    for (unsigned int xi = 0; xi<2*x_res; xi++)
      for (unsigned int yi = 0; yi<2*y_res; yi++)
	foo.put(static_cast<unsigned int>(255-array[xi+2*x_res*(2*y_res-yi-1)]/15));
  }
  {
    ofstream foo ("fieldlut_gx.pgm");
    foo << "P5\n" << 2*y_res << ' ' << 2*x_res << " 255\n";
    for (unsigned int xi = 0; xi<2*x_res; xi++)
      for (unsigned int yi = 0; yi<2*y_res; yi++)
	foo.put(static_cast<unsigned int>(127+127*grad[xi+2*x_res*(2*y_res-yi-1)].x));
  }
  {
    ofstream foo ("fieldlut_gy.pgm");
    foo << "P5\n" << 2*y_res << ' ' << 2*x_res << " 255\n";
    for (unsigned int xi = 0; xi<2*x_res; xi++)
      for (unsigned int yi = 0; yi<2*y_res; yi++)
	foo.put(static_cast<unsigned int>(127+127*grad[xi+2*x_res*(2*y_res-yi-1)].y));
  }
#endif
}

void FieldLUT::update (unsigned int xi, unsigned int yi, double v) {
  if (xi>=2*x_res || yi>=2*y_res)
    return;
  if (array[xi+2*x_res*yi]>v)
    array[xi+2*x_res*yi]=v;
}

double FieldLUT::distance (const Vec& p) const throw () {
  int xind = static_cast<int>(floor (p.x/cell_size)+x_res);
  int yind = static_cast<int>(floor (p.y/cell_size)+y_res);
  if ((xind<0) || (yind<0) || (xind>=2*static_cast<int>(x_res)) || (yind>=2*static_cast<int>(y_res)))
    return error_outside;   // default value for values outside of the field; should not occur actually 
  return array[static_cast<unsigned int>(xind+2*x_res*yind)];
}

Vec FieldLUT::gradient (const Vec& p) const throw () {
  int xind = static_cast<int>(floor (p.x/cell_size)+x_res);
  int yind = static_cast<int>(floor (p.y/cell_size)+y_res);
  if (xind<0)  // at the edge cut off 
    xind=0;
  if (xind>=2*static_cast<int>(x_res))
    xind=2*static_cast<int>(x_res)-1;
  if (yind<0)
    yind=0;
  if (yind>=2*static_cast<int>(y_res))
    yind=2*static_cast<int>(y_res)-1;
  return grad[static_cast<unsigned int>(xind+2*x_res*yind)];  
}

void FieldLUT::draw_line_segment (Vec start, Vec end) {
  LineSegment line (start, end);
  for (unsigned int xi=0; xi<2*x_res; xi++)
    for (unsigned int yi=0; yi<2*y_res; yi++) {
      Vec pp ((static_cast<double>(xi)-static_cast<double>(x_res)+0.5)*static_cast<double>(cell_size), (static_cast<double>(yi)-static_cast<double>(y_res)+0.5)*static_cast<double>(cell_size));
      update (xi,yi,line.distance (pp));
    }
}

void FieldLUT::draw_arc (Vec center, double radius, Angle start, Angle end) {
  Arc arc (center, radius, start, end);
  for (unsigned int xi=0; xi<2*x_res; xi++)
    for (unsigned int yi=0; yi<2*y_res; yi++) {
      Vec pp ((static_cast<double>(xi)-static_cast<double>(x_res)+0.5)*static_cast<double>(cell_size), (static_cast<double>(yi)-static_cast<double>(y_res)+0.5)*static_cast<double>(cell_size));
      update (xi,yi,arc.distance (pp));
    }
}

void FieldLUT::draw_dot (Vec p) {
  for (unsigned int xi=0; xi<2*x_res; xi++)
    for (unsigned int yi=0; yi<2*y_res; yi++) {
      Vec pp ((static_cast<double>(xi)-static_cast<double>(x_res)+0.5)*static_cast<double>(cell_size), (static_cast<double>(yi)-static_cast<double>(y_res)+0.5)*static_cast<double>(cell_size));
      update (xi,yi,(p-pp).length());
    }
}

}
}

