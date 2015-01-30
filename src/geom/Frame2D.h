/*
 * Copyright (c) 1999 - 2001, Artur Merke <amerke@ira.uka.de> 
 * Adapted to robotcontrol 2004, Sascha Lange <salange@uos.de>
 * Adapted in 2007 by CAMBADA <cambada@ua.pt>
 *
 * This file is part of FrameView2d.
 *
 * FrameView2d is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * FrameView2d is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FrameView2d; see the file COPYING.  If not, write to
 * the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _FRAME2D_
#define _FRAME2D_

#include "Vec.h"
#include "iostream"
#include "Angle.h"

namespace cambada {
namespace geom {

/*
interesting facts about frames can be found in the very recommandable book
by R.P. Paul "Robot Manipulators" MIT Press 1981

A two dimensional frame has the following shape:

 / n_x  -n_y   p_x \
 |                 |
 | n_y   n_x   p_y |
 |                 |
 \  0     0     1  /

where we require n_x^2+n_y^2= 1 (i.e. unit length). From this follows, that
all n_x and  n_y can be represented by the cos(a) or sin(a) values for a particular angle a.

A translation is a frame of the following form

 /  1     0     x  \
 |                 |
 |  0     1     y  | = T(x,y)
 |                 |
 \  0     0     1  /

and a rotation can be represented by a frame of the form

 / cos(a) -sin(a) 0 \
 |                  |
 | sin(a) cos(a)  0 | = R(a)
 |                  |
 \   0      0     1 /

*/

class Frame2d {

  friend Frame2d operator*(const Frame2d &, const Frame2d &);
  friend Vec operator*(const Frame2d &, const Vec &);
 public:
  Frame2d(); //init as identity frame
  Frame2d(Vec, Angle);
  static Frame2d Translation(double, double);
  static Frame2d Rotation(const Angle&);
  Angle get_angle() const;
  Vec get_heading() const;

  Vec get_x_axis() const { return Vec(n_x,n_y); }
  Vec get_y_axis() const { return Vec(-n_y,n_x); }
  Vec get_pos() const { return Vec(p_x,p_y); }
  void set_angle(const Angle&);
  void set_angle(Vec);
  void set_position(double,double);
  void set_position(Vec);
  void set_scale(double,double);
  void set_scale(double);

  double get_scale() const { return scale; }

  void invert();
  double n_x, n_y, p_x, p_y;
 private:
  /** \short scale influences the magnification factor of all objects in a frame
      normally scale should always be = 1.0, but if you want all objects in
      your frame to be 2.5 as large, scale should be = 2.5
   */
  double scale;

};

//std::ostream & operator<< (std::ostream&, const Frame2d&);

}
}

#endif
