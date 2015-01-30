/*
 * Copyright (c) 1999 - 2001, Artur Merke <amerke@ira.uka.de>
 * Adapted to robotcontrol 2004, Sascha Lange
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

#include "Frame2D.h"

namespace cambada {
namespace geom {


/*
  std::ostream& operator<< (std::ostream& o,const Frame2d& f) {

   //   return o << "\n   [" << f.n_x << " " << -f.n_y << " " << f.p_x << "]"
   //   << "\n   [" << f.n_y << " " <<  f.n_x << " " << f.p_y << "]";

    return o << f.p_x << " " << f.p_y << " " << f.n_x << " " << f.n_y;
  }
 */
Frame2d operator*(const Frame2d &f1, const Frame2d &f2) {
	Frame2d res;
	res.scale= f1.scale* f2.scale;


	res.n_x= f1.n_x*f2.n_x - f1.n_y*f2.n_y;
	res.n_y= f1.n_y*f2.n_x + f1.n_x*f2.n_y;
	res.p_x= f1.n_x*f2.p_x - f1.n_y*f2.p_y + f1.p_x;
	res.p_y= f1.n_y*f2.p_x + f1.n_x*f2.p_y + f1.p_y;
	return res;
}

Vec operator*(const Frame2d &f1, const Vec &v) {
	Vec res;
	res.x= f1.n_x*v.x - f1.n_y*v.y + f1.p_x;
	res.y= f1.n_y*v.x + f1.n_x*v.y + f1.p_y;
	return res;
}


Frame2d::Frame2d() {
	scale= 1.0;


	n_x= 1.0;
	n_y= 0.0;
	p_x= 0.0;
	p_y= 0.0;
}

Frame2d::Frame2d (Vec p, Angle a) {
	scale=1.0;
	set_angle (a);
	set_position (p);
}

Angle Frame2d::get_angle() const {
	Vec vec(n_x,n_y);
	return vec.angle();
}

Vec Frame2d::get_heading() const {
	return (Vec(n_x,n_y)).normalize();
}

void Frame2d::set_angle(const Angle& a) {
	n_x= cos(a.get_rad()) * scale;
	n_y= sin(a.get_rad()) * scale;

	//f1.n_y= sqrt(1-f1.n_x*f1.n_x);
}
void Frame2d::set_angle(Vec v) {
	double a=v.angle().get_rad();
	n_x= cos(a) * scale;
	n_y= sin(a) * scale;

	//f1.n_y= sqrt(1-f1.n_x*f1.n_x);
}


void Frame2d::set_position(double x,double y) {
	p_x= x;
	p_y= y;
}

void Frame2d::set_position(Vec v) {
	p_x= v.x;
	p_y= v.y;
}

void Frame2d::set_scale(double s) {
	n_x /= scale;
	n_y /= scale;
	scale= s;


	n_x *= scale;
	n_y *= scale;
}

void Frame2d::invert() {
	double old_p_x= p_x;

	p_x= -n_x*p_x     - n_y*p_y;
	p_y=  n_y*old_p_x - n_x*p_y;

	n_y= -n_y;

	scale = 1/scale;
}

Frame2d Frame2d::Translation(double x, double y) {
	Frame2d f1;
	f1.set_position(x,y);
	return f1;
}
Frame2d Frame2d::Rotation(const Angle& a) {
	Frame2d f1;
	f1.set_angle(a);
	return f1;
}

/*****************************************************************************/
#if 0

using namespace Tribots;
using namespace std;

int main()
{
	Vec a(0,-1);
	Vec b(0,1);
	Angle winkel;

	cout <<"A: "<<a <<"B"<<b<<endl;
	winkel=a.angle(b);
	cout <<"Winkel: "<<winkel.get_deg()<<endl;


	cout<<"Welcome to the Frame2d Test"<<endl;
	Vec pos(1000,500); Vec head(1,0) ;Vec c(10,20);
	Vec look,look2;
	head=head.rotate(Angle::deg_angle(45));
	look=pos+c.rotate(head.angle());

	Frame2d f;
	f.set_position(pos.x,pos.y);
	f.set_angle(head);
	look2=f*c;

	cout <<"pos: "<<pos<<" head: "<<head<<c <<f<<endl;
	cout <<"Look V2d berechnung:"<<look<<endl;
	cout <<"Look V2d berechnung:"<<look2<<endl;

	cout << "Hintereinanderausf�hren von von Frames"<<endl;
	Frame2d f1,f2,f3;
	Vec y(3,3);
	f1.set_position(10,10);
	f2.set_position(20,10);

	f3=f1*f2;
	cout <<f1<<f1*y<<f2<<f2*y<<f3<<f3*y<<endl;


	return 0;
}


#endif

}
}
