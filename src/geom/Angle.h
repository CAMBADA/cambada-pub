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


#ifndef Angle_h
#define Angle_h

#include <math.h>

namespace cambada {
namespace geom {

/** Class for representation and computation of angles */
class Angle {
private:
	/** Angle in radians normalized between [0,2pi) */
	double the_angle;
public:
	/** Default Constructor */
	Angle () throw () { the_angle = 0.;}
	/** Constructor to set angle (in radians) */
	Angle (double a) throw () {set_rad (a);}
	/** Copy-Constructor */
	Angle (const Angle& a) throw () : the_angle(a.the_angle) {;}
	/** Assignment operator FIXME (CAMBADA) normalize the assigment */
	const Angle& operator= (const Angle& a) throw () { the_angle=a.the_angle; return *this; }

	/** Comparison operator */
	bool operator== (const Angle&) const throw ();
	bool operator!= (const Angle&) const throw ();
	/** TRUE if first Angle lies between the Angle in second argument */
	bool in_between (const Angle, const Angle) const throw ();

	/** Set the angle in radians */
	void set_rad (double) throw ();
	/** Angle in radians normalized between [0,2pi) */
	double get_rad () const throw ();

	/** produce an angle directly in rad */
	static Angle rad_angle (double) throw ();
	/** produce an angle directly in degrees */
	static Angle deg_angle (double) throw ();

	/** Set the angle in degrees */
	void set_deg (double) throw ();
	/** Angle in degrees normalized between [0,360) */
	double get_deg () const throw ();

	/** Angle in radians normalized between [-pi,pi) */
	double get_rad_pi () const throw ();
	/** Angle in degrees normalized between [-180,180) */
	double get_deg_180 () const throw ();

	/** Addition */
	Angle operator+ (const Angle) const throw ();
	const Angle& operator+= (const Angle) throw ();
	/** Subtraction */
	Angle operator- (const Angle) const throw ();
	const Angle& operator-= (const Angle) throw ();
	/** Complement (2pi-this) */
	Angle operator- () const throw ();
	/** Scalar Multiplying */
	const Angle& operator*= (double) throw();

	/** Normalization function */
	void normalize()
	{
		while( the_angle > 0.0 )
			the_angle -= M_2_PI;

		while( the_angle < 0.0 )
			the_angle += M_2_PI;
	}


	// Spezielle Winkel:
	static const Angle zero;           ///< 0 Grad
	static const Angle twelvth;        ///< 30 Grad
	static const Angle eighth;         ///< 45 Grad
	static const Angle sixth;          ///< 60 Grad
	static const Angle quarter;        ///< 90 Grad
	static const Angle three_eighth;   ///< 135 Grad
	static const Angle half;           ///< 180 Grad
	static const Angle five_eighth;    ///< 225 Grad
	static const Angle three_quarters; ///< 270 Grad
	static const Angle five_sixth;     ///< 300 Grad
	static const Angle seven_eighth;   ///< 315 Grad
	static const Angle eleven_twelvth; ///< 330 Grad
};

/** Scalar multiplication */
Angle operator* (const Angle, double) throw ();
Angle operator* (double, const Angle) throw ();

}
}

#endif

