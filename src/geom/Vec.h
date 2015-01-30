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


#ifndef VEC_H_
#define VEC_H_

#include "Angle.h"
#include <iostream>
#include <math.h>

namespace cambada {
namespace geom {

/** 2-dimensional Vector */
class Vec
{
public:
	float x;    ///< Value in 1. Dimension
	float y;    ///< Value in 2. Dimension

	/** Default Constructor */
	Vec () throw () { x=y=0.;}
	/** Constructor for directly setting the values */
	Vec (float x1, float y1) throw () : x(x1), y(y1) {;}
	/** Copy-Constructor */
	Vec (const Vec& v) throw () : x(v.x), y(v.y) {;}
	/** Assignment operator */
	const Vec& operator= (const Vec& v) { x=v.x; y=v.y; return *this; }

	/** Comparison operator - equal */
	bool operator== (const Vec) const throw ();
	/** Comparison operator - not equal */
	bool operator!= (const Vec) const throw ();

	/** Addition */
	Vec operator+ (const Vec) const throw ();
	const Vec& operator+= (const Vec) throw ();
	/** Subtraction */
	Vec operator- (const Vec) const throw ();
	const Vec& operator-= (const Vec) throw ();
	/** Negation */
	Vec operator- () const throw ();
	/** Scalar Multiplying */
	const Vec& operator*= (double) throw ();
	/** Division - Division by zero is not caught, is nan*/
	const Vec& operator/= (double) throw ();
	/** Scalar product */
	double operator* (const Vec) const throw ();

	/** Rotation by angle arg1, identical to rotate method (.) */
	Vec operator* (const Angle) const throw ();
	/** Rotation of (* this) by angle arg, identical method s_rotate (.) */
	const Vec& operator*= (const Angle) throw ();
	/** Rotation by angle -arg1 */
	Vec operator/ (const Angle) const throw ();
	/** Rotation of (* this) by angle -arg1 */
	const Vec& operator/= (const Angle) throw ();

	// Rotationen:

	Vec rotate (const Angle) const throw ();       ///< Rotation any angle
	Vec rotate_twelvth () const throw ();          ///< rotation 30 Grad
	Vec rotate_eleven_twelvth () const throw ();   ///< rotation -30=330 Grad
	Vec rotate_eighth () const throw ();           ///< rotation 45 Grad
	Vec rotate_seven_eighth () const throw ();     ///< rotation -45=315 Grad
	Vec rotate_sixth () const throw ();            ///< rotation 60 Grad
	Vec rotate_five_sixth () const throw ();       ///< rotation -60=300 Grad
	Vec rotate_quarter () const throw ();          ///< rotation 90 Grad
	Vec rotate_three_quarters () const throw ();   ///< rotation -90 Grad
	Vec rotate_half () const throw ();             ///< rotation 180 Grad=Point reflection in the origin

	Vec s_rotate (const Angle) throw ();           ///< Rotation of this Angle

	// Spiegelungen:

	Vec mirror (const Vec) const throw ();         ///< Reflection in an axis with a given direction when ||Arg1|| > 0, otherwise point reflection
	Vec mirror_x () const throw ();                ///< Reflection in x-axis
	Vec mirror_y () const throw ();                ///< Reflection in y-axis
	Vec mirror_eighth () const throw ();           ///< Reflection at 1. bisector
	Vec mirror_three_eighth () const throw ();     ///< Reflection at 2. bisector

	/** Squared length of the vector */
	double squared_length () const throw ();

	/** Length of the vector */
	double length () const throw ();
	/** Sets the length of the vector */
	Vec setLength(double len) const throw();

	/** Angle of the vector (0 for the null vector) from Y [-pi,pi] */
	Angle angleFromY() const throw();

	/** Angle of the vector (0 for the null vector) */
	Angle angle () const throw ();

	/** Normalize ((0,0) at the zero vector) */
	Vec normalize () const throw ();

	/** Angle between two vectors (NAN for the null vector) */
	Angle angle (const Vec) const throw ();

	/** Returns a vector of length 1 in a given direction */
	static Vec unit_vector (Angle) throw ();

	static const Vec unit_vector_x;           ///< Unit vector in the x-direction
	static const Vec unit_vector_y;           ///< Unit vector in the y-direction
	static const Vec zero_vector;             ///< Null vector

};

/** Scalar Multiplying */
Vec operator* (Vec, double) throw ();
Vec operator* (double, Vec) throw ();
Vec operator/ (Vec, double) throw ();    // Division durch 0 wird nicht abgefangen, ergibt nan

/** Linear Independence */
bool linearly_independent (const Vec, const Vec);

std::ostream& operator<< (std::ostream& os, const Vec& v);

}
}

#endif


