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


#ifndef _Tribots_FieldLUT_h_
#define _Tribots_FieldLUT_h_

//#include "FieldGeometry.h"
#include "Vec.h"
#include "ConfigXML.h"

using namespace cambada::util;
using namespace cambada::geom;

namespace cambada {
namespace loc {

/** Class FieldLUT models a Look UP table for the storage of minimum
    Distances to white lines           
		NOTE: FieldLUT uses its own coordinate system independently of the play direction. 
		Origin is the playing field center
    the positive y axis points toward the blue gate */
class FieldLUT
{
public:
	/** Constructor is handed over, field geometry as well as the cell size into mm */
	FieldLUT ( ConfigXML* config , /*const FieldGeometry&,*/ unsigned int); //throw (std::bad_alloc);
	/** Destruktor */
	~FieldLUT () throw ();

	/** for the point arg1 in the FieldLUT coordinate system the minimum distance look up */
	double distance (const Vec&) const throw ();

	/** the gradients of the distance function at point arg1 in the FieldLUT coordinate system look up */
	Vec gradient (const Vec&) const throw ();

private:
	unsigned int x_res;                                // dissolution in x-direction (1/2 number of cells)
	unsigned int y_res;                                // dissolution in y-direction (1/2 number of cells)
	unsigned int cell_size;                            // Cell size (edge length) in mm

	double* array;                                     //The cell array with distance values in mm (only for positive quadrants)
	Vec* grad;                                // the gradient at each position

	double error_outside;                              // error value for positions more auser half

	void draw_line_segment (Vec, Vec);                 // a line segment consider
	void draw_arc (Vec, double, Angle, Angle);         // // a circular arc consider n
	void draw_dot (Vec);                               // one point consider
	void update (unsigned int, unsigned int, double);  // sets the array entry (arg1, arg2) to value min (arg3, past entry)
};

}
}

#endif
