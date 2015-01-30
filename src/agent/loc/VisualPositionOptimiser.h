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


#ifndef Tribots_VisualPositionOptimiser_h_
#define Tribots_VisualPositionOptimiser_h_

//#include "VisibleObject.h"
#include "FieldLUT.h"
#include <vector>
#include <list>
using namespace std;
using namespace cambada::geom;

typedef vector<Vec> VisibleObjectList;

namespace cambada {
namespace loc {

 /** Class makes the optimization for a robot position possible on 
  * the basis the visual information with the help 
  * of an iterative optimization beginning */ 
class VisualPositionOptimiser {
  private:
    const FieldLUT& the_field_lut;  /// Referenz auf FieldLUT, die verwendet werden soll
    double c;                         /// width parameter of error function 1-(c*c)/(c*c+x*x)
    double c2;                      /// c*c, for short
    double d2;                     /// Widths parameter^2 for spacer weights 
    std::vector<double> weights;   /// Weight matrix for each line segment 
    
  //protected:
  public:
   
	/** compute the error and the derivative of the error,          
	 * Arguments: Error (return), derivative after x (return), 
	 * derivative after y (return), after derivative phi (return), 
	 * x, y, phi (in wheel), list with seen lines, number of max. 
	 * considering line segments   
	 * Coordinates in absolute Cartesian coordinate system with uniform: y axis points to blue gate   
	 * Convention: Weight array ???weights??? must have been set before */ 
    void error (double&, double&, double&, double&, double, double, double, const VisibleObjectList&, unsigned int) const throw ();
  public:
    /** Kostruktor, uebergeben wird FieldLUT, Breite der Fehlerverteilung und Breite der Entfernunggewichtsfunktion */
    VisualPositionOptimiser (const FieldLUT&, double c1, double d1) throw ();
    /** Distance weights compute; 
	 * arg1: Line segments   
	 * arg2: Number of max. considering line segments Return: 
	 *		Sum of the weights */
    double calculate_distance_weights (const VisibleObjectList&, unsigned int =10000) throw ();
    
	/** Minimize the error with RPR-OIwell-behaved update    Result return over arguments   
	 * arg1: At the beginning of and final position x   
	 * arg2: At the beginning of and final position phi  
	 * arg3: List of wise lines. Here ONLY WAY may be contained LINES!   
	 * arg4: Number of iterations    
	 * arg5: Number of max. considering line segments   
	 * Return: Error before last iteration of the Optimierers    
	 * Convention: Distance weights must have been computed before */
    double optimise (Vec&, Angle&, const VisibleObjectList&, unsigned int, unsigned int =10000) const throw ();
    
	/** Compute curvature of the error (2. Derivative) at a
	 *	position, acceptance: Hessian matrix possesses diagonal form result return over arguments   
	 * arg1: d2err/(dx) ^2 and d2err/(dy) ^2   
	 * arg2: d2err/(dphi) ^2   
	 * arg3: (x, y)   
	 * arg4: phi   
	 * arg5: List of wise lines. Here ONLY WAY may be contained LINES!   
	 * arg6: Number of max. considering line segments  
	 * Return value: Error at the position concerned   
	 * Convention: Distance weights must have been computed before */
	double analyse (Vec&, double&, Vec, Angle, const VisibleObjectList&, unsigned int =10000) const throw ();
  };

}
}

#endif
