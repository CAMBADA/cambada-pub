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


#ifndef Tribots_RobotPositionKalmanFilter
#define Tribots_RobotPositionKalmanFilter

#include "Vec.h"
#include "RobotLocation.h"

using namespace cambada::geom;

namespace cambada {
namespace loc {

	/** Implementation of a Kalman filter for the robot position and - heading; 
	 * considered only the position, not the speed*/
	class RobotPositionKalmanFilter {
	private:
		Vec pos;			// Robot position in mm
		double heading;		// Heading in rad; Convention: heading always in the interval [- pi, pi]
		Vec var_pos;		// Variance of the robot position in mm^2
		double var_heading;	// Variance of the heading in rad^2
	public:
		/* Set initials for a position; one hands over: 
		 * arg1: Position in ABSOLUTE CARTESIAN COORDINATES 
		 * arg2: Heading 
		 * arg3: Variance of the position 
		 * arg4: Variance of the heading */
		void set (Vec, Angle, Vec, double) throw ();

		/* Updating the filter; one hands over: 
		 * arg1: Shift vector according to Odometrie/Travel vectors in ROBOT COORDINATES 
		 * arg2: Change of heading of gemaes Odometrie/ Travel vectors
		 * arg3: Position estimation from image processing in ABSOLUTE CARTESIAN COORDINATES 
		 * arg4: Heading from image processing 
		 * arg5: Variance of the image processing estimation (position) 
		 * arg6: Variance of the image processing estimation (heading) */
		void update (Vec, Angle, Vec, Angle,Vec,double) throw ();

		/* Update the filter without new picture information; one hands over: 
		 * arg1: Shift vector according to Odometrie/Travel vectors in ROBOT COORDINATES 
		 * arg2: Change of heading of gemaes Odometrie/Travel vectors 
		 * arg3: Is shift vector on 3/4 of its length to be shortened? 
		 * 		- > meaningfully only with following alignment with picture information */
		void update (Vec, Angle, bool =false) throw ();

		/* Position and heading query; Return over the arguments: 
		 * arg1: Position in ABSOLUTE CARTESIAN COORDINATES 
		 * arg2: Heading
		 * Return value: Quality of the position estimation between 0 (badly) and 1 (well) */
		double get (Vec&, Angle&) const throw ();

		/* The variances for the position supply (in mm^2) */
		Vec get_position_variance () const throw () { return var_pos; }

		/** The variances for the heading supply (in rad^2) */
		double get_heading_variance () const throw () { return var_heading; }

	
		/** The position reflect */
		void mirror () throw();
	};

}
}

#endif

