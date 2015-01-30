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


#include "VisualPositionOptimiser.h"
#include <cmath>

using namespace std;
 
#define DEBUG_VISUALOPTIMISER 0

namespace cambada {
namespace loc {

VisualPositionOptimiser::VisualPositionOptimiser (const FieldLUT& fl, double c1, double d1) throw () : 
	the_field_lut (fl), c(c1), c2(c1*c1), d2(d1*d1), weights (300) {;}

double VisualPositionOptimiser::calculate_distance_weights (const vector< Vec >& lines, unsigned  int max_lines) throw () 
{
  // GUS unsigned int nlines=(max_lines>lines.size() ? lines.size() : max_lines);
  
	unsigned int nlines = (max_lines > lines.size() ? lines.size() : max_lines);
	double ret=0;
	
	weights.resize (nlines);
  
	double reference = 1500 * 1500;
  // GUS vector<VisibleObject>::const_iterator vis_it = vis.begin();
  // TODO vector<Vec>::const_iterator vis_it = vis.begin();
  //vector<double>::iterator weight_it = weights.begin();
  
	for (unsigned int i=0; i<nlines; i++) 
	{
		ret += ((weights[i]) = (reference + d2) / (d2 + lines[i].squared_length()));
	}
 
	return ret;
}

void VisualPositionOptimiser::error (double& err, double& dx, double& dy, double& dphi, double x, double y, double phi, const vector< Vec >& lines, unsigned int max_lines) const throw () 
{
	unsigned int nlines = (max_lines > lines.size() ? lines.size() : max_lines);
	double sinphi = sin (phi);
	double cosphi = cos(phi);

	err = dx = dy = dphi = 0.0;
  
	//vector<VisibleObject>::const_iterator vis_it = vis.begin();
	//vector<VisibleObject>::const_iterator vis_itend = vis.begin()+nlines;
	//vector<double>::const_iterator weight_it = weights.begin();
  
	// while (vis_it < vis_itend) {
	for( unsigned int i = 0; i < nlines; i++ )	
	{
		Vec vp (x + cosphi * lines[i].x - sinphi * lines[i].y, y + sinphi * lines[i].x + cosphi * lines[i].y);	// seen wise line in 
																												// absolute Cartesian coordinates
	    double dist = the_field_lut.distance (vp);   // Distance seen line <-> next model line
		
	    double ef = c2 + dist * dist;
		
	    err += weights[i] * (1 - c2 / ef);			// Error portion compute
		
	    double derrddist = (2 * c2 * dist) / (ef * ef);	// Derivative of the error function after the distance
		
	    Vec ddistdpos = the_field_lut.gradient (vp);   // Derivative of the distance function after the position
		
	    dx += weights[i] * derrddist * ddistdpos.x;   // Gradient: x-portion
	    dy += weights[i] * derrddist * ddistdpos.y;   // Gradient: y-portion
	    dphi += weights[i] * derrddist * (ddistdpos.x * (-sinphi * lines[i].x - cosphi * lines[i].y) + ddistdpos.y * (cosphi * lines[i].x - sinphi * lines[i].y));   // Gradient: phi-portion
    //vis_it++;
	//weight_it++;
	}
}

//double VisualPositionOptimiser::optimise (Vec& xy, Angle& h, const VisibleObjectList& vis, unsigned int niter, unsigned int max_lines) const throw () 
double VisualPositionOptimiser::optimise (Vec& xy, Angle& h, const vector< Vec >& lines, unsigned int niter, unsigned int max_lines) const throw () 
{
	double param [3];		// parameters which can be optimized
	param[0]=xy.x;
	param[1]=xy.y;
	param[2]=h.get_rad();
	double grad [3];  						// Gradient
	double stepwidth [3] = {40, 40, 0.1};	// Incrementations
	double latest_grad [3] = {0, 0, 0};		// last gradient
	double err; 							// error value

	double best_x=1e6, best_y=1e6, best_head=1e6;
	double best_error = 1e6;

	for (unsigned int i = 0; i < niter; i++) 
	{
    	error (err, grad[0], grad[1], grad[2], param[0], param[1], param[2], lines, max_lines);

	    //double delta_angle = (h-Angle::rad_angle(param[2])).get_rad_pi();
    	//err += 2.5 * delta_angle*delta_angle;
    	//grad[2] += 5 * delta_angle;

		if(err < best_error)
		{
			best_x = param[0];
			best_y = param[1];
			best_head = param[2];
			best_error = err;
		}

//	cout << "VisualPosOpt::optimise ite " << i << " err " << err;
//	printf("  x y h : %f %f %f\n", param[0],param[1],param[2]);
	
#if DEBUG_VISUALOPTIMISER
    WorldModel::get_main_world_model().log_stream() << "VisualOptimiser: " << param[0] << ' ' << param[1] << ' ' << param[2]*180/M_PI << ' ' << grad[0] << ' ' << grad[1] << ' ' << grad[2] << '\n';
#endif

		for (unsigned int j=0; j<3; j++) 
		{
			// make updates for each parameter
			if (grad[j]==0)
				latest_grad[j]=0;
			else {
				// Incrementation adjustment
				if (grad[j] * latest_grad[j] > 0)
					stepwidth[j]*=1.2;
				else if (grad[j] * latest_grad[j] < 0)
					stepwidth[j] *= 0.5;
				latest_grad[j] = grad[j];

				// Adjustment of the parameters 
				if (grad[j] > 0)
		  			param[j] -= stepwidth[j];
				else if (grad[j] < 0)
				  param[j] += stepwidth[j];
      		}
		}
	}
/*
  xy.x=param[0];
  xy.y=param[1];
  h.set_rad(param[2]);
  return err;
*/
	xy.x = best_x;
	xy.y = best_y;
	h.set_rad(best_head);
	return best_error;
}

double VisualPositionOptimiser::analyse (Vec& hxy, double& hphi, Vec xy, Angle h, const vector< Vec >& lines, unsigned int max_lines) const throw (){
	unsigned int nlines=(max_lines > lines.size() ? lines.size() : max_lines);
	double phi = h.get_rad();
	double sinphi = sin (phi);
	double cosphi = cos(phi);
  
	double err = 0;		// error
	hxy.x = hxy.y = hphi = 0;
	double derr;			// error 1. Derivative
	double dderr;			// error 2. Derivative
	double dist;			// distance
	Vec ddist;	// distance 1. Derivative
	// 2. Derivative of the distance function after the position is accepted as constantly 0
	Vec pos;
	Vec dposdphi;
	Vec ddposdphi2;
  
	double t1, t2;  		// Helping sizes

	//vector<VisibleObject>::const_iterator vis_it = vis.begin();
	//vector<VisibleObject>::const_iterator vis_itend = vis.begin()+nlines;
	//vector<double>::const_iterator weight_it = weights.begin();
  
	//while (vis_it < vis_itend) 
	for( unsigned int i = 0; i < nlines; i++ )
	{
		pos.x = xy.x + cosphi * lines[i].x - sinphi * lines[i].y;
	
		pos.y =  xy.y + sinphi * lines[i].x + cosphi * lines[i].y;   // seen wise line in absolute Cartesian coordinates
    
		dist = the_field_lut.distance (pos);   // Distance seen line <-> next model line
	
		err += weights[i] * (1-c2/(c2+dist*dist));
	
		if (dist < 2 * c) 	// Heuristic, in order to adjust following cheating somewhat; Assumed: all points removed far are incorrect
		{  
			derr = dist / c2;		// cribbed: here the square error function comes into the play, there “err” even not positively definitely
			dderr = 1 / c2;		// dito
      
			ddist = the_field_lut.gradient (pos);   	// Derivative of the distance function after the position
	  
			dposdphi.x = -sinphi* lines[i].x - cosphi * lines[i].y;
			dposdphi.y = cosphi * lines[i].x - sinphi * lines[i].y;
			ddposdphi2.x = -cosphi*lines[i].x + sinphi * lines[i].y;
			ddposdphi2.y = -sinphi * lines[i].x - cosphi * lines[i].y;
      
			hxy.x += weights[i] * dderr * ddist.x * ddist.x;
			hxy.y += weights[i] * dderr * ddist.y * ddist.y;
			t1 = ddist.x * dposdphi.x + ddist.y * dposdphi.y;
			t2 = ddist.x * ddposdphi2.x + ddist.y * ddposdphi2.y;
			hphi += weights[i] * (dderr * t1 * t1 + derr * t2);
		}
	}

	return err;
}

}
}
