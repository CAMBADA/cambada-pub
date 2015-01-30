/*
 * Copyright (c) 2002-2005, Neuroinformatics research group, 
 * University of Osnabrueck <tribots@informatik.uni-osnabrueck.de>
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


#ifndef _Tribots_random_h_
#define _Tribots_random_h_

#include "Vec.h"
#include <cstdlib>
#include <cmath>

  // Interfaces to different random-number generators

  /** set seed */
  inline void random_seed (const unsigned int& s) throw () {
    std::srand(s); }

  /** evenly distributed random number in [0,1] */
  inline double urandom () throw () {
    return static_cast<double>(std::rand())/RAND_MAX; }

  /** evenly distributed random number in [f, t] */
  inline double urandom (const double& f, const double& t) throw () {
    return (t-f)*urandom ()+f; }

  /** Bernoulli experiment with probability of success p */
  inline bool brandom (const double& p) throw () {
    return (urandom()<p); }

  /** Standard-normaldistributed random number */
  inline double nrandom () throw () {
    double u1 = urandom();
    double u2 = urandom();
    return (std::sqrt(-2.0*std::log(u1))*cos(6.2831853071*u2));
  }

  /** Normalverteilte Zufallszahl N(mu,sigma^2) */
  inline double nrandom (const double& mu, const double& sigma) throw () {
    return nrandom()*sigma+mu; }

  /** Standard-Normalverteilter Zufallszahlvektor */
  inline Vec n2random () throw () {
    double u1 = urandom();
    double u2 = urandom();
    double s = std::sqrt(-2.0*std::log(u1));
    return Vec (s*cos(6.2831853071*u2), s*sin(6.2831853071*u2));
  }


#endif
