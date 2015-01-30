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


#ifndef tribots_robot_location_h
#define tribots_robot_location_h

#include "Angle.h"
#include "Vec.h"

using namespace cambada::geom;

namespace cambada {
namespace loc {


  /** Struktur, um Blockadesituationen zu beschreiben; alle Angaben in WELTKOORDINATEN */
  struct RobotStuckLocation {
    bool robot_stuck;                ///< Roboter blockiert?
    unsigned int msec_since_stuck;   ///< Zeit in msec seit letztem Mal, als eine Blockierung festgestellt wurde
    Vec pos_of_stuck;                ///< Position, an der sich der Roboter bei letzter Blockierung befunden hat
    Vec dir_of_stuck;                ///< Richtung, in die der Roboter bei letzter Blockierung fahren wollte
    inline bool operator() () const throw () { return robot_stuck; }  ///< Abfrage des "robot_stuck"-Attributs
  };


  /** Struktur, um die derzeitige Roboterposition auf dem Spielfeld zu repraesentieren 
      alle Koordinaten (in WELTKOORDINATEN) sind relativ zur aktuellen Spielrichtung:
      (0,0) ist der Mittelpunkt des Feldes
      die y-Achse zeigt in Richtung des gegnerischen Tores
      die x-Achse zeigt rechtwinklig dazu (rechtshaendisches Koordinatensystem)
      ein Winkel von 0 bezeichnet die Parallele zur x-Achse
      alle Laengenangaben in mm, alle Winkelangaben in rad 
      alle Geschwindigkeiten in m/s bzw. rad/s */
  struct RobotLocation {
    Vec pos;                  ///< Roboterposition
    Angle heading;            ///< Verdrehung des Roboters, 0=Roboter ist in Richtung gegnerisches Tor ausgerichtet
    Vec vtrans;               ///< translatorische Robotergeschwindigkeit in m/s
    double vrot;              ///< rotative Geschwindigkeit des des Roboters in rad/s

    double quality;           ///< Qualitaet der Positions- und Geschwindigkeitsschaetzung: 0=unzuverlaessig, 1=zuverlaessig

    bool kick;                ///< Kicker aktiv?

    RobotStuckLocation stuck; ///< Information, ob und wie Roboter blockiert ist
  };

}
}

#endif

