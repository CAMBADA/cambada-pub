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


#ifndef _Tribots_geometry_h_
#define _Tribots_geometry_h_

#include <cmath>
#include <stdexcept>
#include <vector>

#include "Frame2D.h"
#include "Vec.h"

using namespace std;

namespace cambada {
namespace geom {

	class Area;
  class Line;
  class LineSegment;
  class Circle;
  class Arc;
  class Triangle;
  class XYRectangle;
  class Quadrangle;
  class Halfplane;

  /* Objekte mit Frame2d multiplizieren (Bewegung) */
  Line operator* (const Frame2d&, const Line&) throw ();
  LineSegment operator* (const Frame2d&, const LineSegment&) throw ();
  Arc operator* (const Frame2d&, const Arc&) throw ();
  Circle operator* (const Frame2d&, const Circle&) throw ();
  Triangle operator* (const Frame2d&, const Triangle&) throw ();
  Quadrangle operator* (const Frame2d&, const Quadrangle&) throw ();
  Halfplane operator* (const Frame2d&, const Halfplane&) throw ();

  /** Schnittpunkt zweier Geraden; bei parallelen Geraden wird Ausnahme geworfen */
  Vec intersect (const Line&, const Line&) throw (std::invalid_argument);
  /** Schnittpunkte zwischen Geraden/Geradenstuecken */
  std::vector<Vec> intersect (const LineSegment&, const Line&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const Line&, const LineSegment&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const LineSegment&, const LineSegment&) throw (std::bad_alloc);
  /** Schnittpunkte zwischen Gerade und Kreislinie */
  std::vector<Vec> intersect (const Line&, const Circle&) throw (std::bad_alloc);
  inline std::vector<Vec> intersect (const Circle& c, const Line& l) throw (std::bad_alloc) { return intersect (l,c); }
  /** Schnittpunkte zweier Kreise; bei konzentrischen Kreisen wird Ausnahme geworfen */
  std::vector<Vec> intersect (const Circle&, const Circle&) throw (std::bad_alloc);
  /** Schnittpunkte zwischen Gerade und Kreisbogen */
  std::vector<Vec> intersect (const Line&, const Arc&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const Arc&, const Line&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const LineSegment&, const Arc&) throw (std::bad_alloc);
  std::vector<Vec> intersect (const Arc&, const LineSegment&) throw (std::bad_alloc);
  /** Tangentiale Punkte berechnen; wirft Ausnahme, falls Pount innerhalb des Kreises */
  std::vector<Vec> tangent_point (const Circle&, const Vec&) throw (std::bad_alloc, std::invalid_argument);


  /** ein Bereich der zweidimensionalen Ebene */
  class Area {
  public:
    virtual ~Area() throw () {;}
    /** pruefen, ob Argument in der Menge enthalten ist */
    virtual bool is_inside (Vec) const throw () =0;
  };


  /** Klasse Line modelliert eine Gerade im 2-dimensionalen */
  class Line {
  protected:
    friend Vec intersect (const Line&, const Line&) throw (std::invalid_argument);
    friend std::vector<Vec> intersect (const Line&, const Circle&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const Line&, const Arc&) throw (std::bad_alloc);
    friend Vec perpendicular_point (const Vec&, const Line&) throw ();
    friend Line operator* (const Frame2d&, const Line&) throw ();

  public:
    Vec p1;   // p1, p2: 2 Punkte auf der Linie (nicht identisch!)
    Vec p2;
    Line () throw ();
    /** Konstruktor, uebergeben werden zwei Punkte der Linie;
	wirft eine invalid_argument Ausnahme, wenn beide Punkte identisch sind */
    Line (const Vec, const Vec) throw (std::invalid_argument);
    /** Copy-Konstruktor */
    Line (const Line&) throw ();
    /** Zuweisungsoperator */
    const Line& operator= (const Line&) throw ();

    /** liefert den Abstand eines Punktes zur Geraden */
    double distance (const Vec) throw ();
    /** liefert die Seite, auf der Punkt bezueglich Geraden liegt
	Orientierung der Geraden von p1 nach p2;
	liefert -1 falls Punkt links liegt, 0 falls Punkt auf Geraden liegt, +1 falls Punkt rechts der Geraden liegt */
    int side (const Vec) throw ();
    /** Lotpunkt berechnen */
    Vec perpendicular_point (const Vec&) throw ();

    /** Rotation um Ursprung */
    Line& s_rotate   (const Angle&) throw();
    /** Translation */
    Line& s_translate(const Vec&) throw();

    /** Rotation um Ursprung */
    Line rotate (const Angle) const throw ();
    /** Translation */
    Line translate (const Vec) const throw ();

    bool isDef();

    static const Line def;             /// Default dummy line
  };
    

  /** Klasse LineSegment modelliert ein Linienstueck mit Anfangs- und Endpunkt */
  class LineSegment {
    friend std::vector<Vec> intersect (const LineSegment&, const Arc&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const LineSegment&, const Line&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const LineSegment&, const LineSegment&) throw (std::bad_alloc);
    friend LineSegment operator* (const Frame2d&, const LineSegment&) throw ();

  public:
    Vec p1;   // Anfangspunkt
    Vec p2;   // Endpunkt

    LineSegment () throw ();
    /** Konstruktor, uebergeben werden Anfangs- und Endpunkt;
	wirft eine invalid_argument Ausnahme, wenn beide Punkte identisch sind */
    LineSegment (const Vec, const Vec) throw (std::invalid_argument);
    /** Copy-Konstruktor */
    LineSegment (const LineSegment&) throw ();
    /** Zuweisungsoperator */
    const LineSegment& operator= (const LineSegment&) throw ();

    /** liefert den Abstand eines Punktes zum Linienstueck */
    double distance (const Vec) throw ();

    Vec closestPoint (const Vec p) throw (); 

    /** liefert den Anfangspunkt des Linienstuecks */
    const Vec& getStart() const throw();
    
    /** liefert den Endpunkt des Linienstuecks */
    const Vec& getEnd() const throw();

    /** Rotation um Ursprung */
    LineSegment& s_rotate   (const Angle&) throw();
    /** Translation */
    LineSegment& s_translate(const Vec&) throw();
    /** Rotation um Ursprung */
    LineSegment rotate (const Angle) const throw ();
    /** Translation */
    LineSegment translate (const Vec) const throw ();
  };    


  /** Klasse modelliert einen Kreisbogen */
  class Arc {
  private:
    friend std::vector<Vec> intersect (const Line&, const Arc&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const LineSegment&, const Arc&) throw (std::bad_alloc);
    friend Arc operator* (const Frame2d&, const Arc&) throw ();

    Vec center;
    double radius;
    Angle start;
    Angle end;
  public:
    Arc () throw ();
    /** Konstruktor */
    Arc (Vec, double, Angle, Angle) throw ();
    /** Copy-Konstruktor */
    Arc (const Arc&) throw ();
    /** Zuweisungsoperator */
    const Arc& operator= (const Arc&) throw ();

    /** Abstand berechnen */
    double distance (const Vec) throw ();

	// CMBD
	bool is_inside(Vec p);
  };


  /** Klasse Circle modelliert einen Kreis im 2-dimensionalen */
  class Circle : public Area {
  private:
    friend std::vector<Vec> intersect (const Line&, const Circle&) throw (std::bad_alloc);
    friend std::vector<Vec> intersect (const Circle&, const Circle&) throw (std::bad_alloc);
    friend std::vector<Vec> tangent_point (const Circle&, const Vec&) throw (std::bad_alloc, std::invalid_argument);
    friend Circle operator* (const Frame2d&, const Circle&) throw ();

  public:
    Vec center;     // Mittelpunkt
    double radius;  // Radius
    Circle () throw ();
    /** Konstruktor, uebergeben werden Mittelpunkt und Radius */
    Circle (const Vec, double) throw ();
    /** Konstruktor, uebergeben werden drei Punkte des Kreises; bei kollinearen Punkten wird invalid_argument geworfen */
    Circle (const Vec, const Vec, const Vec) throw (std::invalid_argument);
    /** Copy-Konstruktor */
    Circle (const Circle&) throw ();
    /** Zuweisungsoperator */
    const Circle& operator= (const Circle&) throw ();

    /** liefere Mittelpunkt */
    const Vec& get_center () const throw ();
    /** liefere Radius */
    double get_radius () const throw ();

    /** prueft, ob ein Punkt innerhalb des Kreises liegt (einschliesslich Rand) */
    bool is_inside (const Vec) const throw ();  
    /** liefert den Abstand eines Punktes zur Kreislinie, fuer innere Punkte -> negative Werte */
    double distance (const Vec) const throw ();
  };


  /** Klasse Triangle modelliert ein Dreieck */
  class Triangle : public Area {
  public:
    Triangle () throw ();
    Triangle (Vec,Vec,Vec) throw ();
    Triangle (const Triangle&) throw ();
    bool is_inside (Vec) const throw ();
    Vec p1, p2, p3;
    
    friend Triangle operator* (const Frame2d&, const Triangle&) throw ();
  };


  /** Klasse XYRectangle fuer achsenparalleles Rechteck */
  class XYRectangle : public Area {
  public:
    XYRectangle () throw ();
    XYRectangle (const XYRectangle&) throw ();
    /** Konstruktor, Argumente: 2 diagonale Eckpunkte */
	// top left , lower rifght
	XYRectangle (Vec,Vec) throw ();
    /** pruefen, ob ein Punkt innerhalb liegt (einschliesslich Rand) */
    bool is_inside (const Vec) const throw ();
    
	// return the adjustment of the vector p to XYRectangle boundry
	// adjust to lower y face
	Vec adjust(Vec p);
	
	
	Vec p1;
    Vec p2;
  };

  
  /** Klasse Quadrangle fuer ein konvexes Viereck */
  class Quadrangle : public Area {
  public:
    Quadrangle () throw ();
    Quadrangle (const Quadrangle&) throw ();
    /** Konstruktor, Funktion wie bei Rectangle */
    Quadrangle (Vec, Vec) throw ();
    /** Konstruktor, ubergeben werden zwei Punkte sowie die Breite des Korridors */
    Quadrangle (const Vec&, const Vec&, double) throw ();    
    /** Konstruktor, ubergeben werden zwei Punkte sowie die Anfangs und Endbreite des Korridors */
    Quadrangle (const Vec&, const Vec&, double,double) throw ();    
    /** Konstruktor, ubergeben werden die Eckpunkte umlaufend um das Viereck */
    Quadrangle (Vec, Vec, Vec, Vec) throw ();
    bool is_inside (const Vec) const throw ();
    Vec p1, p2, p3, p4;

    friend Quadrangle operator* (const Frame2d&, const Quadrangle&) throw ();
  }; 
 

  /** Klasse Halfplane fuer eine Halbebene */
  class Halfplane : public Area {
  public:
    Halfplane () throw ();
    Halfplane (const Halfplane&) throw ();
    /** Konstruktor mit einem Punkt auf der Grenze (Arg1) und einem Normalenvektor (Arg2), der in das innere weist */ 
    Halfplane (Vec, Vec) throw ();
    /** Konstruktor mit einem Punkt auf der Grenze (Arg1) und einem Winkel (Arg2);
	die innersen Punkte der Halbebene sind diejenigen von denen man von Arg1 in einem Winkel
	zwischen arg2 und arg2+180Grad */
    Halfplane (Vec, Angle) throw ();
    bool is_inside (Vec) const throw ();
  private:
    Vec p1;
    Vec norm;

    friend Halfplane operator* (const Frame2d&, const Halfplane&) throw ();
  };

}
}

#endif

