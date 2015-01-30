/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA AGENT
 *
 * CAMBADA AGENT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA AGENT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ZONES_H_
#define ZONES_H_

#include <vector>
#include "Field.h"
#include "geometry.h"

namespace cambada {

class Zones {

public:

    enum Zone { Zone1 = 0, Zone2, Zone3, Zone4, Zone5, Zone6, Zone7, Zone8, Zone9, Zone10 };

public:
    Zones(Field* field);
    ~Zones();

    double getZoneLength(Zone zone);
    double getZoneWidth(Zone zone);

    inline std::vector<cambada::geom::XYRectangle>& getZoneBoxes(void){ return this->zoneBoxes; }

private:

    double zoneLength[ Zone10 + 1 ];
    double zoneWidth [ Zone10 + 1 ];

    std::vector<cambada::geom::XYRectangle> zoneBoxes;
};

} /* namespace cambada */
#endif /* ZONES_H_ */
