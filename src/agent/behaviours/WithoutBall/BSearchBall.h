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

#ifndef BSEARCHBALL_H_
#define BSEARCHBALL_H_

#include "Behaviour.h"

namespace cambada {

/**
 * \brief Behaviour that searches the ball in the field
 *
 * IC: Ball not visible
 * CC: Ball not visible
 */
class BSearchBall: public cambada::Behaviour {
public:
	BSearchBall();
	virtual ~BSearchBall();

	virtual void gainControl();
	virtual bool checkInvocationCondition();
	virtual bool checkCommitmentCondition();
	void calculate(DriveVector* dv);

private:
	unsigned int currentIdx;
	vector<Vec> fieldPts;		// Tour points in field
};

} /* namespace cambada */
#endif /* BSEARCHBALL_H_ */
