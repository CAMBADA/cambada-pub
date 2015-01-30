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

#ifndef POSITIONKF_H_
#define POSITIONKF_H_

#include <Eigen/Dense>
#include <iostream>

#include "Angle.h"

using namespace std;
using namespace cambada::geom;

namespace cambada {
namespace loc {

class PositionKF
{
public:
	PositionKF();
	virtual ~PositionKF();
	void predict(const Eigen::Vector3f& control,const Eigen::Matrix3f& R);
	void update(const Eigen::Vector3f& measure,const Eigen::Matrix3f& Q);
	void filter(const Eigen::Vector3f& control, const Eigen::Vector3f& measure, const Eigen::Matrix3f& R, const Eigen::Matrix3f& Q);
	void get(Eigen::Vector3f& pose);
	void reset();
	void set(Eigen::Vector3f& state, Eigen::Matrix3f& covar);

private:
	Eigen::Matrix3f A, B, C, covar;
	Eigen::Vector3f state;
	bool firstTime;

};

}
}

#endif /* POSITIONKF_H_ */
