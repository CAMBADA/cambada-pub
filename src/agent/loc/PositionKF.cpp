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

#include "PositionKF.h"

namespace cambada {
namespace loc {

PositionKF::PositionKF()
{
	firstTime = true;
	A.setIdentity();
	B.setIdentity();
	C.setIdentity();

	state << 0, 0 ,0;
	covar << 0, 0, 0,
			 0, 0, 0,
			 0, 0, 0;
}

void PositionKF::predict(const Eigen::Vector3f& control,const Eigen::Matrix3f& R)
{
		state = A*state + B*control;
		state(2) = Angle(state(2)).get_rad_pi();

		covar = A*covar*A.transpose() + R;

		/*
		cout << "KF control " << endl << control << endl
			 << "KF R " << endl << R << endl
			 << "KF State " << endl << state << endl
			 << "KF Covar " << endl << covar << endl;
		*/
}

void PositionKF::update(const Eigen::Vector3f& measure,const Eigen::Matrix3f& Q)
{

	Eigen::Matrix3f K;
	Eigen::Vector3f residual;

	if(firstTime)
	{
		//cout << "KF firstTime " << endl;
		state = (measure);
		covar = Q;
		firstTime = false;
	}
	else
	{
		K = covar*C.transpose()*(C*covar*C.transpose() + Q).inverse();

		residual = (measure - C*state);
		residual(2) = Angle(residual(2)).get_rad_pi();
		state = state + K*residual;
		state(2) = Angle(state(2)).get_rad_pi();

		covar = (Eigen::Matrix3f::Identity() - K*C)*covar;
	}

	/*
	cout << "KF measure " << endl << measure << endl
		 << "KF Q " << endl << Q << endl
		 << "KF K " << endl << K << endl
		 << "KF residual " << endl << residual << endl
		 << "KF State " << endl << state << endl
		 << "KF Covar " << endl << covar << endl;
	*/
}

void PositionKF::filter(const Eigen::Vector3f& control, const Eigen::Vector3f& measure, const Eigen::Matrix3f& R, const Eigen::Matrix3f& Q)
{
	if(firstTime == false)
		predict(control, R);
	update(measure, Q);
}

void PositionKF::get(Eigen::Vector3f& pose)
{
	pose = state;
}

void PositionKF::reset()
{
	state << 0, 0 ,0;
	covar << 0, 0, 0,
			 0, 0, 0,
			 0, 0, 0;
	firstTime = true;
}

void PositionKF::set(Eigen::Vector3f& state, Eigen::Matrix3f& covar)
{
	this->state = state;
	this->covar = covar;
}

PositionKF::~PositionKF()
{
}

}
}
