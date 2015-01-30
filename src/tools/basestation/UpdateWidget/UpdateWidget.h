/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA BASESTATION
 *
 * CAMBADA BASESTATION is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA BASESTATION is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __UPDATEWIDGET_H
#define __UPDATEWIDGET_H

#include <QtGui>

#include "Robot.h"
#include "rtdb_api.h"
#include "rtdb_user.h"
#include "DB_Robot_info.h"


class UpdateWidget : public QWidget
{

	Q_OBJECT

private:
	bool valid_connection;
	QTimer *Update_timer;
	bool LogViewMode;

public: 
	UpdateWidget();
	virtual ~UpdateWidget();

	DB_Robot_Info Robots_info;
	DB_Coach_Info DB_Coach;

    QComboBox* formationCombo;
    void setFormationCombo(QComboBox * _formationCombo);
    void transmitFormation(int formationId);


	DB_Robot_Info * get_info_pointer(void);
	DB_Coach_Info * get_coach_pointer(void);

public slots:
	void UpdateInfo(void);
	void transmitCoach(void);
	void SetLogViewMode (bool on_off);

	

};

#endif
