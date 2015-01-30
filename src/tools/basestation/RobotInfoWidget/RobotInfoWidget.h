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

#ifndef __ROBOTINFOWIGDET_H
#define __ROBOTINFOWIGDET_H

#include "ui_RobotInfoWG.h"
#include "DB_Robot_info.h"
#include <QTimer>
#include <QEvent>

class RobotInfoWidget : public QWidget, public Ui::RBInfoWG
{
	Q_OBJECT

public:
	RobotInfoWidget(QWidget * parent=0);
	~RobotInfoWidget();
	void get_info_pointer( DB_Robot_Info * rw);
	void get_robot_number(int num);


private:
	DB_Robot_Info *DB_Info; //informação da base de dados
	int my_number;
	QTimer *UpdateTimer;
	double Pi;


public slots:
	void updateInfo(void);
	void ChangeTextSize(int size);
};

#endif
