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

#ifndef __ROBOTWIDGET_H
#define __ROBOTWIDGET_H

#include "ui_robotwidget.h"
#include "Robot.h"
#include "DB_Robot_info.h"
#include "RobotDialog.h"
#include <QTimer>
#include <QDialog>

#include <iostream>

#include <QThread>



class MyThread : public QThread
{

public:
	MyThread() {};
	virtual ~MyThread(){};


	void setMyThread( QString cmd, int number )
	{
		this->cmd = cmd;
		this->number = number;
	}

	virtual void run()
	{
		//cerr << cmd.toAscii() << "  :  ";
		//cerr << system( cmd.toAscii() ) << endl;

		system( cmd.toAscii() );
	}

private:
	QString cmd;
	int number;
};

//============================================================ RWidget ==============================================

class RWidget : public QWidget, public Ui::RobotWidget
{

	Q_OBJECT


public:
	RWidget(QWidget *partent = 0);
	~RWidget();
	void changeRobotStatus(QString *status);
	void get_info_pointer( DB_Robot_Info * rw);
	void get_robot_number(int num);
	void get_coach_pointer( DB_Coach_Info * ci);

private:

	DB_Robot_Info *DB_Info; //informação da base de dados
	DB_Coach_Info *db_coach_info;
	int my_number;
	char NA_flag;
	
	QTimer *UpdateTimer;

	QDialog *RobotDetailsDialog;
	RobotDialog *RDial;

	QString *KO;
	QString *NA;
	QString *OK;
	QString *SB;

	QColor stat_Red;
	QColor stat_Green;
	QColor stat_White;
	QColor stat_blue;

	bool RunBotFlip;

	MyThread start;
	MyThread stop;
	MyThread agent;
	MyThread Monitor_on;
	MyThread Monitor_off;

//	bool Started_flag;
	bool lock_flag;

	

public slots:
	void runBotPressed (void);
	void changeRobotRun(void);
	void changeRobotStop(void);
	void updateInfo(void);
	void ChangeTextSize(int size);
	void RoleChanged(int role_id);
	void TeamColorChanged(int color_id);
	void GoalColorChanged(int color_id);

	void startBotPressed(void);
	void stopBotPressed(void);
	void agentBotPressed(void);
	void monitor_on_Pressed(void);
	void monitor_off_Pressed(void);

	void detailsBotPressed (void);
	void relocBotPressed (void);

	void lockRobot(int lock);

	void changeBackGroundColor(QString *status);

signals:
	void transmitCoach(void);
	
};

#endif
