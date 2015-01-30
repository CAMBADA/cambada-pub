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

#ifndef __REFBOXDIALOG_H
#define __REFBOXDIALOG_H

#include <QTcpSocket>
#include <QUdpSocket>
#include <QSocketNotifier>

//#include "RefBoxWidget.h"
#include "ui_RefBoxDialog.h"

#include "DB_Robot_info.h"

//#include "RefBoxXML.h"

class RefBoxDialog : public QDialog, public Ui::RefBoxDialog
{
	Q_OBJECT

public:
	RefBoxDialog(QDialog *parent=0);
	~RefBoxDialog();
	int connected;
	void get_coach_pointer( DB_Coach_Info * ci);

	void processNewRefBoxMsg();
	void processRefBoxMsg();

private:
	DB_Coach_Info *db_coach_info;

//	RefBoxXML* udpParser;

protected:
	QString destHost;
	quint16 destPort;
	QString interface;

	/* TCP Socket */
	QTcpSocket *socket;
	QUdpSocket *udpSocket;
	//QSocketNotifier *notifier;

	char data_received[1500];
	int before_stop_gamePart;


public slots:
	void connectToHost(void);
	void receiveRefMsg (void);

	void SetInterface(void);
	void SetHostAdd(void);
	void SetHostPort(int val);
	
	void update_manual_config(void);
	void apply_Button_pressed(void);

	void Timer_start_bot_pressed(void);
	void Timer_stop_bot_pressed(void);

signals:
	void transmitCoach(void);
	void changeGoalColor (int);
	void updateGameParam(void);

};

#endif
