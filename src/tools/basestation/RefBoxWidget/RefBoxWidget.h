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

#ifndef __REFBOXWIDGET_H
#define __REFBOXWIDGET_H

#include "ui_RefBoxWG.h"
#include "RefBoxDialog.h"
#include <QDialog>
#include <QTimer>

#include "DB_Robot_info.h"

class RefBoxWidget: public QWidget, public Ui::RefBoxWG
{
	Q_OBJECT

public:
	RefBoxWidget(QWidget * parent =0);
	~RefBoxWidget();

	void get_coach_pointer( DB_Coach_Info * ci);
	void get_info_pointer( DB_Robot_Info * rw);
	

private:
	DB_Coach_Info *db_coach_info;
	DB_Robot_Info *DB_Info;

	QDialog *RBDialog;
	RefBoxDialog *RBDial;
	QTimer *UpdateTimer;

	

public slots:
	void detailsBotPressed(void);

	void PlayOnPressed(void);
	void StopPressed(void);
	void HaltPressed(void);
	void DroppedBallPressed(void);
	void ParkingPressed(void);

	/* our */
	void OurKickOffPressed(void);
	void OurFreeKickPressed(void);
	void OurGoalKickPressed(void);
	void OurThrowinPressed(void);
	void OurCornerKickPressed(void);
	void OurPenaltyPressed(void);

	/* their */
	void TheirKickOffPressed(void);
	void TheirFreeKickPressed(void);
	void TheirGoalKickPressed(void);
	void TheirThrowinPressed(void);
	void TheirCornerKickPressed(void);
	void TheirPenaltyPressed(void);


	void updateStateInfo(void);

	void updateCoachInfo(void);

	void changeGoalColor_sl(int);

	void UpdateGameParameter_slot(void);

	void updateRefLog(void);

	void SetLogViewMode_slot(bool);


	


signals:
	void transmitCoach(void);
	void changeGoalColor (int);
	void UpdateGameParameter_signal(void);
	void SetLogViewMode_signal(bool);
};

#endif
