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

#include "RobotDialog.h"


RobotDialog::RobotDialog(QDialog *parent)
{
	setupUi(parent);
	RobotNumberVal->setValue(-1);

	DB_Info = NULL;
	my_number = -1;

	connect(RobotNumberVal, SIGNAL(valueChanged(int)), this, SLOT(robotNumberChanged(int)));
	connect(TeamColorCombo, SIGNAL(activated(int )), this, SLOT(teamColorChanged(int)));
	connect(GoalColorCombo, SIGNAL(activated(int )), this, SLOT(goalColorChanged(int)));
	connect(StartBot, SIGNAL(clicked()), this, SLOT(startBotPressed()));

}


RobotDialog::~RobotDialog()
{

}

void RobotDialog::get_info_pointer( DB_Robot_Info * rw)
{
	DB_Info = rw;
	updateInfo();
}

void RobotDialog::get_coach_pointer( DB_Coach_Info * ci)	
{
	db_coach_info=ci;
}

void RobotDialog::get_robot_number(int num)
{
	my_number = num;

	if(DB_Info != NULL ) updateInfo();
	
}

void RobotDialog::updateInfo (void)
{
	if ( (DB_Info == NULL) || (my_number == -1) ) return;

	RobotNumberVal->setValue(DB_Info->Robot_info[my_number].number);


	if (DB_Info->Robot_info[my_number].teamColor == Cyan) TeamColorCombo->setCurrentIndex(1);
	else TeamColorCombo->setCurrentIndex(0);

	if (DB_Info->Robot_info[my_number].goalColor == Yellow) TeamColorCombo->setCurrentIndex(1);
	else GoalColorCombo->setCurrentIndex(0);

}

void RobotDialog::robotNumberChanged(int num)
{

if ( (db_coach_info == NULL) || (my_number == -1) ) return;

db_coach_info->Coach_Info.playerInfo[my_number].number=num;

}

void RobotDialog::teamColorChanged(int color_id)
{
if ( (db_coach_info == NULL) || (my_number == -1) ) return;

if (color_id == 0) db_coach_info->Coach_Info.playerInfo[my_number].teamColor= Magenta;
if (color_id == 1) db_coach_info->Coach_Info.playerInfo[my_number].teamColor= Cyan;
}

void RobotDialog::goalColorChanged(int color_id)
{
if ( (db_coach_info == NULL) || (my_number == -1) ) return;

if (color_id == 0) db_coach_info->Coach_Info.playerInfo[my_number].goalColor= Blue;
if (color_id == 1) db_coach_info->Coach_Info.playerInfo[my_number].goalColor= Yellow;

}

void RobotDialog::startBotPressed(void)
{

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);
	system( (cmd + " cd ~/bin;./STOP").toAscii() );
	system( (cmd + "  cd ~/bin;./START").toAscii() );
}

