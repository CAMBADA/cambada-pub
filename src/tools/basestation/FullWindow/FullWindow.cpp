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

#include "FullWindow.h"


FWind::FWind(QMainWindow *parent)
{
	/* Criação da janela */
	setupUi(parent);

	/* Inicialização das variáveis locais */
	fullscreenflag = 0;
	fullwindow = parent;

	Robots_info = NULL;
	db_coach_info = NULL;

	UpdateTimer = new QTimer();

	/* Conecções */
	connect(FullScrBot, SIGNAL(clicked()), this, SLOT(WindowFullScreenMode()));
	connect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
	connect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameInfo()));


	UpdateTimer->start(100);
}
	

FWind::~FWind()
{
	//disconnect(FullScrBot, SIGNAL(clicked()), this, SLOT(WindowFullScreenMode()));
	//disconnect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
	//disconnect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	//disconnect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameInfo()));

	delete UpdateTimer;
}

void FWind::WindowFullScreenMode (void)
{
	if ( fullscreenflag == 0 )
	{
		fullscreenflag = 1;
		fullwindow->showFullScreen();
		FullScrBot->setText(QString("Leave Full Screen"));

	}
	else
	{
		fullscreenflag = 0;
		fullwindow->showMaximized();
		FullScrBot->setText(QString("Full Screen"));
	}

}

void FWind::get_info_pointer( DB_Robot_Info * rw)
{
	Robots_info = rw;
	FieldW->get_info_pointer(Robots_info);
}

void FWind::get_coach_pointer( DB_Coach_Info * ci)	
{
	db_coach_info=ci;
}

void FWind::UpdateGameInfo(void)
{

QTime GTime;
int min=0, sec=0;

	
	if(db_coach_info != NULL) 
	{
		if( (db_coach_info->GamePart == 1) || (db_coach_info->GamePart == 2))
		{
			min = (int)( (db_coach_info->GameTime.elapsed() / 1000) / 60 );

			sec = (int)( (db_coach_info->GameTime.elapsed() / 1000) % 60 );

			GTime.setHMS(0,min, sec);
			Game_time_clock->setText(GTime.toString("mm:ss"));


			/* Score */
			if (db_coach_info->Coach_Info.ourGoals > 9) Cambada_score->setNumDigits(2);
				else Cambada_score->setNumDigits(1);

			if (db_coach_info->Coach_Info.theirGoals > 9) MF_score->setNumDigits(2);
				else MF_score->setNumDigits(1);

			Cambada_score->display(db_coach_info->Coach_Info.ourGoals);
			MF_score->display(db_coach_info->Coach_Info.theirGoals);

		}

		else
		{
			GTime.setHMS(0,min, sec);
			Game_time_clock->setText(GTime.toString("mm:ss"));
		}
	}

}

