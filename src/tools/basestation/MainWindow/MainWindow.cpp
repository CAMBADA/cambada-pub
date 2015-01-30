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

#include "MainWindow.h"

MWind *wind;

MWind::MWind(QMainWindow *parent)
{
	/* Criação da janela */
	setupUi( parent );
	
	//FieldW->updateField();

	/* Pintar as letras dos group em branco */
	QColor color = QColor::fromRgb(255,255,255,255);
	QPalette plt;

	plt.setColor(QPalette::Foreground, color);
	Group_R1->setPalette(plt); 
	Group_R2->setPalette(plt); 
	Group_R3->setPalette(plt); 
	Group_R4->setPalette(plt); 
	Group_R5->setPalette(plt); 
    Group_R6->setPalette(plt);
	Group_T->setPalette(plt); 
	Group_RB->setPalette(plt); 

	fullinfowindow = new QMainWindow;
	FIW = new FInfoWind(fullinfowindow);

	/* inicialização das variáveis */
	mwind = parent;
	fullscreenflag = 0;
	robots[0] = RobotW1;
	robots[1] = RobotW2;
	robots[2] = RobotW3;
	robots[3] = RobotW4;
	robots[4] = RobotW5;
    robots[5] = RobotW6;

	GoalColorCombo->setCurrentIndex(1);


	/* Inicialização do update widget */
	UpdateWG =new UpdateWidget();
	Robots_info = UpdateWG->get_info_pointer();
    FIW->get_info_pointer( Robots_info );
	FieldW->get_info_pointer( Robots_info );
	RefBoxWG->get_info_pointer( Robots_info );
	

	for(unsigned i=0;i<NROBOTS;i++)
		robots[i]->get_info_pointer( Robots_info );

	db_coach_info = UpdateWG->get_coach_pointer();
	for(unsigned i=0;i<NROBOTS;i++)
		robots[i]->get_coach_pointer( db_coach_info );

	RefBoxWG->get_coach_pointer( db_coach_info );
	FieldW->get_coach_pointer( db_coach_info );


	/* Inicializar a combobox dos roles */
	AllRoleCombo->clear();

	QString str;
	QString rm = "Role";
	AllRoleCombo->insertItem(0, "Auto");
	for (int i=1; i<num_roles;i++)
	{
		str=role_names[i];
		
		if(i!=0) str.remove(rm);
			AllRoleCombo->insertItem(i, str);
	}


	for (unsigned i=0; i<NROBOTS; i++)
		robots[i]->get_robot_number(i);
/*
	Robot_info_W1->get_robot_number( 0 );
	Robot_info_W2->get_robot_number( 1 );
	Robot_info_W3->get_robot_number( 2 );
	Robot_info_W4->get_robot_number( 3 );
	Robot_info_W5->get_robot_number( 4 );
	Robot_info_W6->get_robot_number( 5 );
*/


	/* Inicializar o Timer de update do game time*/
	UpdateTimer = new QTimer();

	/* Conectar as funções da barra de menus */
    connect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
	connect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	connect(actionConnect, SIGNAL(triggered()), RefBoxWG, SLOT(detailsBotPressed()));
	connect(actionFull_Screen, SIGNAL(triggered()), this, SLOT(changeWindowFullScreenMode()));
    connect(actionView_Full_Screen_info, SIGNAL(triggered()), this, SLOT(showFullScreenInfoWindow()));
	connect(AllRunBot, SIGNAL(clicked()), this, SLOT(AllRunBotPressed()));
	connect(AllStopBot, SIGNAL(clicked()), this, SLOT(AllStopBotPressed()));
	connect(TeamColorCombo, SIGNAL(activated ( int)), this, SLOT(TeamColorChanged(int)));
	connect(GoalColorCombo, SIGNAL(activated ( int)), this, SLOT(GoalColorChanged(int)));
	connect(AllRoleCombo, SIGNAL(activated(int )), this, SLOT(allRoleChanged(int )));

	//Obstacles
	connect(actionAll_On_Off, SIGNAL(toggled ( bool)), this, SLOT(allObstaclePoints(bool)));
    connect(actionAgent_1, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r0 (bool)));
	connect(actionAgent_2, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r1 (bool)));
	connect(actionAgent_3, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r2 (bool)));
	connect(actionAgent_4, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r3 (bool)));
	connect(actionAgent_5, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r4 (bool)));
    connect(actionAgent_6, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r5 (bool)));
	
	//Debug Points
    connect(actionDebug_All_On_Off, SIGNAL(toggled ( bool)), this, SLOT(allDebugPoints(bool)));
    connect(actionDebugAgent_1, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r0 (bool)));
	connect(actionDebugAgent_2, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r1 (bool)));
    connect(actionDebugAgent_3, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r2 (bool)));
	connect(actionDebugAgent_4, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r3 (bool)));
	connect(actionDebugAgent_5, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r4 (bool)));
    connect(actionDebugAgent_6, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r5 (bool)));
	
    connect(actionTop_View, SIGNAL(toggled(bool)), this, SLOT(on_actionTop_View_toggled(bool)));
    connect(actionLock, SIGNAL(toggled(bool)), FieldW, SLOT(lock(bool)));


	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(UpdateGameTime()));
	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(transmitCoach()));


	for(unsigned i=0; i<NROBOTS;i++)
		connect(robots[i], SIGNAL(transmitCoach()), this, SLOT(transmitCoach()));
	connect(RefBoxWG, SIGNAL(transmitCoach()), this, SLOT(transmitCoach()));

	connect(RefBoxWG, SIGNAL(changeGoalColor (int)), this, SLOT(GoalColorChanged(int)));

	connect(RefBoxWG, SIGNAL(UpdateGameParameter_signal()), this, SLOT(UpdateGameParameters()));

	connect(RefBoxWG, SIGNAL(SetLogViewMode_signal(bool)), UpdateWG, SLOT(SetLogViewMode(bool)));
    // Arranque em auto-formation
    FormationCombo->setEnabled(false);

	/* instalar o filtro de eventos */
	parent->installEventFilter(this);

	TeamColorChanged( 0 );
	GoalColorChanged( 1 );
	
    UpdateWG->setFormationCombo(FormationCombo);
    UpdateTimer->start(10);


	/* Descomentar estas linhas para testes no campo */
	commWatcher = new MyThreadWatcher( labelComm, "comm" );
	commWatcher->start();

	coachWatcher = new MyThreadWatcher( labelCoach, "coach" );
	coachWatcher->start();

	loggerWatcher = new MyThreadWatcher( labelLogger, "logger" );
	loggerWatcher->start();

	/* Inicializar o GameClock */
	db_coach_info->gTimeSecOffset=0;
	db_coach_info->GameTime.start();
	db_coach_info->Coach_Info.time=0;
	db_coach_info->Coach_Info_in.time=0;
	db_coach_info->logTimeOffset=0;
	db_coach_info->addLogTimeOffset=false;
	QTime GTime;
	GTime.setHMS(0,0,0);
	Game_time_clock->setText(GTime.toString("mm:ss"));

	/* Inicializar o Logo */
	cambada_logo_pixmap = new QPixmap(*Cambada_logo->pixmap());
	Cambada_logo->clear();

	/* Formation */
	QFile file("../config/formation.conf");
	FormationCombo->clear();
	if( file.open(QIODevice::ReadOnly | QIODevice::Text) )
	{
		connect(FormationCombo, SIGNAL(activated(int )), this, SLOT(FormationChanged(int )));
		int nFormations = 0;
		QTextStream in(&file);
		
		while( !in.atEnd() ) 
		{
			QString line = in.readLine();
			if( line.contains("FORMATIONDT") )
			{
				line.remove("FORMATIONDT");
                                line = line.trimmed();
				FormationCombo->insertItem(nFormations,line.left(line.indexOf(' ')) );
                nFormations++;
			}
			else if( line.contains("FORMATION") )
			{
				line.remove("FORMATION");
				FormationCombo->insertItem(nFormations,line.trimmed());
				nFormations++;
			}
		}

        connect(checkBoxManualFormation, SIGNAL(stateChanged(int)),this,SLOT(ManualFormationChanged(int)));
        FormationCombo->setEnabled(false);
    }
	else
	{
		printf("Error Opening the Formation File\n");
		FormationCombo->insertItem(0,"Unvailable");
        FormationCombo->setEnabled(false);
        checkBoxManualFormation->setEnabled(false);
	}

    connect(actionVisible, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMapVisible(bool)));
    connect(action3D, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMap3D(bool)));
    connect(actionColor, SIGNAL(toggled(bool)), FieldW, SLOT(setHeightMapColor(bool)));

}



MWind::~MWind()
{
	// Disconnect
    disconnect(actionFlip, SIGNAL(triggered()), FieldW, SLOT(flip()));
    disconnect(actionDebug_Points, SIGNAL(triggered()), FieldW, SLOT(debug_point_flip()));
	disconnect(actionConnect, SIGNAL(triggered()), RefBoxWG, SLOT(detailsBotPressed()));
	disconnect(actionFull_Screen,  SIGNAL(triggered()), this, SLOT(changeWindowFullScreenMode()));
    disconnect(actionView_Full_Screen_info, SIGNAL(triggered()), this, SLOT(showFullScreenInfoWindow()));
	disconnect(AllRunBot, SIGNAL(clicked()), this, SLOT(AllRunBotPressed()));
	disconnect(AllStopBot, SIGNAL(clicked()), this, SLOT(AllStopBotPressed()));
	disconnect(TeamColorCombo, SIGNAL(activated ( int)), this, SLOT(TeamColorChanged(int)));
	disconnect(GoalColorCombo, SIGNAL(activated ( int)), this, SLOT(GoalColorChanged(int)));
	disconnect(AllRoleCombo, SIGNAL(activated(int )), this, SLOT(allRoleChanged(int )));

	disconnect(actionAll_On_Off, SIGNAL(toggled ( bool)), this, SLOT(allObstaclePoints(bool)));
    disconnect(actionAgent_1, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r0 (bool)));
	disconnect(actionAgent_2, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r1 (bool)));
	disconnect(actionAgent_3, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r2 (bool)));
	disconnect(actionAgent_4, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r3 (bool)));
	disconnect(actionAgent_5, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r4 (bool)));
    disconnect(actionAgent_6, SIGNAL(toggled ( bool)), FieldW, SLOT(obstacles_point_flip_r5 (bool)));

	//Debug Points
    //disconnect(actionDebug_All_On_Off, SIGNAL(toggled ( bool)), this, SLOT(allDebugPoints(bool)));
    disconnect(actionDebugAgent_1, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r0(bool)));
    disconnect(actionDebugAgent_2, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r1(bool)));
    disconnect(actionDebugAgent_3, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r2(bool)));
    disconnect(actionDebugAgent_4, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r3(bool)));
    disconnect(actionDebugAgent_5, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r4(bool)));
    disconnect(actionDebugAgent_6, SIGNAL(toggled ( bool)), FieldW, SLOT(debug_point_flip_r5(bool)));
	
	// Destroy "Gustavo" Threads
	commWatcher->terminate();
	while (commWatcher->wait()!=1){};
	coachWatcher->terminate();
	while (coachWatcher->wait()!=1){};
	loggerWatcher->terminate();
	while (coachWatcher->wait()!=1){};
	
	

	//Delete
	if(UpdateWG!=NULL)		delete UpdateWG; UpdateWG=NULL;
	if(UpdateTimer!=NULL)	delete UpdateTimer; UpdateTimer=NULL;
	if(commWatcher!=NULL)	delete commWatcher; commWatcher=NULL;
	if(coachWatcher!=NULL)	delete coachWatcher; coachWatcher=NULL;
	if(loggerWatcher!=NULL)	delete loggerWatcher; loggerWatcher=NULL;
	if(cambada_logo_pixmap!=NULL)	delete cambada_logo_pixmap;	 cambada_logo_pixmap=NULL;
    if(fullinfowindow!=NULL) delete fullinfowindow; fullinfowindow=NULL;
    if(FIW!=NULL)			delete FIW; FIW=NULL;

	if( this->RefBoxWG != NULL ) delete this->RefBoxWG; this->RefBoxWG = NULL;
}

void MWind::changeWindowFullScreenMode (void)
{
	if ( fullscreenflag == 0 )
	{
		fullscreenflag = 1;
		mwind->showFullScreen();
		actionFull_Screen->setText(QString("Leave Full Screen"));

	}
	else
	{
		fullscreenflag = 0;
		mwind->showMaximized();
		actionFull_Screen->setText(QString("Full Screen"));
	}

}

void MWind::showFullScreenInfoWindow (void)
{
	fullinfowindow->showMaximized();
}

void MWind::AllRunBotPressed(void)
{
		for (unsigned i=0; i<NROBOTS; i++)
			robots[i]->changeRobotRun();

}
void MWind::AllStopBotPressed(void)
{
	for (unsigned i=0; i<NROBOTS; i++)
			robots[i]->changeRobotStop();
}

void MWind::TeamColorChanged(int team)
{
//QColor Mag = QColor::fromRgb(191,63,63,255);
QColor Mag = QColor::fromRgb(222,111,161,255);//Qt::magenta;//
QColor Cy = QColor::fromRgb(128,160,191,255);
QPalette plt;


for (unsigned i=0; i<NROBOTS; i++)
	robots[i]->TeamColorChanged(team);

	if (team == 0) 
	{
		plt.setColor(QPalette::Button, Mag);
		if (db_coach_info != NULL) db_coach_info->TeamColor=Magenta;
	}
	else 
	{
		plt.setColor(QPalette::Button, Cy);
		if (db_coach_info != NULL) db_coach_info->TeamColor=Cyan;
	}
	
	TeamColorCombo->setPalette(plt);

}

void MWind::GoalColorChanged(int goal)
{

QColor Yell = QColor::fromRgb(255,191,105,255);
QColor Bl =  QColor::fromRgb(0,180,247,255);//Qt::blue;//
QPalette plt;

for (unsigned i=0; i<NROBOTS; i++)
	robots[i]->GoalColorChanged(goal);

	if (goal == 0) 
	{
		if (db_coach_info != NULL) db_coach_info->GoalColor=Yellow;
		plt.setColor(QPalette::Button, Yell);
	}
	else 
	{
		if (db_coach_info != NULL) db_coach_info->GoalColor=Blue;
		plt.setColor(QPalette::Button, Bl);
	}

	GoalColorCombo->setPalette(plt);
	GoalColorCombo->setCurrentIndex(goal);
}



bool MWind::eventFilter(QObject *obj, QEvent *event)
{
	if (event->type() == QEvent::KeyPress)
	{

		/* Shortcuts da basestation */
		QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

		if (keyEvent->key() == Qt::Key_F1)
			RobotW1->runBotPressed();

		if (keyEvent->key() == Qt::Key_F2)
			RobotW2->runBotPressed();

		if (keyEvent->key() == Qt::Key_F3)
			RobotW3->runBotPressed();

		if (keyEvent->key() == Qt::Key_F4)
			RobotW4->runBotPressed();

		if (keyEvent->key() == Qt::Key_F5)
			RobotW5->runBotPressed();

        if (keyEvent->key() == Qt::Key_F6)
            RobotW6->runBotPressed();

		if (keyEvent->key() == Qt::Key_Escape)
		{
			for (unsigned i=0; i<NROBOTS; i++)
				robots[i]->changeRobotStop();
		}

		if (keyEvent->key() == Qt::Key_F12)
		{
			for (unsigned i=0; i<NROBOTS; i++)
				robots[i]->changeRobotRun();
		}

		if (keyEvent->key() == Qt::Key_U)
		{
            //FIELD3D: FieldW->IsAnySelected=0;
		}
		
		if (keyEvent->key() == Qt::Key_C)
		{
			RefBoxWG->detailsBotPressed();
		}

		if (keyEvent->key() == Qt::Key_F)
		{
			changeWindowFullScreenMode();
		}

		if (keyEvent->key() == Qt::Key_O)
		{
			actionAll_On_Off->toggle();
		}

			
	}
	
	if ( (obj == mwind) && (event->type() == QEvent::Resize) )
	{
		QSize s = mwind->size();
//		printf("Size %d\n", s.rheight());

		/* Janela Info */
		char size_inc = 0;
		if 	(s.rwidth() > 1263) size_inc=5;
		else if	(s.rwidth() > 1248) size_inc=4;
		else if	(s.rwidth() > 1194) size_inc=3;
		else if	(s.rwidth() > 1059) size_inc=2;
		else if	(s.rwidth() > 993 ) size_inc=1;

		size_inc=3;
/*		Robot_info_W1->ChangeTextSize(5+size_inc);
		Robot_info_W2->ChangeTextSize(5+size_inc);
		Robot_info_W3->ChangeTextSize(5+size_inc);
		Robot_info_W4->ChangeTextSize(5+size_inc);
		Robot_info_W5->ChangeTextSize(5+size_inc);
		Robot_info_W6->ChangeTextSize(5+size_inc);
*/

		/* Janela principal */
		size_inc = 0;
		if 	(s.rwidth() > 1278) size_inc=3;
		else if	(s.rwidth() > 1158) size_inc=2;
		else if	(s.rwidth() > 1086) size_inc=1;

		size_inc=3;
		RobotW1->ChangeTextSize(5+size_inc);
		RobotW2->ChangeTextSize(5+size_inc);
		RobotW3->ChangeTextSize(5+size_inc);
		RobotW4->ChangeTextSize(5+size_inc);
		RobotW5->ChangeTextSize(5+size_inc);
        RobotW6->ChangeTextSize(5+size_inc);
		


		// Deal with Logo
		//printf("cenas %d\n",s.rheight());
		if (s.rheight() > 787) 
		{
			Cambada_logo->setPixmap ( *cambada_logo_pixmap );
		}
		else Cambada_logo->clear();
		return true;
	}


	return false;
}

void MWind::transmitCoach(void)
{
	UpdateWG->transmitCoach();
}

void MWind::UpdateGameTime(void)
{

    //FieldW->update();

QTime GTime;
int min=0, sec=0;


	if(db_coach_info != NULL) 
	{	
	//printf("UpdateGoals %d\n",db_coach_info->Coach_Info_in.ourGoals);
		/* Score */
        //if (db_coach_info->Coach_Info_in.ourGoals > 9) Cambada_score->setNumDigits(2);
        //else Cambada_score->setNumDigits(1);

        //if (db_coach_info->Coach_Info_in.theirGoals > 9) MF_score->setNumDigits(2);
        //else MF_score->setNumDigits(1);

        //Cambada_score->display(db_coach_info->Coach_Info_in.ourGoals);
        //MF_score->display(db_coach_info->Coach_Info_in.theirGoals);

		/*Game Time*/
/*		if(db_coach_info->gameTimeFlag==true) //computador produtor
		{
//			time_t currentTime = time(NULL);
//			double gameTime = difftime(currentTime,db_coach_info->gameStartTime);

			db_coach_info->Coach_Info.time = db_coach_info->GameTime.elapsed()/1000;
//			db_coach_info->Coach_Info.time= (int) (gameTime+0.5);
			transmitCoach();
			//printf()
		}*/
		
		min = (int)( (db_coach_info->Coach_Info_in.time) / 60 );
		sec = (int)( (db_coach_info->Coach_Info_in.time) % 60 );
//printf("gameTime %d %d \n",min,sec);
		QString minstr;
		minstr.setNum(min);
		if (minstr.length()==1) minstr = "0"+minstr;
		QString secstr;
		secstr.setNum(sec);
		if (secstr.length()==1) secstr = "0"+secstr;
		Game_time_clock->setText(minstr+":"+secstr);


#if 0
		if( (db_coach_info->GamePart == 1) || (db_coach_info->GamePart == 2))
		{
			min = (int)( (db_coach_info->GameTime.elapsed() / 1000) / 60 );

			sec = (int)( (db_coach_info->GameTime.elapsed() / 1000) % 60 );

			GTime.setHMS(0,min, sec);
			Game_time_clock->setText(GTime.toString("mm:ss"));


		}

		else /* Reset dos contadores */
		{
			//GTime.setHMS(0,min, sec);
			//Game_time_clock->setText(GTime.toString("mm:ss"));

			//Cambada_score->setNumDigits(1);
			//MF_score->setNumDigits(1);

			//Cambada_score->display(0);
			//MF_score->display(0);
		}

#endif
	}

}
void MWind::UpdateGameParameters(void)
{
	if(db_coach_info != NULL) 
		{
			//Game_time_clock->setText(db_coach_info->GameTime.toString("mm:ss"));
			UpdateGameTime();

		}

}

void MWind::allRoleChanged(int role_id)
{
	for (unsigned i=0; i<NROBOTS; i++)
			robots[i]->RoleChanged(role_id);

}

void MWind::FormationChanged(int form_id)
{
    if(!this->checkBoxManualFormation->isChecked())
        return;
    else
       UpdateWG->transmitFormation(this->FormationCombo->currentIndex());
}

void MWind::ManualFormationChanged(int state)
{
    if(state == Qt::Checked ){
        FormationCombo->setEnabled(true);

        if(db_coach_info != NULL){
            db_coach_info->Coach_Info.manualFormation = true;
            db_coach_info->Coach_Info.half=FormationCombo->currentIndex();
            UpdateWG->transmitCoach();
        }

    }else{
        FormationCombo->setEnabled(false);

        if(db_coach_info != NULL){
            db_coach_info->Coach_Info.manualFormation = false;
            FormationCombo->setCurrentIndex(db_coach_info->Coach_Info.half);
            UpdateWG->transmitCoach();
        }
    }

}

void MWind::allObstaclePoints(bool on_off)
{
	actionAgent_1->setChecked ( on_off );
	actionAgent_2->setChecked ( on_off );
	actionAgent_3->setChecked ( on_off );
	actionAgent_4->setChecked ( on_off );
	actionAgent_5->setChecked ( on_off );
	actionAgent_6->setChecked ( on_off );

    //FIELD3D: FieldW->obstacles_point_flip_all (on_off);
}

void MWind::allDebugPoints(bool on_off)
{
	actionDebugAgent_1->setChecked ( on_off );
	actionDebugAgent_2->setChecked ( on_off );
	actionDebugAgent_3->setChecked ( on_off );
	actionDebugAgent_4->setChecked ( on_off );
	actionDebugAgent_5->setChecked ( on_off );
	actionDebugAgent_6->setChecked ( on_off );

    //FIELD3D: FieldW->debug_point_flip_all (on_off);
}

void MWind::on_checkBoxManualFormation_stateChanged(int arg1)
{


    if(db_coach_info != NULL) {
        db_coach_info->Coach_Info.manualFormation = this->checkBoxManualFormation->isChecked();
        UpdateWG->transmitCoach();
        cout << "hellot\n";
        FormationCombo->setEnabled(this->checkBoxManualFormation->isChecked());
    }
}

void MWind::on_actionTaxi_Follow_Mode_toggled(bool arg1)
{
    FieldW->taxiFollow = arg1;
}

void MWind::on_actionTop_View_toggled(bool arg1)
{
    FieldW->setTop(arg1);
    //this->actionLock->setEnabled(!arg1);
}

void MWind::on_actionLock_toggled(bool arg1)
{

}
