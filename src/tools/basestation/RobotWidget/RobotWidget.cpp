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

#include "RobotWidget.h"

RWidget::RWidget(QWidget *parent)
{
	setupUi(parent);

	/* Mudar a cor dos roles para */


	/* inicialização das variáveis */
	KO = new QString("KO");
	NA = new QString("NA");
	OK = new QString("OK");
	SB = new QString("SB");

	lock_flag = 0;


	//stat_White = 	QColor::fromRgb(255,127,50,255);
	stat_White = 	QColor::fromRgb(72,72,72,255);
	stat_Red = 	QColor::fromRgb(191, 63, 63, 255);
	stat_Green = 	QColor::fromRgb(100, 172, 100, 255);
	stat_blue = 	QColor::fromRgb(191, 63, 63, 255);//QColor::fromRgb(255,127,50,255);//QColor::fromRgb(128, 160, 191, 255);
/*	stat_Red = 	QColor::fromRgb(255, 153, 153, 255);
	stat_Green = 	QColor::fromRgb(208, 240, 192, 255);
	stat_blue = 	QColor::fromRgb(70, 130, 180, 255);
*/
	DB_Info = NULL;
	db_coach_info = NULL;
	my_number = 0;

	UpdateTimer = new QTimer();
	RobotDetailsDialog = new QDialog (parent);
	RDial = new RobotDialog(RobotDetailsDialog);

	RunBotFlip = 0;

	/* Inicializar a combobox dos roles */
	RobotRoleCombo->clear();

	QString str;
	QString rm = "Role";
	RobotRoleCombo->insertItem(0, "Auto");
	for (int i=1; i<num_roles;i++)
	{
		str=role_names[i];
		
		if(i!=0) str.remove(rm);
		RobotRoleCombo->insertItem(i, str);
	}


	/* conecções dos objectos */
	connect(runBot, SIGNAL(clicked()), this, SLOT(runBotPressed ()));
	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(updateInfo()));
	//connect(RobotdetailsBot, SIGNAL(clicked()), this, SLOT(detailsBotPressed()));
	connect(RobotRoleCombo, SIGNAL(activated(int )), this, SLOT(RoleChanged(int )));
	connect(robotTeamBox, SIGNAL(activated(int )), this, SLOT(TeamColorChanged(int )));
	connect(robotGoalBox, SIGNAL(activated(int )), this, SLOT(GoalColorChanged(int )));

	connect(RelocBot, SIGNAL(clicked()), this, SLOT(relocBotPressed()));

	connect(StartBot, SIGNAL(clicked()), this, SLOT(startBotPressed()));
	connect(StopBot, SIGNAL(clicked()), this, SLOT(stopBotPressed()));
	connect(AgentBot, SIGNAL(clicked()), this, SLOT(agentBotPressed()));
	connect(Monitor_on_Bot, SIGNAL(clicked()), this, SLOT(monitor_on_Pressed()));
	connect(Monitor_off_Bot, SIGNAL(clicked()), this, SLOT(monitor_off_Pressed()));


	connect(lockRobot_check, SIGNAL(stateChanged(int )), this, SLOT(lockRobot(int)));



	/* Inicializações */
	NA_flag=1;
	changeRobotStatus(NA);
	UpdateTimer->start(50);
	TeamColorChanged( 0 );
}

RWidget::~RWidget()
{
	disconnect(runBot, SIGNAL(clicked()), this, SLOT(runBotPressed ()));
	disconnect(UpdateTimer, SIGNAL(timeout()), this, SLOT(updateInfo()));
	disconnect(RobotRoleCombo, SIGNAL(activated(int )), this, SLOT(RoleChanged(int )));
	disconnect(robotTeamBox, SIGNAL(activated(int )), this, SLOT(TeamColorChanged(int )));
	disconnect(robotGoalBox, SIGNAL(activated(int )), this, SLOT(GoalColorChanged(int )));
	disconnect(RelocBot, SIGNAL(clicked()), this, SLOT(relocBotPressed()));
	disconnect(StartBot, SIGNAL(clicked()), this, SLOT(startBotPressed()));
	disconnect(StopBot, SIGNAL(clicked()), this, SLOT(stopBotPressed()));
	disconnect(AgentBot, SIGNAL(clicked()), this, SLOT(agentBotPressed()));
	disconnect(Monitor_on_Bot, SIGNAL(clicked()), this, SLOT(monitor_on_Pressed()));
	disconnect(Monitor_off_Bot, SIGNAL(clicked()), this, SLOT(monitor_off_Pressed()));
	disconnect(lockRobot_check, SIGNAL(stateChanged(int )), this, SLOT(lockRobot(int)));

	delete KO;
	delete NA;
	delete OK;
	delete SB;

	delete UpdateTimer;
	delete RobotDetailsDialog;
	delete RDial;
}

void RWidget::changeRobotStatus(QString *status)
{
	QColor color;
	QColor Black = QColor::fromRgb(0,0,0,255);
	QColor back_color = QColor::fromRgb(72,72,72,255);
	QPalette plt;
	

	if (*status == *SB) color = stat_White;
	else if (*status == *NA) color = stat_Red;
	else if (*status == *KO) color = stat_Red;
	else if (*status == *OK) color = stat_Green;
	else return;
	
	//if ( (*status == *SB) || (*status == *NA) || (*status == *KO) ) 
	//	{
			//runBot->setText(QString("Run"));
			//runBot->setChecked(0);
			
	//	}

	//if (*status == *OK)
	//{	
		//runBot->setText(QString("Stop"));
		//runBot->setChecked(1);
	//}

	robotStatusLabel->setText(*status);
	plt.setColor(QPalette::Background, color);
	plt.setColor(QPalette::Foreground, Black);
	robotStatusLabel->setPalette(plt); 

	//Change widget color
	changeBackGroundColor(status);

}

void RWidget::changeBackGroundColor(QString *status)
{

QColor BColor;
QColor FColor;
QColor Black = QColor::fromRgb(0,0,0,255);
QColor White = QColor::fromRgb(255,255,255,255);
QColor Tip_Color = QColor::fromRgb(72,72,72,255);
QPalette plt,plt2;


	if (*status == *NA)
	{
		NA_flag=1;
		BColor=stat_Red;
		FColor=stat_Red;
	}
	else
	{
		NA_flag=0;
		BColor=Tip_Color;
		FColor=White;
	}

	
plt.setColor(QPalette::Background, BColor);
plt.setColor(QPalette::Foreground, FColor);

Role_Label->setPalette(plt);
Role_info_lb->setPalette(plt);
Behaviour_Label->setPalette(plt);
Behaviour_info_lb->setPalette(plt);

Bat_Label->setPalette(plt);

Bat_1_info_lb->setPalette(plt);
Bat_2_info_lb->setPalette(plt);
Bat_3_info_lb->setPalette(plt);
Bat_4_info_lb->setPalette(plt);

}

void RWidget::runBotPressed (void)
{


	if(RunBotFlip == 0)
	{
		changeRobotRun ();
	}
	else
	{
		changeRobotStop ();
	}	

}

void RWidget::changeRobotRun (void)
{

	if (!lock_flag)
	{
		/* Altera a cor ao botão */
		QColor Green = QColor::fromRgb(100,172,100,255);
		QPalette plt;

		plt.setColor(QPalette::Button, Green);
		runBot->setPalette(plt);

		runBot->setText("Stop");

		/* Difunde a informação */
		db_coach_info->Coach_Info.playerInfo[my_number].running = true;
		RunBotFlip=1;
		runBot->setChecked(1);
		emit transmitCoach();	
	}
}

void RWidget::changeRobotStop (void)
{
	if (!lock_flag)
	{
		/* Altera a cor ao botão */
		QColor Red = QColor::fromRgb(191,63,63,255);
		QPalette plt;

		plt.setColor(QPalette::Button, Red);
		runBot->setPalette(plt);

		runBot->setText("Run");

		/* Difunde a informação */

		db_coach_info->Coach_Info.playerInfo[my_number].running = false;
		RunBotFlip=0;
		runBot->setChecked(0);
		emit transmitCoach();	
	}
}


void RWidget::get_info_pointer( DB_Robot_Info * rw)
{
	DB_Info = rw;
}

void RWidget::get_robot_number(int num)
{
	my_number = num;
	RDial->get_robot_number(my_number);
}

void RWidget::updateInfo(void)
{



	if(DB_Info != NULL)
	{
	 	switch (DB_Info->Robot_status[my_number])
	 	{
		case STATUS_NA : 
				changeRobotStatus(NA);
				break;
		case STATUS_OK : 
				changeRobotStatus(OK);
				break;

		case STATUS_SB : 
				changeRobotStatus(SB);
				break;

		case STATUS_KO : 
				changeRobotStatus(KO);
				break;
	 	}

		
		/* Role Info */
		if ( 	(DB_Info->Robot_info[my_number].role < num_roles) &&
			(DB_Info->Robot_info[my_number].role >=0)		)

			Role_info_lb->setText(role_names [DB_Info->Robot_info[my_number].role]);
		else
			Role_info_lb->setText("Auto");

		/* Behaviour info */
		if ( 	(DB_Info->Robot_info[my_number].behaviour < num_behaviours) &&
		(DB_Info->Robot_info[my_number].behaviour >=0)			)
		
		Behaviour_info_lb->setText(behaviour_names[DB_Info->Robot_info[my_number].behaviour]);
		else
		Behaviour_info_lb->setText("Unknown");


	/* Informação do estado das baterias */
	



	QString str;
		/* Carregar uma palete de cores para alterar os labels com as cores certas */
		QColor vermelho = Qt::red;
		QColor branco = QColor::fromRgb(255, 255, 255, 255);
		QColor azul = QColor::fromRgb(128,160,191,255);
		QColor color;
		QPalette plt;


if(NA_flag==0)
{	
	/* bateria 0 */
	if ((DB_Info->Robot_info[my_number].battery[0] > 0.5))// && (DB_Info->lifetime[my_number] > NOT_RUNNING_TIMEOUT))
		str = QString("%1").arg(DB_Info->Robot_info[my_number].battery[0], 3, 'f',1);

	else str = QString("%1").arg(0.000, 3, 'f',1);

	Bat_1_info_lb->setText(str);
	if (DB_Info->Robot_info[my_number].battery[0] < V_9_6_VOLTAGE_LIMIT)
		color=vermelho;
	else
		color=branco;

	plt.setColor(QPalette::Foreground, color);
	Bat_1_info_lb->setPalette(plt);

	/* bateria 1 */
	if ((DB_Info->Robot_info[my_number].battery[1] > 0.5))// && (DB_Info->lifetime[my_number] > NOT_RUNNING_TIMEOUT))
		str = QString("%1").arg(DB_Info->Robot_info[my_number].battery[1], 3, 'f',1);

	else str = QString("%1").arg(0.00, 3, 'f',1);

	Bat_2_info_lb->setText(str);

	if (DB_Info->Robot_info[my_number].battery[1] < V_12_0_VOLTAGE_LIMIT)
		color=vermelho;
	else
		color=branco;

	plt.setColor(QPalette::Foreground, color);
	Bat_2_info_lb->setPalette(plt);

	/* bateria 2*/
	if ((DB_Info->Robot_info[my_number].battery[2] > 0.5) )//&& (DB_Info->lifetime[my_number] > NOT_RUNNING_TIMEOUT))
		str = QString("%1").arg(DB_Info->Robot_info[my_number].battery[2], 3, 'f',1);

	else str = QString("%1").arg(0.00, 3, 'f',1);

	Bat_3_info_lb->setText(str);

	if (DB_Info->Robot_info[my_number].battery[2] < V_12_0_VOLTAGE_LIMIT)
		color=vermelho;
	else
		color=branco;

	plt.setColor(QPalette::Foreground, color);
	Bat_3_info_lb->setPalette(plt);



/* Suporte para a bateria do portátil -> Basta descomentar */
	/* bateria 3*/

	//str = QString("%1").arg(DB_Info->lpBat[my_number].status, 3, 'f',3);

	Bat_4_info_lb->setNum(DB_Info->lpBat[my_number].status);
	

	if (DB_Info->lpBat[my_number].charge == DISCHARGING)
		{
			color=branco;
			if (DB_Info->lpBat[my_number].status < 10) color = vermelho;
		}
	else
		color=azul;

	
	plt.setColor(QPalette::Foreground, color);
	Bat_4_info_lb->setPalette(plt);

}

	QString val;

	/* DB lifetime*/
	if (DB_Info->lifetime[my_number] > 0.5)
	{
		if (DB_Info->lifetime[my_number] < NOT_RUNNING_TIMEOUT) 
		{
			const int ltt = DB_Info->lifetime[my_number] / 100;
			//val.setNum(ltt);
			val = "";
			for( int i = 0 ; i <= ltt ; i++ )
				val+="|";
		}
		else
		{
			val.setNum(DB_Info->lifetime[my_number]/1000);
			val += " secs";
		}
	}
	else 
		val = "";

		Db_info_lb->setText(val);

//	if (DB_Info->lifetime[my_number] < NOT_RUNNING_TIMEOUT)
		color=branco;
//	else
//		color=vermelho;

	plt.setColor(QPalette::Foreground, color);
	Db_info_lb->setPalette(plt);


	}
	
	//QString val;
	//val.setNum(DB_Info->Robot_info[my_number].coordinationFlag);
	if (DB_Info->Robot_info[my_number].coordinationFlag[0] <= num_coordination_types && DB_Info->Robot_info[my_number].coordinationFlag[0] >= 0)
	{
		QString val = QString(coordination_names[DB_Info->Robot_info[my_number].coordinationFlag[0]]);
		CF_val->setText(val.trimmed());
	}

	

}

void RWidget::detailsBotPressed (void)
{

	if (DB_Info != NULL) RDial->updateInfo();

	RobotDetailsDialog->show();
}

void RWidget::ChangeTextSize(int size)
{
	QFont newFont = RobotRoleCombo->font();

	newFont.setPointSize(size);

	RobotRoleCombo->setFont(newFont);
	robotTeamBox->setFont(newFont);
	robotGoalBox->setFont(newFont);
	runBot->setFont(newFont);
//	RobotdetailsBot->setFont(newFont);
	RelocBot->setFont(newFont);
}

void RWidget::get_coach_pointer( DB_Coach_Info * ci)	
{
	db_coach_info=ci;
	RDial->get_coach_pointer(ci);
}

void RWidget::RoleChanged(int role_id)
{

	if (!lock_flag)
	{

		if (role_id < 1) db_coach_info->Coach_Info.playerInfo[my_number].roleAuto = true;

		else
		{
			db_coach_info->Coach_Info.playerInfo[my_number].roleAuto = false;
			db_coach_info->Coach_Info.playerInfo[my_number].role = (RoleID)role_id;
		}

	
		RobotRoleCombo->setCurrentIndex(role_id);
		emit transmitCoach();
	}
}

void RWidget::TeamColorChanged(int color_id)
{

	if (!lock_flag)
	{

		//QColor Mag = QColor::fromRgb(191,63,63,255);
		QColor Mag = QColor::fromRgb(222,111,161,255);//Qt::magenta;//QColor::fromRgb(186,158,183,255);
		QColor Cy = QColor::fromRgb(128,160,191,255);
		QPalette plt;

		if (color_id == 0)
		{
			if (db_coach_info != NULL) db_coach_info->Coach_Info.playerInfo[my_number].teamColor = Magenta;
			plt.setColor(QPalette::Button, Mag);
		}

		else 
		{
			if (db_coach_info != NULL) db_coach_info->Coach_Info.playerInfo[my_number].teamColor = Cyan;
			plt.setColor(QPalette::Button, Cy);
		}

		robotTeamBox->setCurrentIndex(color_id);
		robotTeamBox->setPalette(plt);

		emit transmitCoach();	
	}
}

void RWidget::GoalColorChanged(int color_id)
{

	if (!lock_flag)
	{
		QColor Yell = QColor::fromRgb(255,191,105,255);
		QColor Bl = QColor::fromRgb(0,180,247,255);
		QPalette plt;


		if (color_id == 0)
		{
			db_coach_info->Coach_Info.playerInfo[my_number].goalColor = Yellow;
			plt.setColor(QPalette::Button, Yell);
			robotGoalBox->setCurrentIndex(0);
		}
		else
		{
			db_coach_info->Coach_Info.playerInfo[my_number].goalColor = Blue;
			plt.setColor(QPalette::Button, Bl);
			robotGoalBox->setCurrentIndex(1);
		}


		robotGoalBox->setPalette(plt);


		emit transmitCoach();	
	}
}


void RWidget::startBotPressed(void)
{

	printf("Start Robot %d\n", my_number+1);

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);


//	if(Started_flag == 0)
//		start.setMyThread(cmd + " \"(cd /home/cambada/cambada_bins/bin; sudo -E ./cambada_monitor >& /dev/null  & )\" ",my_number+1);
		start.setMyThread(cmd + " \"(cd /home/cambada/bin; ./start >& /dev/null  & )\" ",my_number+1);

/*	else
		start.setMyThread(cmd + " ~/cambada_bins/bin/cambada_agent",my_number+1);
*/


	start.start();
//	Started_flag = 1;

/*
	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);
	system( (cmd + " ./START").toAscii() );
*/
}

void RWidget::stopBotPressed(void)
{

	printf("Stop Robot %d\n", my_number+1);

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);

	stop.setMyThread(cmd + " \"(cd /home/cambada/bin; ./stop )\" ",my_number+1);
	stop.start();

}

void RWidget::agentBotPressed(void)
{
	printf("Agent Kill and Start %d\n", my_number+1);

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);

	agent.setMyThread(cmd +  " \"sudo -E killall -INT agent; (cd /home/cambada/bin; sudo -E ./agent >& /dev/null & )\" ",my_number+1);
	agent.start();
}

void RWidget::monitor_on_Pressed(void)
{
	printf("Monitor On on Robot %d\n", my_number+1);

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);

	Monitor_on.setMyThread(cmd + " \"(cd /home/cambada/bin; ./start monitor )\" ",my_number+1);
	Monitor_on.start();
}

void RWidget::monitor_off_Pressed(void)
{
	printf("Monitor Off on Robot %d\n", my_number+1);

	if( my_number == -1) return;

	QString cmd = "ssh -o ConnectTimeout=3 cambada@172.16.39.";
	cmd += QString::number(my_number+1);

	Monitor_off.setMyThread(cmd +  " \"(cd /home/cambada/bin; ./stop monitor )\" ",my_number+1);
	Monitor_off.start();
}
void RWidget::relocBotPressed (void)
{
	if (db_coach_info != NULL) db_coach_info->Coach_Info.changePositionSN[my_number]++;

	emit transmitCoach();	

}


void RWidget::lockRobot(int lock)
{
	lock_flag = lock;

	runBot->setDisabled ( lock );
	RobotRoleCombo->setDisabled ( lock );
	robotTeamBox->setDisabled ( lock );
	robotGoalBox->setDisabled ( lock );

}
