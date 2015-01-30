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

#include "RobotInfoWidget.h"

RobotInfoWidget::RobotInfoWidget(QWidget *parent)
{

	/* Criação do widget */
	setupUi(parent);

	/* Inicialização das variáveis */
	Pi = 3.14159265358979323846;
	my_number = 0;
	DB_Info = NULL;
	UpdateTimer = new QTimer();

	/* conecções dos objectos */
	connect(UpdateTimer, SIGNAL(timeout()), this, SLOT(updateInfo()));

	/* Inicializações */
	UpdateTimer->start(50);
}

RobotInfoWidget::~RobotInfoWidget()
{
	disconnect(UpdateTimer, SIGNAL(timeout()), this, SLOT(updateInfo()));
	delete UpdateTimer;
}

void RobotInfoWidget::get_info_pointer( DB_Robot_Info * rw)
{
	DB_Info = rw;
}

void RobotInfoWidget::get_robot_number(int num)
{
	my_number = num;
}

void RobotInfoWidget::updateInfo(void)
{

 if (DB_Info != NULL)
 {

	QString str;


	/* Robot Number & Running status */
	Robot_number_lb->setNum(DB_Info->Robot_info[my_number].number);
	Robot_Running_lb->setNum(DB_Info->Robot_info[my_number].running);

	/* posição , velocidade e oreientação do robot */
	str = QString("%1").arg(DB_Info->Robot_info[my_number].pos.x, 3, 'f',2);
	Robot_position_lb_x->setText(str);

	str = QString("%1").arg(DB_Info->Robot_info[my_number].pos.y, 3, 'f',2);
	Robot_position_lb_y->setText(str);



	str = QString("%1").arg(DB_Info->Robot_info[my_number].vel.x, 3, 'f',2);
	Robot_velocity_lb_x->setText(str);

	str = QString("%1").arg(DB_Info->Robot_info[my_number].vel.y, 3, 'f',2);
	Robot_velocity_lb_y->setText(str);



	robot_orientation_lb_deg->setNum((int)(DB_Info->Robot_info[my_number].orientation*180/Pi));

	str = QString("%1").arg(DB_Info->Robot_info[my_number].orientation, 3, 'f',2);
	robot_orientation_lb_rad->setText(str);


	/* posição e velocidade da bola */

		/* Passar as coordenadas relativas da bola para coordenadas absolutas */
		double abs_x=DB_Info->Robot_info[my_number].ball.pos.x,
			abs_y=DB_Info->Robot_info[my_number].ball.pos.y;
		double 	rel_x=DB_Info->Robot_info[my_number].ball.posRel.x,
			rel_y=DB_Info->Robot_info[my_number].ball.posRel.y;

/*		abs_x = DB_Info->Robot_info[my_number].pos.x +
			cos(DB_Info->Robot_info[my_number].orientation)*
			rel_x-sin(DB_Info->Robot_info[my_number].orientation)*rel_y;
			
		abs_y = DB_Info->Robot_info[my_number].pos.y +
			sin(DB_Info->Robot_info[my_number].orientation)*
			rel_x+cos(DB_Info->Robot_info[my_number].orientation)*rel_y;*/



	str = QString("%1").arg(abs_x, 3, 'f',2);
	ball_position_lb_x->setText(str);

	str = QString("%1").arg(abs_y, 3, 'f',2);
	ball_position_lb_y->setText(str);



	str = QString("%1").arg(DB_Info->Robot_info[my_number].ball.vel.x, 3, 'f',2);
	ball_velocity_lb_x->setText(str);

	str = QString("%1").arg(DB_Info->Robot_info[my_number].ball.vel.y, 3, 'f',2);
	ball_velocity_lb_y->setText(str);


	ball_engaged_lb->setNum(DB_Info->Robot_info[my_number].ball.engaged);
	ball_visible_lb->setNum(DB_Info->Robot_info[my_number].ball.visible);
	ball_own_lb->setNum(DB_Info->Robot_info[my_number].ball.own);



	/* Role Info */
	if ( 	(DB_Info->Robot_info[my_number].role < num_roles) &&
		(DB_Info->Robot_info[my_number].role >=0)		)
		robot_role_lb->setText(role_names[DB_Info->Robot_info[my_number].role]);
	else
		robot_role_lb->setText("Unknown");


	/* Behaviour info */
	if ( 	(DB_Info->Robot_info[my_number].behaviour < num_behaviours) &&
		(DB_Info->Robot_info[my_number].behaviour >=0)			)
		
		robot_behavior_lb->setText(behaviour_names[DB_Info->Robot_info[my_number].behaviour]);
	else
		robot_behavior_lb->setText("Unknown");


	//robot_visible_lb->setNum(DB_Info->Robot_info[my_number].visible);
	robot_visible_lb->setNum(1);
	robot_coaching_lb->setNum(DB_Info->Robot_info[my_number].coaching);


	/* Team & Goal color */
	if (DB_Info->Robot_info[my_number].teamColor == Magenta)
		team_color_lb->setText("Magenta");

	else if (DB_Info->Robot_info[my_number].teamColor == Cyan)
		team_color_lb->setText("Cyan");

	else
		team_color_lb->setText("Unknown");


	if (DB_Info->Robot_info[my_number].goalColor == Blue)
		goal_color_lb->setText("Blue");

	else if (DB_Info->Robot_info[my_number].goalColor == Yellow)
		goal_color_lb->setText("Yellow");

	else
		goal_color_lb->setText("Unknown");



	/* Informação do estado das baterias */
	
		/* Carregar uma palete de cores para alterar os labels com as cores certas */
		QColor vermelho = Qt::red;
		QColor branco = QColor::fromRgb(255, 255, 255, 255);
		QColor azul = QColor::fromRgb(128,160,191,255);
		QColor color;
		QPalette plt;



	/* bateria 0 */
	if (DB_Info->Robot_info[my_number].battery[0] > 0.5)
		str.setNum(DB_Info->Robot_info[my_number].battery[0]);

	else str.setNum(0);


	bat_info_1->setText(str);
	

	if (DB_Info->Robot_info[my_number].battery[0] < V_9_6_VOLTAGE_LIMIT)
		color=vermelho;
	else
		color=branco;

	plt.setColor(QPalette::Foreground, color);
	bat_info_1->setPalette(plt);



	/* bateria 1 */
	if (DB_Info->Robot_info[my_number].battery[1] > 0.5)
		str.setNum(DB_Info->Robot_info[my_number].battery[1]);

	else str.setNum(0);
	
	bat_info_2->setText(str);

	if (DB_Info->Robot_info[my_number].battery[1] < V_12_0_VOLTAGE_LIMIT)
		color=vermelho;
	else
		color=branco;

	plt.setColor(QPalette::Foreground, color);
	bat_info_2->setPalette(plt);

	/* bateria 2*/
	if (DB_Info->Robot_info[my_number].battery[2] > 0.5)
		str.setNum(DB_Info->Robot_info[my_number].battery[2]);

	else str.setNum(0);

	bat_info_3->setText(str);

	if (DB_Info->Robot_info[my_number].battery[2] < V_12_0_VOLTAGE_LIMIT)
		color=vermelho;
	else
		color=branco;

	plt.setColor(QPalette::Foreground, color);
	bat_info_3->setPalette(plt);


	/* Suporte para a bateria do portátil */
	/* bateria 3*/

	str.setNum(DB_Info->lpBat[my_number].status);

	bat_info_4->setText(str);


	if (DB_Info->lpBat[my_number].charge == DISCHARGING)
		color=azul;
	else
		color=branco;

	if (DB_Info->lpBat[my_number].status < 10) color = vermelho;

	plt.setColor(QPalette::Foreground, color);
	bat_info_4->setPalette(plt);



 }
}


void RobotInfoWidget::ChangeTextSize(int size)
{
	QFont newFont = Robot_number_lb_f->font();

	newFont.setPointSize(size);


	Robot_number_lb_f->setFont(newFont);
    	Robot_number_lb->setFont(newFont);
	Robot_Running_lb_f->setFont(newFont);
   	Robot_Running_lb->setFont(newFont);
 	Robot_Pos_f->setFont(newFont);
    	Robot_position_lb_x_f->setFont(newFont);
    	Robot_position_lb_x->setFont(newFont);
    	Robot_position_lb_y_f->setFont(newFont);
    	Robot_position_lb_y->setFont(newFont);
 	Robot_Vel_f->setFont(newFont);
    	Robot_velocity_lb_x_f->setFont(newFont);
    	Robot_velocity_lb_x->setFont(newFont);
    	Robot_velocity_lb_y_f->setFont(newFont);
    	Robot_velocity_lb_y->setFont(newFont);
    	Orientation_lb_f->setFont(newFont);
   	robot_orientation_lb_deg_f->setFont(newFont);
   	robot_orientation_lb_deg->setFont(newFont);
   	robot_orientation_lb_rad_f->setFont(newFont);
   	robot_orientation_lb_rad->setFont(newFont);
	ball_position_lb_f->setFont(newFont);
    	ball_position_lb_x_f->setFont(newFont);
    	ball_position_lb_x->setFont(newFont);
    	ball_position_lb_y_f->setFont(newFont);
    	ball_position_lb_y->setFont(newFont);
    	ball_velocity_lb_f->setFont(newFont);
    	ball_velocity_lb_x_f->setFont(newFont);
    	ball_velocity_lb_x->setFont(newFont);
    	ball_velocity_lb_y_f->setFont(newFont);
    	ball_velocity_lb_y->setFont(newFont);
    	ball_engaged_lb_f->setFont(newFont);
    	ball_engaged_lb->setFont(newFont);
    	ball_visible_lb_f->setFont(newFont);
    	ball_visible_lb->setFont(newFont);
    	robot_role_lb_f->setFont(newFont);
    	robot_role_lb->setFont(newFont);
    	robot_behavior_lb_f->setFont(newFont);
    	robot_behavior_lb->setFont(newFont);
    	robot_visible_lb_f->setFont(newFont);
    	robot_visible_lb->setFont(newFont);
    	robot_coaching_lb_f->setFont(newFont);
    	robot_coaching_lb->setFont(newFont);
    	team_color_lb_f->setFont(newFont);
    	team_color_lb->setFont(newFont);
	goal_color_lb_f->setFont(newFont);
    	goal_color_lb->setFont(newFont);
    	bat_info_lb_f->setFont(newFont);
    	bat_info_1->setFont(newFont);
    	bat_info_2->setFont(newFont);
    	bat_info_3->setFont(newFont);
	bat_info_4->setFont(newFont);
	kicker_c_label->setFont(newFont);
	kicker_c_info->setFont(newFont);

}
