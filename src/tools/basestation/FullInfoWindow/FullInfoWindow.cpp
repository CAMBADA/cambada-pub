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

#include "FullInfoWindow.h"


FInfoWind::FInfoWind(QMainWindow *parent)
{
	/* Criação da janela */
	setupUi(parent);

	/* Inicialização das variáveis locais */
	fullscreenflag = 0;
	fullinfowindow = parent;

	/* Pintar as letras dos group em branco */
	QColor color = QColor::fromRgb(255,255,255,255);
	QPalette plt;

	plt.setColor(QPalette::Foreground, color);
	GB_Robot1->setPalette(plt); 
	GB_Robot2->setPalette(plt); 
	GB_Robot3->setPalette(plt); 
	GB_Robot4->setPalette(plt); 
	GB_Robot5->setPalette(plt); 
	GB_Robot6->setPalette(plt); 

	/* Conecções */
	connect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	connect(actionView_Full_Screen, SIGNAL(triggered()), this, SLOT(WindowFullScreenMode ()));

	parent->installEventFilter(this);





}
	

FInfoWind::~FInfoWind()
{
	//disconnect(actionQuit, SIGNAL(triggered()), parent, SLOT(close()));
	//disconnect(actionView_Full_Screen, SIGNAL(triggered()), this, SLOT(WindowFullScreenMode ()));
}

void FInfoWind::WindowFullScreenMode (void)
{
	if ( fullscreenflag == 0 )
	{
		fullscreenflag = 1;
		fullinfowindow->showFullScreen();
		actionView_Full_Screen->setText(QString("Leave Full Screen"));

	}
	else
	{
		fullscreenflag = 0;
		fullinfowindow->showMaximized();
		actionView_Full_Screen->setText(QString("View Full Screen"));
	}

}

void FInfoWind::get_info_pointer( DB_Robot_Info * rw)
{
	Robots_info = rw;
	Robot_info_WG_1->get_info_pointer(Robots_info);
	Robot_info_WG_2->get_info_pointer(Robots_info);
	Robot_info_WG_3->get_info_pointer(Robots_info);
	Robot_info_WG_4->get_info_pointer(Robots_info);
	Robot_info_WG_5->get_info_pointer(Robots_info);
	Robot_info_WG_6->get_info_pointer(Robots_info);

	Robot_info_WG_1->get_robot_number( 0 );
	Robot_info_WG_2->get_robot_number( 1 );
	Robot_info_WG_3->get_robot_number( 2 );
	Robot_info_WG_4->get_robot_number( 3 );
	Robot_info_WG_5->get_robot_number( 4 );
	Robot_info_WG_6->get_robot_number( 5 );
}


bool FInfoWind::eventFilter(QObject *obj, QEvent *event)
{
	
	if ( (obj == fullinfowindow) && (event->type() == QEvent::Resize) )
	{
		QSize s = fullinfowindow->size();


		//printf("fullh %d w %d\n", s.rheight(), s.rwidth());

		char size_inc = 0;
		if 	(s.rwidth() > 1232) size_inc=5;
		else if	(s.rwidth() > 1216) size_inc=4;
		else if	(s.rwidth() > 1149) size_inc=3;
		else if	(s.rwidth() > 994 ) size_inc=2;
		else if	(s.rwidth() > 913 ) size_inc=1;

		Robot_info_WG_1->ChangeTextSize(5+size_inc);
		Robot_info_WG_2->ChangeTextSize(5+size_inc);
		Robot_info_WG_3->ChangeTextSize(5+size_inc);
		Robot_info_WG_4->ChangeTextSize(5+size_inc);
		Robot_info_WG_5->ChangeTextSize(5+size_inc);
		Robot_info_WG_6->ChangeTextSize(5+size_inc);
		ChangeTextSize(5+size_inc);
		
		return true;
	}


	return false;
}

void FInfoWind::ChangeTextSize(int size)
{
	QFont newFont;// = our_state_lb_f->font();

	newFont.setPointSize(size);

/*
	our_state_lb_f->setFont(newFont);
    	our_state_lb->setFont(newFont);
       	their_state_lb_f->setFont(newFont);
    	their_state_lb->setFont(newFont);
    	dest_host_lb_f->setFont(newFont);
    	dest_host_lb->setFont(newFont);
    	dest_ip_lb_f->setFont(newFont);
   	dest_ip_lb_f1->setFont(newFont);
   	team_color_lb_f->setFont(newFont);
    	team_color_lb->setFont(newFont);
    	team_side_lb_f->setFont(newFont);
   	team_side_lb->setFont(newFont);
*/
}
