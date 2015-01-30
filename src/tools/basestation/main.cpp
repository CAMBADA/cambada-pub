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

#include <QApplication>
#include <GL/glut.h>
#include <signal.h>
#include "MainWindow.h"

QApplication* app = NULL;

void closeMe(int sig)
{
	if( sig == SIGINT)
	{
		if( app != NULL )
		{
			app->closeAllWindows();
		}
	}
}

int main(int argc, char *argv[])
{
  	if( signal( SIGINT , closeMe ) == SIG_ERR )
	{
		cerr << "basetation :: Register SIGINT Error"<<endl;
		return 1;
	}
  
	app = new QApplication(argc, argv);
	QMainWindow Mwind;    
	
	glutInit(&argc,argv);

	// LMOTA
	// variable went global
//	MWind wind(&Mwind);
	wind = new MWind(&Mwind);


	Mwind.adjustSize();
   	Mwind.showMaximized();

	QApplication::setStyle(new QPlastiqueStyle); //Descomentar Quando o Qt não se passar...

	int ret =  app->exec();

	if (wind != NULL) delete wind; wind=NULL;

//	system("./rtdb_clean");
//	fprintf(stderr, "fui-me\n");
	return ret;
}
