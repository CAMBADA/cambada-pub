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

#ifndef __MAINWINDOW_H
#define __MAINWINDOW_H

#include <QtGui>

#include "ui_MainWindow.h"
#include "FullInfoWindow.h"
#include "UpdateWidget.h"
#include "Robot.h"

#include <iostream>
#include <string>
using namespace std;
using namespace cambada;


class MyThreadWatcher : public QThread
{

public:
	MyThreadWatcher(QWidget* w, string cmd) 
		{
			this->cmd = cmd; 
			this->labelWidget   = w;
		};
	//virtual ~MyThreadWatcher(){cerr <<"MyThreadWatcher::destructor"<<endl;};


	bool findProcess(string pname)
	{
		FILE* fp;
	//	string cmd = " acpi -b | cut -d':' -f2";
		string cmd = "ps --no-headers -C " + pname;

		if( (fp=popen(cmd.c_str(), "r")) == NULL )
		{
			cerr << "MyThreadWatcher :: findProcess() error"<<endl;
			return false;
		}

		int pid = -1;
	
		const int LINE_BUFFER_SIZE =  128;

		char line[LINE_BUFFER_SIZE];

		bzero(line,sizeof(char)*LINE_BUFFER_SIZE);
		fgets( line, sizeof(char)*LINE_BUFFER_SIZE, fp);
		pclose(fp);
	//	printf("line s: %s  %d\n",line, strlen(line));

		sscanf(line,"%d",&pid);
	//	printf("pid  %d\n", pid);

		if( pid < 0 )
			return false;
		
		return true;
	}

	virtual void run()
	{
		//cerr << cmd.toAscii() << "  :  ";
		//cerr << system( cmd.toAscii() ) << endl;

			QColor red(QColor::fromRgb(191, 63, 63, 255));
			QColor green(QColor::fromRgb(100, 172, 100, 255));
			QColor white(QColor::fromRgb(72,72,72,255));
		while(1)
		{	
			QPalette plt(labelWidget->palette());
//			plt.setColor(QPalette::Foreground, white);
			
			if( findProcess( cmd.c_str() ) )
			{	
				plt.setColor(QPalette::Foreground, green);
				plt.setColor(QPalette::Text, green);
				//cerr <<cmd.c_str() << " : TRUE"<<endl;
			}
			else
			{
				plt.setColor(QPalette::Foreground, red);
				plt.setColor(QPalette::Text, red);
				//plt.setColor( labelWidget->foregroundRole() , red);
				//cerr <<cmd.c_str() <<" : FALSE"<<endl;
			}
			labelWidget->setPalette( plt );
			sleep(1);
		}
	}

private:
	string cmd;
	QWidget* labelWidget;
};





class MWind : public QMainWindow , public Ui::MainWindow
{
	Q_OBJECT


public:
	MWind(QMainWindow *parent=0);
	~MWind();


	//LMOTA
	void incrementOurGoals(){
	  db_coach_info->Coach_Info.ourGoals++;
	  // change referee signal...
	  db_coach_info->Coach_Info.gameState=SIGourGoalScored;
	}
	void incrementTheirGoals(){db_coach_info->Coach_Info.theirGoals++;
	  // change referee signal...
	  db_coach_info->Coach_Info.gameState=SIGtheirGoalScored;}
	const WSColor ourColor() const{
	  assert(db_coach_info!=NULL);
	  return db_coach_info->TeamColor;}
	const WSColor theirColor() const{assert(db_coach_info!=NULL);
	  return (db_coach_info->TeamColor==Magenta?Cyan:Magenta);}



private:
	bool fullscreenflag; 		//indica se a janela se encontra no modo fullscreen
	QMainWindow *mwind;  		//ponteiro para o mainwindow pai (parent*)
	QMainWindow *fullinfowindow;	//ponteiro para a fullinfowindow
	UpdateWidget *UpdateWG;		//widget de actualização da base de dados
	DB_Robot_Info *Robots_info;	//informação dos robots
	DB_Coach_Info *db_coach_info;

	const QPixmap *cambada_logo_pixmap;
	

	RWidget *robots[6];

	FInfoWind *FIW;

	QTimer *UpdateTimer;

	MyThreadWatcher*  commWatcher;
	MyThreadWatcher*  coachWatcher;
	MyThreadWatcher*  loggerWatcher;

protected:
	bool eventFilter(QObject *obj, QEvent *event);


public slots:
    void changeWindowFullScreenMode (void);
	void showFullScreenInfoWindow(void);
	void AllRunBotPressed(void);
	void AllStopBotPressed(void);
	void TeamColorChanged(int team);
	void GoalColorChanged(int goal);
	void transmitCoach(void);
	void allRoleChanged(int role_id);

    void ManualFormationChanged(int state);
	void FormationChanged(int form_id);

	void allObstaclePoints(bool on_off);
	void allDebugPoints(bool on_off);


	void UpdateGameTime(void);
    void UpdateGameParameters(void);
private slots:
    void on_checkBoxManualFormation_stateChanged(int arg1);

    void on_actionTaxi_Follow_Mode_toggled(bool arg1);
    void on_actionTop_View_toggled(bool arg1);
    void on_actionLock_toggled(bool arg1);
};

extern MWind * wind;

#endif
