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

#ifndef __LOGWIDGET_H
#define __LOGWIDGET_H

#include "ui_LogWidget.h"
#include <QTimer>
#include <QFile>
#include <vector>
#include <deque>

#include <fstream>

#include "DB_Robot_info.h"
#include "CoachLogModeInfo.h"

#define LOG_STOPED 0
#define LOG_PLAYING 1
#define MAX_AUTOLOG_REGISTERS 36000	//maximum number of registers (frames) on auto log (36000 is 1 hour)	100 for 10 seconds
#define FILE_DUMP_FREQUENCY 600		//number of frames accumulated before dumping to the file (600 is 1 minute)		10 for each second
#define N_FILES MAX_AUTOLOG_REGISTERS / FILE_DUMP_FREQUENCY

using namespace cambada;

class Log_Information
{
	public:
	FormationInfo finfo;
	CoachInfo coach;
	Robot robot[NROBOTS];
};

class LogWidget: public QWidget, public Ui::LogWG
{
	Q_OBJECT

public:
	LogWidget(QWidget * parent =0);
	~LogWidget();
	void get_info_pointer( DB_Robot_Info * rw);
	void get_coach_pointer( DB_Coach_Info * ci);

private:
	DB_Robot_Info *DB_Info;
	DB_Coach_Info *db_coach_info;
	CoachLogRobotsInfo *coachLogRobots;
	CoachLogModeFlag *coachLogFlag;

	std::deque<Log_Information> LogInfo;

	FILE *LoadLogFile;
	bool ReadyToRead;
	unsigned int currentFrame;
	unsigned int PlayerStatus;
	QTimer *Log_timer;

	//Auto logger variables
	QTimer *saveTimer;		//Timer for the frequency of log data saving
	QString initialMessage;
	QString finalMessage;
	QString logLine;
	std::ofstream autoLogFile;

	unsigned int frameCounter;
	long oneLogEntrySize;

	unsigned int firstFile;
	unsigned int lastFile;
	unsigned int nextFileToWrite;

	void concatAutoLog();
	bool removeDir(const QString dirName);


public slots:
	void OpenFilePressed(void);
	void LoadNextFrame(void);
	void LoadFrame( int frame_number);
	void SetLogViewMode_slot(int check_state);

	void PlayBotPressed(void);
	void Rewind10Pressed(void);
	void Rewind100Pressed(void);
	void Forward10Pressed(void);
	void Forward100Pressed(void);

	void setPlayingStatus(void);
	void setStopedStatus(void);

	void timer_update(void);
	void saveCurrentDBInfo();

signals:
	void SetLogViewMode_signal(bool);

};

#endif
