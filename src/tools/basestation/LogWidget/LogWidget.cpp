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

#include "LogWidget.h"
#include <QFileDialog>
#include <iostream>

#include <sys/time.h>
#define DEBUG_TIME 1

using namespace std;

LogWidget::LogWidget(QWidget * parent)
{
	setupUi(parent);
	coachLogRobots = new CoachLogRobotsInfo();
	coachLogFlag = new CoachLogModeFlag();
	coachLogFlag->log = false;
	DB_put(COACHLOGMODEFLAG,coachLogFlag);

	ReadyToRead=true;
	DB_Info=NULL;
	db_coach_info=NULL;
	currentFrame=0;

	PlayerStatus=LOG_STOPED;
	Log_timer=new QTimer();

	connect(LoadFileBot, SIGNAL(clicked()), this, SLOT(OpenFilePressed()));
	connect(MovieSlider, SIGNAL(valueChanged ( int )) ,this, SLOT(LoadFrame( int )));
	connect(LogMode_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(SetLogViewMode_slot(int)));

	connect(Log_timer, SIGNAL(timeout ()), this, SLOT(timer_update()));

	connect(PlayBot, SIGNAL(clicked()), this, SLOT(PlayBotPressed()));
	connect(Rewind100Bot, SIGNAL(clicked()), this,SLOT(Rewind100Pressed()));
	connect(Rewind10Bot, SIGNAL(clicked()), this,SLOT(Rewind10Pressed()));
	connect(FForward100Bot, SIGNAL(clicked()), this, SLOT(Forward100Pressed()));
	connect(FForward10Bot, SIGNAL(clicked()), this, SLOT(Forward10Pressed()));

//Auto logger features
	frameCounter = 0;
	saveTimer = new QTimer();
	saveTimer->start(100);
	connect(saveTimer, SIGNAL( timeout() ), this, SLOT( saveCurrentDBInfo() ) );

	LogInfo.clear();
	logLine.clear();

	initialMessage = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n<Root>\n";
	finalMessage = "</Root>\n";
/*	autoLogFile.open ("../logs/autoLog.xml");
	QString firstWrite = initialMessage + finalMessage;
	autoLogFile.write(firstWrite,firstWrite.size());
	autoLogFile.close();*/
	firstFile = 0;
	lastFile = 0;
	nextFileToWrite = 0;

	if ( !QDir("../logs").exists() ) {
		QDir().mkdir("../logs");
	}
	if ( !QDir("../logs/temp").exists() ) {
		QDir().mkdir("../logs/temp");
	}
}


LogWidget::~LogWidget()
{
	saveTimer->stop();
	concatAutoLog();

	disconnect(LoadFileBot, SIGNAL(clicked()), this, SLOT(OpenFilePressed()));
	disconnect(MovieSlider, SIGNAL(valueChanged ( int )) ,this, SLOT(LoadFrame( int )));
	disconnect(LogMode_CheckBox, SIGNAL(stateChanged(int)), this, SLOT(SetLogViewMode_slot(int)));
	disconnect(Log_timer, SIGNAL(timeout ()), this, SLOT(timer_update()));
	disconnect(PlayBot, SIGNAL(clicked()), this, SLOT(PlayBotPressed()));
	disconnect(Rewind100Bot, SIGNAL(clicked()), this,SLOT(Rewind100Pressed()));
	disconnect(Rewind10Bot, SIGNAL(clicked()), this,SLOT(Rewind10Pressed()));
	disconnect(FForward100Bot, SIGNAL(clicked()), this, SLOT(Forward100Pressed()));
	disconnect(FForward10Bot, SIGNAL(clicked()), this, SLOT(Forward10Pressed()));

	disconnect(saveTimer, SIGNAL( timeout() ), this, SLOT( saveCurrentDBInfo() ) );

	// TODO FIXME Put this here when desctructor is really called
	//Basestation shutting down, end the log file
//	autoLogFile = fopen("../logs/autoLog.xml","a");
//	fprintf(autoLogFile,"</Root>");
//	fclose(autoLogFile);

	delete Log_timer;
}

void LogWidget::OpenFilePressed(void)
{

	QString fpath = QFileDialog::getOpenFileName(this,"Choose the Log File","./");
	
	if (fpath.isEmpty()!= 0)
	{
		printf("LogWidget :: Invalid Log File\n");
		ReadyToRead=false;
		return;
	}

	if(QFile::exists(fpath)!=1)
	{
		printf("LogWidget :: LoadLogFile does not exist\n");
		ReadyToRead=false;
		return;
	}

	LoadLogFile = fopen(fpath.toAscii().constData(), "r");
	if (LoadLogFile==NULL)
	{
		printf("LogWidget :: Opening LoadLogFile Error\n");
		ReadyToRead=false;
		return;
	}

	//WHEN LOADING A FILE, STOP SAVING
	saveTimer->stop();

	//Make sure to clean any previously loaded log
	LogInfo.clear();

	//Check for file info and consistency
	int nItems=0;
//	int temp=0;
//	char line[2048];
	bool exitLoad = false;
	Log_Information LI;

	//FIXME protection that should not exist if the file never got corrupted
	int nTries = 0;

	fscanf(LoadLogFile, "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n<Root>\n");

	while( exitLoad == false )
	{
		nItems = fscanf(LoadLogFile, "<Instance gametime=\"%d\" gamestate=\"%d\" cambada=\"%d\" mf=\"%d\" formation=\"%d\">\n",
				&(LI.coach.time) , &(LI.coach.gameState), &(LI.coach.ourGoals), &(LI.coach.theirGoals), &(LI.finfo.formationIdFreePlay));//FIXME need to add new formationIdSP

		if(nItems!=5)
		{
			if (nItems!=EOF) {
				//FIXME protection that should not exist if the file never got corrupted
				char* dummy = new char[5000];
				fgets(dummy, 5000, LoadLogFile);
				nTries++;
				cerr << "LOGPLAYER ERROR loading header - Try " << nTries << endl;
				continue;
			} else {
				cerr << "LOGPLAYER ERROR loading header - items " << nItems << endl;
				break;
			}
		}

		for( int i = 0 ; i < NROBOTS ; i++ )
		{
			int id, running, rAuto, coaching, oppDribbling, visible, engaged, airborne, own;
			int nRole, nBehavior, nTeamColor, nGoalColor, nGameState;
			unsigned int nIt;

			char currentLine[2048];

//			fprintf(stderr,"STARTING robot %d\n",i+1);

			fgets(currentLine, 2048, LoadLogFile);

//			fprintf(stderr,"%s\n",currentLine);

			nIt = sscanf(currentLine, "<Agent id=\"%d\" running=\"%d\" robotx=\"%f\" roboty=\"%f\" orientation=\"%f\""
								  " velx=\"%f\" vely=\"%f\" vela=\"%f\" role=\"%d\" behavior=\"%d\" stuck=\"%c\""
								  " sposid=\"%d\" nobst=\"%d\" visible=\"%d\" own=\"%d\" engaged=\"%d\" airborne=\"%d\""
								  " absx=\"%f\" absy=\"%f\" relx=\"%f\" rely=\"%f\" z=\"%f\" ballvelx=\"%f\" ballvely=\"%f\""
								  " bat1=\"%f\" bat2=\"%f\""
								  " bat3=\"%f\" oppDribbling=\"%d\" coaching=\"%d\" roleAuto=\"%d\" teamColor=\"%d\""
								  " goalColor=\"%d\" gameState=\"%d\" coordFlag1=\"%d\" coordFlag2=\"%d\" cVecx=\"%f\""
								  " cVecy=\"%f\" dPoint0x=\"%f\" dPoint0y=\"%f\" dPoint1x=\"%f\" dPoint1y=\"%f\""
								  " dPoint2x=\"%f\" dPoint2y=\"%f\" dPoint3x=\"%f\" dPoint3y=\"%f\">\n",
						&id, &running, &(LI.robot[i].pos.x), &(LI.robot[i].pos.y),
						&(LI.robot[i].orientation), &(LI.robot[i].vel.x), &(LI.robot[i].vel.y),
						&(LI.robot[i].angVelocity), &nRole, &nBehavior,
						&(LI.robot[i].stuck), &(LI.finfo.posId[i]), &(LI.robot[i].nObst),
						&visible, &own, &engaged, &airborne, &(LI.robot[i].ball.pos.x), &(LI.robot[i].ball.pos.y),
						&(LI.robot[i].ball.posRel.x), &(LI.robot[i].ball.posRel.y),
						&(LI.robot[i].ball.height), &(LI.robot[i].ball.vel.x), &(LI.robot[i].ball.vel.y),
						&(LI.robot[i].battery[0]), &(LI.robot[i].battery[1]), &(LI.robot[i].battery[2]), &LI.robot[i].opponentDribbling,
						&coaching, &rAuto, &nTeamColor, &nGoalColor,
						&nGameState, &(LI.robot[i].coordinationFlag[0]),
						&(LI.robot[i].coordinationFlag[1]), &(LI.robot[i].coordinationVec.x),
						&(LI.robot[i].coordinationVec.y), &(LI.robot[i].debugPoints[0].x), &(LI.robot[i].debugPoints[0].y),
						&(LI.robot[i].debugPoints[1].x), &(LI.robot[i].debugPoints[1].y), &(LI.robot[i].debugPoints[2].x),
						&(LI.robot[i].debugPoints[2].y), &(LI.robot[i].debugPoints[3].x), &(LI.robot[i].debugPoints[3].y) );

//			cerr << "first nItems: " << nIt << endl;

			if(nIt!=45)
			{
				cerr << "LOGPLAYER ERROR loading record with " << nIt << " items." << endl;
				exitLoad = true;
				break;
			}

			nIt = 0;

			for( unsigned int oo = 0 ; oo < LI.robot[i].nObst ; oo++ )
			{
				float trash;
				fgets(currentLine, 2048, LoadLogFile);
//				fprintf(stderr,"%s\n",currentLine);

				nIt += sscanf(currentLine, "<Obst obstx=\"%f\" obsty=\"%f\" size=\"%f\" teammate=\"%d\"/>\n",
						&(LI.robot[i].obstacles[oo].absCenter.x),&(LI.robot[i].obstacles[oo].absCenter.y), &trash,
						&(LI.robot[i].obstacles[oo].id));
			}

//			cerr << " id " << id << " items " << nIt <<  "  obsRobO " << (int)LI.robot[i].nObst << " result: " << (4 * (LI.robot[i].nObst)) << endl;

			if( nIt !=  (4 * LI.robot[i].nObst) )
			{
				cerr << "LOGPLAYER ERROR loading obst: AGENT "<< i <<" has "<< LI.robot[i].nObst << "obs, total items "<< nIt << endl;
				exitLoad = true;
				break;
			}

			LI.robot[i].role = (RoleID)nRole;
			LI.robot[i].behaviour = (BehaviourID)nBehavior;
			LI.robot[i].teamColor = (WSColor)nTeamColor;
			LI.robot[i].goalColor = (WSColor)nGoalColor;
			LI.robot[i].currentGameState = (WSGameState)nGameState;

			if(running != 0)
				LI.robot[i].running = true;
			else
				LI.robot[i].running = false;

			if(rAuto != 0)
				LI.robot[i].roleAuto = true;
			else
				LI.robot[i].roleAuto = false;

			if(coaching != 0)
				LI.robot[i].coaching = true;
			else
				LI.robot[i].coaching = false;

//			if(oppDribbling != 0)
//				LI.robot[i].opponentDribbling = true;
//			else
//				LI.robot[i].opponentDribbling = false;


			if(visible != 0)
				LI.robot[i].ball.visible = true;
			else
				LI.robot[i].ball.visible = false;

			if(engaged != 0)
				LI.robot[i].ball.engaged = true;
			else
				LI.robot[i].ball.engaged = false;

			if(airborne != 0)
				LI.robot[i].ball.airborne = true;
			else
				LI.robot[i].ball.airborne = false;

			if (own != 0)
				LI.robot[i].ball.own=true;
			else
				LI.robot[i].ball.own=false;


			//Vec rel = LI.robot[i].ball.posAbs - LI.robot[i].me.position;
			//rel.s_rotate( Angle(-LI.robot[i].me.orientation) );

			//LI.robot[i].ball.position = rel;

			fgets(currentLine, 2048, LoadLogFile);
//			fprintf(stderr,"%s\n",currentLine);
//			fscanf(LoadLogFile, "</Agent>\n");
		}

		fscanf(LoadLogFile, "</Instance>\n");
		LogInfo.push_back(LI);
		cerr << "nFrames " << LogInfo.size() << endl;

	}

	fclose(LoadLogFile);
	ReadyToRead=true;
	MovieSlider->setRange(1,LogInfo.size());

	currentFrame = 1;
	LoadFrame(currentFrame);

	return;
}

void LogWidget::LoadNextFrame(void)
{
	if(DB_Info == NULL || db_coach_info == NULL || ReadyToRead==false || LogInfo.size()<=0)
	{
		return;
	}

	if((currentFrame+1)>LogInfo.size())
	{
		return;
	}

	
	LoadFrame(currentFrame+1);

			
}

void LogWidget::LoadFrame( int frame_number )
{
	if(DB_Info == NULL || db_coach_info == NULL || ReadyToRead==false || LogInfo.size()<=0)
	{
		return;
	}

	currentFrame=frame_number;

	//Fill RTBD local representation
	for (int i=0; i<NROBOTS; i++)
	{
		DB_Info->Robot_info[i]=LogInfo[currentFrame-1].robot[i];
		coachLogRobots->robot[i]=LogInfo[currentFrame-1].robot[i];//Log information for coach to calc maps
		if (DB_Info->Robot_info[i].running==0 && DB_Info->Robot_info[i].pos==Vec::zero_vector && DB_Info->Robot_info[i].ball.pos==Vec::zero_vector)
			DB_Info->Robot_status[i]=STATUS_KO;
		else if (DB_Info->Robot_info[i].running==1)
			DB_Info->Robot_status[i]=STATUS_OK;
		else
			DB_Info->Robot_status[i]=STATUS_SB;
	}

	db_coach_info->Coach_Info_in = LogInfo[currentFrame-1].coach;

	//Update FrameLabel
	FrameNumber->setText(QString::number(currentFrame));
	DB_put(COACHLOGROBOTSINFO,coachLogRobots);
	
}

void LogWidget::get_info_pointer( DB_Robot_Info * rw)
{
	DB_Info=rw;
}

void LogWidget::get_coach_pointer( DB_Coach_Info * ci)	
{
	db_coach_info=ci;
}

void LogWidget::SetLogViewMode_slot(int check_state)
{
// ver se vale a pena proteger esta função com a variável ReadytoRead
	if (check_state==2)
	{
		//WHEN IN LOG MODE, STOP SAVING
		saveTimer->stop();
		currentFrame = LogInfo.size();
		MovieSlider->setValue(currentFrame);
		db_coach_info->logTimeOffset = db_coach_info->Coach_Info.time-db_coach_info->gTimeSecOffset;	//When entering log mode, save current time, for restoring later
		emit SetLogViewMode_signal(true);
		coachLogFlag->log= true;
	}
	else if(check_state==0)
	{
		//WHEN NOT IN LOG MODE, START SAVING
		saveTimer->start(100);
		db_coach_info->addLogTimeOffset = true;		//When leaving log mode, flag coachInfo to restore the time before the log viewing started
		emit SetLogViewMode_signal(false);
		coachLogFlag->log= false;
	}
	DB_put(COACHLOGMODEFLAG,coachLogFlag);

	return;
}

void LogWidget::PlayBotPressed(void)
{
	if (PlayerStatus==LOG_STOPED)
	{
		setPlayingStatus();
	}
	else
	{
		setStopedStatus();
	}
}

void LogWidget::setPlayingStatus(void)
{

	if(ReadyToRead==true)
	{
		PlayerStatus=LOG_PLAYING;
		Log_timer->start(FrameTime->value());
	}
}

void LogWidget::setStopedStatus(void)
{
	PlayerStatus=LOG_STOPED;
	Log_timer->stop();
}

void LogWidget::timer_update(void)
{
	
	if((currentFrame+1)>LogInfo.size())
	{
		return;
	}

	MovieSlider->setValue(currentFrame+1);
}

void LogWidget::Rewind10Pressed(void)
{

	if(ReadyToRead==false)
		return;

	if(PlayerStatus==LOG_PLAYING)
	{
		if((int)(currentFrame-1)<0)
			return;
		setStopedStatus();
		MovieSlider->setValue(currentFrame-1);
		setPlayingStatus();
	}
	else
	{
		if((int)(currentFrame-1)<0)
			return;
		MovieSlider->setValue(currentFrame-1);
	}
}

void LogWidget::Rewind100Pressed(void)
{
	if(ReadyToRead==false)
		return;

	if(PlayerStatus==LOG_PLAYING)
	{
		if((int)(currentFrame-100)<0)
			return;
		setStopedStatus();
		MovieSlider->setValue(currentFrame-100);
		setPlayingStatus();
	}
	else
	{
		if((int)(currentFrame-100)<0)
			return;
		MovieSlider->setValue(currentFrame-100);
	}
}

void LogWidget::Forward10Pressed(void)
{
	if(ReadyToRead==false)
		return;

	if(PlayerStatus==LOG_PLAYING)
	{
		if((currentFrame+1)>LogInfo.size())
			return;
		setStopedStatus();
		MovieSlider->setValue(currentFrame+1);
		setPlayingStatus();
	}
	else
	{
		if((currentFrame+1)>LogInfo.size())
			return;
		MovieSlider->setValue(currentFrame+1);
	}
}

void LogWidget::Forward100Pressed(void)
{
	if(ReadyToRead==false)
		return;

	if(PlayerStatus==LOG_PLAYING)
	{
		if((currentFrame+100)>LogInfo.size())
			return;
		setStopedStatus();
		MovieSlider->setValue(currentFrame+100);
		setPlayingStatus();
	}
	else
	{
		if((currentFrame+100)>LogInfo.size())
			return;
		MovieSlider->setValue(currentFrame+100);
	}
}



/** This is a periodic function, responsible for filling in the logInformation vector with the current status (thus making log immediately available) and also append to the String that will be flushed to the automatic file.*/
void LogWidget::saveCurrentDBInfo()
{
//	appReadyToExit = false;
//	Robot robot;//[N_CAMBADAS];
//	CoachInfo coach;
//	FormationInfo finfo;

#if DEBUG_TIME
	struct timeval initTime, deltaTime;

	gettimeofday( &initTime , NULL );
#endif


	Log_Information currentData;

	if (DB_Info == NULL || db_coach_info == NULL) {
		fprintf(stderr,"Error wrinting log: RTDB pointer null\n");
		return;
	}

	//Save current coach data
	currentData.coach = db_coach_info->Coach_Info_in;
	//Create current coach data String
	logLine += QString("<Instance gametime=\"" + QString::number(currentData.coach.time) + "\" gamestate=\"" + QString::number(currentData.coach.gameState) + "\" cambada=\"" + QString::number(currentData.coach.ourGoals) + "\" mf=\"" + QString::number(currentData.coach.theirGoals) + "\" formation=\"" + QString::number(0) + "\">\n");

//	sprintf("<Instance gametime=\"%d\" gamestate=\"%d\" cambada=\"%d\" mf=\"%d\" formation=\"%d\">",
//		currentData.coach.time , currentData.coach.gameState, currentData.coach.ourGoals, currentData.coach.theirGoals, 0);//FIXME finfo.formationID);

	for( int i = 0 ; i < N_CAMBADAS ; i++ )
	{
		//Save current robot data
		currentData.robot[i] = DB_Info->Robot_info[i];

		if( DB_Info->lifetime[i] > NOT_RUNNING_TIMEOUT )
			currentData.robot[i].running = false;

		logLine += QString("<Agent id=\"" + QString::number(currentData.robot[i].number) + "\" running=\"" + QString::number(currentData.robot[i].running) + "\"");
		logLine += " robotx=\"" + QString::number((double)currentData.robot[i].pos.x,'f',2) + "\" roboty=\"" + QString::number((double)currentData.robot[i].pos.y,'f',2) + "\" orientation=\"" + QString::number((double)currentData.robot[i].orientation,'f',2) + "\"";
		logLine += " velx=\""+ QString::number((double)currentData.robot[i].vel.x,'f',2) + "\" vely=\"" + QString::number((double)currentData.robot[i].vel.y,'f',2) + "\" vela=\"" + QString::number((double)currentData.robot[i].angVelocity,'f',2) + "\"";
		logLine += " role=\""+ QString::number(currentData.robot[i].role) + "\"";
		logLine += " behavior=\"" + QString::number(currentData.robot[i].behaviour) + "\"";
		logLine += " stuck=\"" + QString::number(currentData.robot[i].stuck) + "\"";
		logLine += " sposid=\"" +QString::number(0) + "\""; //FIXME +QString::number(finfo.formationSPos[i]) + "\"";
		logLine += " nobst=\"" + QString::number(currentData.robot[i].nObst)+ "\"";
		logLine += " visible=\"" + QString::number(currentData.robot[i].ball.visible) + "\"";
		logLine += " own=\"" + QString::number(currentData.robot[i].ball.own) + "\"";
		logLine += " engaged=\"" + QString::number(currentData.robot[i].ball.engaged) + "\"";
		logLine += " airborne=\"" + QString::number(currentData.robot[i].ball.airborne) + "\"";
		logLine += " absx=\"" + QString::number((double)currentData.robot[i].ball.pos.x,'f',2) + "\" absy=\"" + QString::number((double)currentData.robot[i].ball.pos.y,'f',2) + "\" relx=\"" + QString::number((double)currentData.robot[i].ball.posRel.x,'f',2) + "\" rely=\"" + QString::number((double)currentData.robot[i].ball.posRel.y,'f',2) + "\" z=\"" + QString::number((double)currentData.robot[i].ball.height,'f',2) + "\"";
		logLine += " ballvelx=\"" + QString::number((double)currentData.robot[i].ball.vel.x,'f',2) + "\" ballvely=\"" + QString::number((double)currentData.robot[i].ball.vel.y,'f',2) + "\"";	//teminator was here:   "\">\n"

		//New information (complete Robot) added before RoboCup 2013
		logLine += " bat1=\"" + QString::number((double)currentData.robot[i].battery[0],'f',2) + "\" bat2=\"" + QString::number((double)currentData.robot[i].battery[1],'f',2) +  "\" bat3=\"" + QString::number((double)currentData.robot[i].battery[2],'f',2) + "\"";
		logLine += " oppDribbling=\"" + QString::number(currentData.robot[i].opponentDribbling) + "\"";
		logLine += " coaching=\"" + QString::number(currentData.robot[i].coaching) + "\"";
		logLine += " roleAuto=\"" + QString::number(currentData.robot[i].roleAuto) + "\"";
		logLine += " teamColor=\"" + QString::number(currentData.robot[i].teamColor) + "\"";
		logLine += " goalColor=\"" + QString::number(currentData.robot[i].goalColor) + "\"";
		logLine += " gameState=\"" + QString::number(currentData.robot[i].currentGameState) + "\"";
		logLine += " coordFlag1=\"" + QString::number(currentData.robot[i].coordinationFlag[0]) + "\" coordFlag2=\"" + QString::number(currentData.robot[i].coordinationFlag[1]) + "\" cVecx=\"" + QString::number((double)currentData.robot[i].coordinationVec.x,'f',2) + "\" cVecy=\"" + QString::number((double)currentData.robot[i].coordinationVec.y,'f',2) + "\"";
		logLine += " dPoint0x=\"" + QString::number((double)currentData.robot[i].debugPoints[0].x,'f',2) + "\" dPoint0y=\"" + QString::number((double)currentData.robot[i].debugPoints[0].y,'f',2) + "\" dPoint1x=\"" + QString::number((double)currentData.robot[i].debugPoints[1].x,'f',2) + "\" dPoint1y=\"" + QString::number((double)currentData.robot[i].debugPoints[1].y,'f',2) + "\" dPoint2x=\"" + QString::number((double)currentData.robot[i].debugPoints[2].x,'f',2) + "\" dPoint2y=\"" + QString::number((double)currentData.robot[i].debugPoints[2].y,'f',2) + "\" dPoint3x=\"" + QString::number((double)currentData.robot[i].debugPoints[3].x,'f',2) + "\" dPoint3y=\"" + QString::number((double)currentData.robot[i].debugPoints[3].y,'f',2) + "\">\n";

		for( unsigned int o = 0 ; o < currentData.robot[i].nObst ; o++ )
		{
			logLine += "<Obst obstx=\"" +	QString::number((double)currentData.robot[i].obstacles[o].absCenter.x,'f',2) + "\" obsty=\"" + QString::number((double)currentData.robot[i].obstacles[o].absCenter.y,'f',2) + "\" size=\""
			+ QString::number(0.5,'f',2) +  "\" teammate=\"" + QString::number(currentData.robot[i].obstacles[o].id) + "\"/>\n";
		}

		logLine += "</Agent>\n";
	}
	logLine += "</Instance>\n";

	LogInfo.push_back(currentData);
	if (LogInfo.size() == 1) {
		oneLogEntrySize = logLine.size();
	} else if (LogInfo.size() > MAX_AUTOLOG_REGISTERS) {
		LogInfo.pop_front();
	}
	frameCounter++;
	MovieSlider->setRange(1,LogInfo.size());

#if DEBUG_TIME
//	gettimeofday( &deltaTime, NULL );
//	fprintf(stderr,"LOG_UPDATE time: %ld\n", (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - (initTime.tv_sec*1000 + initTime.tv_usec/1000) );
#endif

	if ( frameCounter >= FILE_DUMP_FREQUENCY ) {
		if ( (nextFileToWrite == firstFile) && (nextFileToWrite != lastFile) ) {
			firstFile++;
			if (firstFile > N_FILES-1) {
				firstFile = 0;
			}
		}

		char currentFileName[30];
		sprintf(currentFileName, "../logs/temp/autoLog%02d.xml",nextFileToWrite);
		autoLogFile.open(currentFileName);

		autoLogFile.write(logLine.toStdString().c_str(), logLine.size());			//add the text registers kept in memory since the last dump
		autoLogFile.flush();

		autoLogFile.close();
		logLine.clear();									//clear the text registers in memory
		frameCounter = 0;									//reset counter of number of frames in memory

		nextFileToWrite++;
		if ( (nextFileToWrite-lastFile) > 1 )
			lastFile++;

		if (nextFileToWrite > N_FILES-1) {
			nextFileToWrite = 0;
		}
		if (lastFile > N_FILES-1) {
			lastFile = 0;
		}

	#if DEBUG_TIME
		gettimeofday( &deltaTime, NULL );
		fprintf(stderr,"LOG_UPDATE time with fileWrite: %ld\n", (deltaTime.tv_sec*1000 + deltaTime.tv_usec/1000) - (initTime.tv_sec*1000 + initTime.tv_usec/1000) );
	#endif
	}
}


void LogWidget::concatAutoLog() {
	fprintf(stderr,"Writing log file, please be patient!\n");
	autoLogFile.open("../logs/autoLog.xml");
	autoLogFile.write(initialMessage.toStdString().c_str(), initialMessage.size());	//start the file with the initial mesasge

	if ( !(firstFile == lastFile && firstFile == nextFileToWrite) ) {
	//	fprintf(stderr,"firstFile: %d, lastFile: %d\n",firstFile,lastFile);

		unsigned int i=firstFile;
		bool endCycle=false, endNext=false;
		while (!endCycle) {
			char currentFileName[30];
			sprintf(currentFileName, "../logs/temp/autoLog%02d.xml",i);
	//		fprintf(stderr,"Reading file %02d\n",i);

			ifstream currentFile(currentFileName);
			currentFile.seekg(0,currentFile.end);			//put pointer at the end of the file
			long currentSize = currentFile.tellg();			//when pointer at the end, the current position is the file size

			char* buffer;
			buffer = new char[currentSize];

			currentFile.seekg(0);						//go to beggining
			currentFile.read(buffer, currentSize);		//read the whole file
			currentFile.close();

			autoLogFile.write(buffer, currentSize);

			if (endNext)
				endCycle = true;

			i++;
			if (i == lastFile) {
				endNext = true;
			}
			if (i > N_FILES-1) {
				i = 0;
			}
		}
	}
	autoLogFile.write(logLine.toStdString().c_str(), logLine.size());
	autoLogFile.write(finalMessage.toStdString().c_str(), finalMessage.size());
	autoLogFile.flush();
	autoLogFile.close();

	removeDir("../logs/temp");
	if ( QDir("../logs/temp").exists() ) {
		QDir().rmdir("../logs/temp");
	}
}


bool LogWidget::removeDir(const QString dirName)
{
	bool result;
	QDir dir(dirName);

	if (dir.exists()) {
		Q_FOREACH(QFileInfo info, dir.entryInfoList(QDir::NoDotAndDotDot | QDir::System | QDir::Hidden  | QDir::AllDirs | QDir::Files, QDir::DirsFirst))
		{
			if (info.isDir()) {
				result = removeDir(info.absoluteFilePath());
			}
			else {
				result = QFile::remove(info.absoluteFilePath());
			}

			if (!result) {
				return result;
			}
		}
		result = dir.rmdir(dirName);
	}
	return result;
}
