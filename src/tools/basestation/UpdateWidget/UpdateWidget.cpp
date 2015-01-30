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

#include "UpdateWidget.h"



UpdateWidget::UpdateWidget()
{


	Update_timer=new QTimer();
	LogViewMode=false;
	if( DB_init() == 0 || DB_init() == 0 || DB_init() == 0 )
	{
		printf("RtDB connection successful.\n");
		valid_connection = true;
	}
	else
	{
		printf("RtDB connection NOT available.\n");
		valid_connection = false;
	}
	

	if (valid_connection)
	{
		connect(Update_timer, SIGNAL(timeout ()), this, SLOT(UpdateInfo()));
		//connect(Update_timer, SIGNAL(timeout ()), this, SLOT(transmitCoach()));
		Update_timer->start(20);
	}

	/* incializações da base de dados */
    for (unsigned i=0; i<NROBOTS; i++){
		Robots_info.lifetime[i] = 0;
        Robots_info.lpBat[i].status = 0;
        Robots_info.lpBat[i].charge = 0;
        Robots_info.Robot_status[i] = STATUS_NA;
        Robots_info.Robot_info[i].currentGameState = stopRobot;
    }

	DB_Coach.GameTime.setHMS(0,0,0);
	DB_Coach.GamePart=0;



}

UpdateWidget::~UpdateWidget()
{
	disconnect(Update_timer, SIGNAL(timeout ()), this, SLOT(UpdateInfo()));
	delete Update_timer;
	if( valid_connection )
	{
		DB_free();
		printf("RtDB connection released.\n");
	}
}

void UpdateWidget::UpdateInfo(void)
{
	if (LogViewMode)
	{
		return;
	}
	//return;
	
	Robot temp_info;
	LaptopInfo lpBatTemp[NROBOTS];
	CoachInfo coach_temp;
    FormationInfo formation_temp;

	int CoachLifetime;
	bool valid_info=true;
	if( (CoachLifetime=DB_get( Whoami(), COACH_INFO, (void*)&coach_temp)) == -1 )
		if((CoachLifetime=DB_get( Whoami(), COACH_INFO, (void*)&coach_temp)) == -1 )
		{
			printf("RtDB read coach info error\n");
			valid_info=false;
		}

	if(valid_info)
	{
        DB_Coach.Coach_Info_in=coach_temp;
        if(formationCombo != NULL)
            formationCombo->setCurrentIndex(coach_temp.half);
    }

    if( (CoachLifetime=DB_get( Whoami(), FORMATION_INFO, (void*)&formation_temp)) == -1 )
        if((CoachLifetime=DB_get( Whoami(), FORMATION_INFO, (void*)&formation_temp)) == -1 )
        {
            printf("RtDB read coach info error\n");
            valid_info=false;
        }

    if(valid_info)
    {
        if(formationCombo != NULL)
            formationCombo->setCurrentIndex(formation_temp.formationIdFreePlay);//FIXME need to add new formationIdSP

    }

	for(int i=0; i<NROBOTS;i++)
	{
		long lifetime;
		valid_info = true;
		if( (lifetime=DB_get( i+1, ROBOT_WS, (void*)&temp_info)) == -1 )
			if( (lifetime=DB_get( i+1, ROBOT_WS, (void*)&temp_info)) == -1 )
			{
				valid_info=false;
				Robots_info.Robot_status[i] = STATUS_KO;
				printf("RtDB read error\n");
			}

		int batteryLifetime;
		if( (batteryLifetime=DB_get( i+1, LAPTOP_INFO, (void*)&lpBatTemp[i])) == -1 )
			if( (batteryLifetime=DB_get( i+1, LAPTOP_INFO, (void*)&lpBatTemp[i])) == -1 )
			{
				printf("RtDB read bat info error\n");
			}


		if( batteryLifetime <0 || batteryLifetime > 30000 )
		{
			Robots_info.lpBat[i].charge= -1;
			Robots_info.lpBat[i].status= -1;
		}
		else
		{
			Robots_info.lpBat[i].charge=lpBatTemp[i].charge;
			Robots_info.lpBat[i].status=lpBatTemp[i].status;
		}	

        //fprintf(stderr, "RTDB: robot %d -> validinfo = %d lifetime = %d\n", i, valid_info, lifetime);
		if(valid_info )
		{
			memcpy(&Robots_info.Robot_info[i], &temp_info, sizeof(Robot));
			if (lifetime < NOT_RUNNING_TIMEOUT)
			{

				if (Robots_info.Robot_info[i].number == 0)
					Robots_info.Robot_status[i] = STATUS_NA;

				else if (Robots_info.Robot_info[i].running )
					Robots_info.Robot_status[i] = STATUS_OK;
				else
					Robots_info.Robot_status[i] = STATUS_SB;
			}
			else
				Robots_info.Robot_status[i] = STATUS_NA;
		}

		Robots_info.lifetime[i] = lifetime;


		//printf("robot %d status %d running %d number %d\n", i,Robots_info.Robot_status[i], Robots_info.Robot_info[i].me.running, temp_info.me.number);
	}




}

DB_Robot_Info * UpdateWidget::get_info_pointer(void)
{
	return &Robots_info;
}

void UpdateWidget::setFormationCombo(QComboBox * _formationCombo){
    formationCombo = _formationCombo;
}

void UpdateWidget::transmitFormation(int formationId)
{
    FormationInfo formation_temp;

    bool valid_info=true;

    if( DB_get( Whoami(), FORMATION_INFO, (void*)&formation_temp) == -1 )
            valid_info=false;


    if(valid_info)
    {
        formation_temp.formationIdFreePlay = formationId; //FIXME need to add new formationIdSP
        DB_put( FORMATION_INFO , (void*) &formation_temp );
    }

}

void UpdateWidget::transmitCoach(void)
{
	// update game time

	if (DB_Coach.addLogTimeOffset) {	//When log viewing is disabled, restore the time previous
		DB_Coach.GameTime.restart();
		DB_Coach.addLogTimeOffset = false;
	}
	DB_Coach.Coach_Info.time = DB_Coach.GameTime.elapsed()/1000;
	DB_Coach.Coach_Info.time += DB_Coach.gTimeSecOffset;
	DB_Coach.Coach_Info.time += DB_Coach.logTimeOffset;

	if (valid_connection) {
		if( (DB_put(COACH_INFO, (void*)(&DB_Coach.Coach_Info))) == -1 )
			fprintf(stderr,"Error cenas");
	}
}

DB_Coach_Info * UpdateWidget::get_coach_pointer(void)
{
	return &DB_Coach;
}

void UpdateWidget::SetLogViewMode (bool on_off)
{
	LogViewMode=on_off;
}
