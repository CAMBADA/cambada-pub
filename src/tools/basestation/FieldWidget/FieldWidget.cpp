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

#include <QtGui>
#include <QtOpenGL>

#include <math.h>

#include <GL/glut.h>
#include "FieldWidget.h"
#include "ConfigXML.h"

using namespace cambada;
using namespace cambada::util;

//==================================================== Constructor ==========================================
FieldWidget::FieldWidget(QWidget *parent)
    : QGLWidget(parent)
{
	//return; // FIXME (this widget is causing a segfault in basestation process)

	/* Flip vertical e horizontal */
	flipV=-1;
	flipH=-1;


	/* Inicializa as listas de objectos a 0 */
	field=0;
	robot=0;
	ball=0;
	select=0;
	robotAngle=0;
	debug_point=0;
	ballVel=0;			//added by Joao
	
	ConfigXML config;
	if( config.parse("../config/cambada.conf.xml") == false )
	{
		cerr << "ERROR " << endl;
		exit(1);	
	}

	_FIELD_HEIGHT			= config.getField("field_length")/1000.0;
	_FIELD_WIDTH			= config.getField("field_width")/1000.0;
	_LINE_THICKNESS			= config.getField("line_thickness")/1000.0;
	_GOAL_AREA_LENGTH		= config.getField("goal_area_length")/1000.0;
	_GOAL_AREA_WIDTH		= config.getField("goal_area_width")/1000.0;
	_PENALTY_AREA_LENGTH	= config.getField("penalty_area_length")/1000.0;
	_PENALTY_AREA_WIDTH		= config.getField("penalty_area_width")/1000.0;
	_CENTER_CIRCLE_RADIUS	= config.getField("center_circle_radius")/1000.0;
	_BALL_DIAMETER			= config.getField("ball_diameter")/1000.0;
	_CORNER_CIRCLE_RADIUS	= config.getField("corner_arc_radius")/1000.0;
	_PENALTY_MARK_DISTANCE	= config.getField("penalty_marker_distance")/1000.0;
	_BLACK_POINT_WIDTH		= _FIELD_WIDTH/4.0;
	_BLACK_POINT_LENGTH		= config.getField("penalty_marker_distance")/1000.0;
	_ROBOT_RADIUS			= config.getField("robot_radius")/1000.0;

	/* Constantes de trabalho */
	Pi = 3.14159265358979323846;
	NumSectors = 400;
	DB_Info = NULL;
	db_coach_info = NULL;


	/* Criação do update_timer */
    Update_timer = new QTimer();
    connect(Update_timer, SIGNAL(timeout()), this, SLOT(update_robot_info()));
    Update_timer->start(50);

	/* Inicialização das cores do campo */
    FieldGreen = QColor::fromRgb(71,164,50,255);//QColor::fromCmykF(0.40, 0.0, 1.0, 0.0).dark();
    FieldBlack = QColor::fromRgb(0,0,0,255);
	FieldWhite = QColor::fromRgb(255,255,255,255);
	FieldBlue = QColor::fromRgb(128,160,191,255);
	FieldYellow = QColor::fromRgb(255,191,105,255);
	

	/* Inicializações da informação dos Robots */
	robot_info[0].color= QColor::fromRgb(244,194,194,255);//QColor::fromRgb(0,0,0,255);
	robot_info[1].color= QColor::fromRgb(255,216,0,255);
	robot_info[2].color= QColor::fromRgb(255,153,153,255);
	robot_info[3].color= QColor::fromRgb(188, 143, 143);//QColor::fromRgb(50,205,50,255);//QColor::fromRgb(255,0,255,255);
	robot_info[4].color= QColor::fromRgb(201,160,220,255);
	robot_info[5].color= QColor::fromRgb(115, 194, 251,255);

	robot_info[0].Pos=Vec(-3,-3);
		robot_info[0].PosBall=Vec(-2,-2);
			robot_info[0].angle=0;
				for (int i=0; i< 4 ; i++) robot_info[0].Debug_point[i]=Vec(-3.5-i*0.5,-3);
	robot_info[1].Pos=Vec(-3,0);
		robot_info[1].PosBall=Vec(-2,0);
			robot_info[1].angle=60;
				for (int i=0; i< 4 ; i++) robot_info[1].Debug_point[i]=Vec(-3.5-i*0.5,+0);
	robot_info[2].Pos=Vec(-3,+3);
		robot_info[2].PosBall=Vec(-2,+2);
			robot_info[2].angle=120;
				for (int i=0; i< 4 ; i++) robot_info[2].Debug_point[i]=Vec(-3.5-i*0.5,+3);
	robot_info[3].Pos=Vec(+3,-3);
		robot_info[3].PosBall=Vec(+2,-2);
			robot_info[3].angle=180;
				for (int i=0; i< 4 ; i++) robot_info[3].Debug_point[i]=Vec(+3.5+i*0.5,-3);
	robot_info[4].Pos=Vec(3,0);
		robot_info[4].PosBall=Vec(+2,0);	
			robot_info[4].angle=240;
				for (int i=0; i< 4 ; i++) robot_info[4].Debug_point[i]=Vec(3.5+i*0.5,+0);
	robot_info[5].Pos=Vec(+3,+3);
		robot_info[5].PosBall=Vec(+2,+2);
			robot_info[5].angle=0;
				for (int i=0; i< 4 ; i++) robot_info[5].Debug_point[i]=Vec(+3.5+i*0.5,+3);
		
	
	IsAnySelected = 0;
	for (int i=0; i< NROBOTS ; i++)	robot_info[i].selected=0;
	for (int i=0; i< NROBOTS ; i++)	robot_info[i].visible=0;
	for (int i=0; i< NROBOTS ; i++)	
		for(int j=0; j<4; j++)
                 robot_info[i].Debug_point_visible[j] = 0;
		//robot_info[5].visible=0;
	for (int i=0; i< NROBOTS ; i++)	robot_info[i].ballvisible=0;
	for (int i=0; i< NROBOTS ; i++)	robot_info[i].ballengaged=0;

	//Obstacles
	for (int i=0; i< NROBOTS ; i++)	
	{
		robot_info[i].nObst=4;
		robot_info[i].obstacleVisible=0;
	}
	for (int i=0; i< NROBOTS ; i++)
	{
		for (unsigned int j=0; j< robot_info[i].nObst; j++)
		{
			robot_info[i].obstacleAbsCenter[j].x = robot_info[i].Pos.x+0.5;
			robot_info[i].obstacleAbsCenter[j].y = robot_info[i].Pos.y+0.5;

			robot_info[i].obstacleWidth[j]=0.5;
			robot_info[i].obstacleteamMate[j]=i;
		}
	}

	/* Stuck */
	for (int i=0; i< NROBOTS ; i++)	robot_info[i].stuck=0;

}

//==================================================== Destructor ==========================================


FieldWidget::~FieldWidget()
{
	/* Destroi todas as listas de objectos */
    makeCurrent();
    glDeleteLists(field, 1);
    glDeleteLists(robot, 1);
    glDeleteLists(ball, 1);
    glDeleteLists(select, 1);
    glDeleteLists(ball_engaged, 1);
    glDeleteLists(robotAngle, 1);
    glDeleteLists(debug_point, 1);
    glDeleteLists(obstacle_point, 1);
    glDeleteLists(ballVel, 1);			//added by Joao

    delete Update_timer;
}

//==================================================== initializeGL ==========================================

void FieldWidget::initializeGL()
{

	/* Atribuição da cor ao campo */
	qglClearColor(FieldGreen);

	/*Construção das listas de objectos */
	field = makeField();
	robot = makeRobot();
	ball= makeBall();
	select= makeSelect();
	ball_engaged = makeBallEngaged();
	robotAngle = makeRobotAngle();
	debug_point = makeDebugPoint();
	obstacle_point = makeObstaclePoint();
	ballVel = makeBallVel();		//added by Joao
	robotContour = makeRobotContour();

	/* Configurações do OGL */
	//glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);

	/*ENABLE TRANSPARENCY*/
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

//==================================================== paintGL ==========================================

void FieldWidget::paintGL()
{
	//return;
//NOTA: A forma como se desenham os robots não é irrelevante
//	as figuras que aparecem por cima são as primeiras a serem desenhadas

	int countFreePlay=0;
	int countOther=0;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	int	closerToBall=-1;
	int closerDist=1000;

	for (unsigned i=0; i<NROBOTS; i++)
	{
		/*Check if the robot is in standby and make its color semi-transparent if it is*/
//		if( (db_coach_info->GamePart == 1) || (db_coach_info->GamePart == 2) && (DB_Info->Robot_status[i]==STATUS_SB) )
		if( (DB_Info->Robot_status[i]==STATUS_SB) )
		{
			robot_info[i].color.setAlpha(120);
		}
		else
		{
			robot_info[i].color.setAlpha(255);
		}

		/* Select robot */
		if(IsAnySelected)
		{
			//Se algum robot estiver seleccionado procura e desenha o quadrado
			//á volta do respectivo robot
			for (unsigned i=0; i<NROBOTS; i++)
			{
				if (robot_info[i].selected)
				{
					glLoadIdentity();
					glPushMatrix();
					qglColor(FieldBlack);
					glTranslated(Sel_Pos.x, Sel_Pos.y, 0.0);
					glCallList(select);
					glPopMatrix();
				}
			}
		}

		/* Robots text */
		if (robot_info[i].visible)
		{
		
			glPushMatrix();
			//glLoadIdentity();

			/*Define text (number) color*/
			QColor textColor=FieldWhite;	//default is white
			if(DB_Info->Robot_info[i].role == rStriker)	//if Striker, make it red
				textColor = QColor::fromRgb(255,0,0,255);
			else if(robot_info[i].opponentDribbling > N_CAMBADAS)		//if opponentDribbling make it black
				textColor = QColor::fromRgb(0,0,0,255);

			if( (DB_Info->Robot_status[i]==STATUS_SB) )		//make it semi-transparent if in standby
			{
				textColor.setAlpha(120);
			}

			qglColor(textColor);

			glRasterPos3f(robot_info[i].Pos.x- _ROBOT_RADIUS/6+( ((-flipH+1)/2) * _ROBOT_RADIUS/2.5), robot_info[i].Pos.y+ _ROBOT_RADIUS/4-( ((-flipV+1)/2) * _ROBOT_RADIUS/2),0.0);
			//glRasterPos3f(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.0);
			//glRasterPos3f(robot_info[i].Pos.x-( ((flipH+1)/2) * _ROBOT_RADIUS/2), robot_info[i].Pos.y+( ((flipV+1)/2) * 2*ROBOT_RADIUS/3), 0.0);
			//glTranslated(0.0, 0.0, 0.0);
			robot_text (i+1);
			glPopMatrix();
		}

		/* Ball */
		// Se o robot e a bola forem visiveis a bola é desenhada
		if (robot_info[i].visible && robot_info[i].ballvisible)
		{
			// Se a bola estiver engaged desenha a bola e o contorno no sitio certo
			if(robot_info[i].ballengaged)
			{
				double angle=robot_info[i].angle;
				//if (flipV == -1) angle += 180;

				glLoadIdentity();
				glPushMatrix();
				qglColor(FieldBlack);
				glTranslated(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.02);	//ZZ changed by Joao
				glRotated(angle, 0.0, 0.0, 1.0);
				glCallList(ball_engaged);
				glPopMatrix();				

				glLoadIdentity();
				glPushMatrix();
				qglColor(robot_info[i].color);
				glTranslated(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.02);	//ZZ changed by Joao
				glRotated(robot_info[i].angle, 0.0, 0.0, 1.0);
				glTranslated(0, _ROBOT_RADIUS, 0);
				glCallList(ball);
				glPopMatrix();
			}
			else
			{
				//Se não, desenha a bola na posição recebida da rtdb
				glPushMatrix();
				qglColor(robot_info[i].color);
				glTranslated(robot_info[i].PosBall.x, robot_info[i].PosBall.y, 0.02);	//ZZ changed by Joao
				glCallList(ball);
				glPopMatrix();
			}

			if ( robot_info[i].velBall.length() < closerDist )
			{
				closerToBall = i;
				closerDist = robot_info[i].velBall.length();
			}
			//Vectores de velocidade
/*			if ( true )//Debug_point_visible )	//added by Joao
			{
				glPushMatrix();
				if ( robot_info[i].velBall.length() > 0.70 )
					qglColor(robot_info[i].color);
				else
					qglColor("red");
				glTranslated(robot_info[i].PosBall.x, robot_info[i].PosBall.y, 0.02);
				glRotated(robot_info[i].dirBall, 0.0, 0.0, 1.0);
				glScaled(robot_info[i].velBall.length(), 1.0, 1.0);
				glCallList(ballVel);
				glPopMatrix();
			}*/
		}





		/* Robot Angle */
/*		// Se o robot estiver visivel desenha-o no campo
		if (robot_info[i].visible)
		{
			//glLoadIdentity();
			glPushMatrix();
			qglColor(FieldBlack);
			glTranslated(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.0);

			double angle=robot_info[i].angle;
			//if (flipV == -1) angle += 180;
				glRotated(angle, 0.0, 0.0, 1.0);

			glCallList(robotAngle);

			glPopMatrix();
		}
*/

		/* Robots */
		// Se o robot estiver visivel desenha-o no campo
		if (robot_info[i].visible)
		{
			//glLoadIdentity();
			glPushMatrix();
			qglColor(robot_info[i].color);
			glTranslated(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.0);

			double angle=robot_info[i].angle;
			//if (flipV == -1) angle += 180;
				glRotated(angle, 0.0, 0.0, 1.0);

			glCallList(robot);

			glPopMatrix();
		}


		/* Debug Points text */	

		for (int dp=0; dp<NDEBUG_POINTS; dp++)
		{		
			if ( robot_info[i].Debug_point_visible[dp] && robot_info[i].visible )
			{	
				glPushMatrix();
				qglColor(FieldWhite);
		
				glRasterPos3f(robot_info[i].Debug_point[dp].x- _ROBOT_RADIUS/6+( ((-flipH+1)/2) * _ROBOT_RADIUS/2.5), robot_info[i].Debug_point[dp].y+ _ROBOT_RADIUS/4-( ((-flipV+1)/2) * _ROBOT_RADIUS/2),0.03);

				robot_text (dp);
				glPopMatrix();
			}
		}

		/* Debug Point */

		for (int dp=0; dp<NDEBUG_POINTS; dp++)
		{
			if ( robot_info[i].Debug_point_visible[dp] && robot_info[i].visible )
			{
				glPushMatrix();
				qglColor(robot_info[i].color);
				glTranslated(robot_info[i].Debug_point[dp].x, robot_info[i].Debug_point[dp].y, 0.03);		//ZZ changed by Joao
	    		glRotated(45.0, 0.0, 0.0, 1.0);
				glCallList(debug_point);
				glPopMatrix();
			}
		}






		/* Obstacles Points */
		if ( robot_info[i].obstacleVisible && (robot_info[i].nObst > 0) && robot_info[i].visible )
		{
			for (unsigned int obs=0; obs < robot_info[i].nObst; obs++)
			{
				if ( (robot_info[i].obstacleWidth[obs] < 0.8) && (robot_info[i].obstacleWidth[obs] > 0.05) )//Only obstacles < 0.8m e > 5 cm
				{
					// Obstacle Text
					if (robot_info[i].obstacleteamMate[obs] >= 0)
					{
		
					glPushMatrix();
					qglColor(FieldWhite);
		
					glRasterPos3f(robot_info[i].obstacleAbsCenter[obs].x- _ROBOT_RADIUS/6+( ((-flipH+1)/2) * _ROBOT_RADIUS/2.5), robot_info[i].obstacleAbsCenter[obs].y + _ROBOT_RADIUS/4-( ((-flipV+1)/2) * _ROBOT_RADIUS/2),0.01);		//ZZ changed by Joao
					robot_text (robot_info[i].obstacleteamMate[obs]+1);
					glPopMatrix();
					}

					//Draw Points
					glPushMatrix();
					qglColor(robot_info[i].color);
					glTranslated(robot_info[i].obstacleAbsCenter[obs].x, robot_info[i].obstacleAbsCenter[obs].y, 0.01);		//ZZ changed by Joao
					glScaled(robot_info[i].obstacleWidth[obs]*100, robot_info[i].obstacleWidth[obs]*100, 1);
					glCallList(obstacle_point);
					glPopMatrix();

					


				}
			}
		}


		if (DB_Info->Robot_status[i] == STATUS_OK)
		{
			if (robot_info[i].gState == freePlay)
				countFreePlay++;
			else
				countOther++;
		}


	}


	//DRAW VELOCITY OF THE CLOSEST ROBOT
	glPushMatrix();
	if ( robot_info[closerToBall].velBall.length() > 0.70 )
		qglColor(robot_info[closerToBall].color);
	else
		qglColor("red");
	glTranslated(robot_info[closerToBall].PosBall.x, robot_info[closerToBall].PosBall.y, 0.02);
	glRotated(robot_info[closerToBall].dirBall, 0.0, 0.0, 1.0);
	glScaled(robot_info[closerToBall].velBall.length(), 1.0, 1.0);
	glCallList(ballVel);
	glPopMatrix();


	/* Visualization of gamestate, roles and passes*/
//	stopRobot, freePlay

//	preOwnKickOff, postOwnKickOff, preOwnGoalKick, postOwnGoalKick, preOwnCornerKick, postOwnCornerKick, preOwnThrowIn, postOwnThrowIn, preOwnFreeKick, postOwnFreeKick, preOwnPenalty, postOwnPenalty,

//	dropBall, parking, errorState

//	preOpponentKickOff,	postOpponentKickOff, preOpponentGoalKick, postOpponentGoalKick, preOpponentCornerKick, postOpponentCornerKick, preOpponentThrowIn, postOpponentThrowIn, preOpponentFreeKick, postOpponentFreeKick, preOpponentPenalty, postOpponentPenalty,


/*AVAILABLE QCOLOR: white, black, cyan, darkCyan, red, darkRed, magenta, darkMagenta, green, darkGreen, yellow, darkYellow, blue, darkBlue, gray, darkGray, lightGray*/

	/*Check for active roles during freePlay*/
	if (countFreePlay > countOther)
	{
		for (unsigned i=0; i<NROBOTS; i++)
		{
			/* If the robot has any role that is not supposed to be active during Freeplay, outline it in red*/
			if ( DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role != rGoalie && DB_Info->Robot_info[i].role != rStriker && DB_Info->Robot_info[i].role != rMidfielder )
			{
				glPushMatrix();
				qglColor("red");
				glTranslated(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.0);
				double angle=robot_info[i].angle;
				glRotated(angle, 0.0, 0.0, 1.0);
				glCallList(robotContour);
				glPopMatrix();
			}
		}
	}
	else
	{
		vector<int> replacers;
		vector<int> barriers;
		/*check which robots are replacer or barrier*/
		for (unsigned i=0; i<NROBOTS; i++)
		{
			if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rReplacer)
			{
				replacers.push_back(i);
				glPushMatrix();
				qglColor("white");
				glTranslated(robot_info[i].Pos.x, robot_info[i].Pos.y, 0.0);
				double angle=robot_info[i].angle;
				glRotated(angle, 0.0, 0.0, 1.0);
				glCallList(robotContour);
				glPopMatrix();
			}

			if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rBarrier)
				barriers.push_back(i);
		}


		for (unsigned i=0; i<NROBOTS; i++)
		{
			/*Check Own setplay states*/
			if (	robot_info[i].gState == preOwnKickOff ||
					robot_info[i].gState == preOwnGoalKick ||
					robot_info[i].gState == preOwnCornerKick ||
					robot_info[i].gState == preOwnThrowIn ||
					robot_info[i].gState == preOwnFreeKick/* ||
					robot_info[i].gState == preOwnPenalty*/)
			{

				/*During preOwn state, draw dotted lines to the replacer(s) with free indication (also indicate receiver priority?)*/
				if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rReceiver)
				{
					for (unsigned int rep=0; rep<replacers.size(); rep++)
					{
						glPushMatrix();
						qglColor(FieldBlack);
						glRasterPos3f(robot_info[i].coordinationVec.x- _ROBOT_RADIUS/6+( ((-flipH+1)/2) * _ROBOT_RADIUS/2.5), robot_info[i].coordinationVec.y+ _ROBOT_RADIUS/4-( ((-flipV+1)/2) * _ROBOT_RADIUS/2),0.0);
						robot_text (i+1);
						glPopMatrix();

						glPushMatrix();
						if (robot_info[i].coordinationFlag[0] == (int)LineClear)
						{
							dashedLine(robot_info[i].coordinationVec.x, robot_info[i].coordinationVec.y, robot_info[replacers.at(rep)].PosBall.x, robot_info[replacers.at(rep)].PosBall.y, "green");
						}
						else
						{
							dashedLine(robot_info[i].coordinationVec.x, robot_info[i].coordinationVec.y, robot_info[replacers.at(rep)].PosBall.x, robot_info[replacers.at(rep)].PosBall.y, "red");
						}
						glPopMatrix();
					}
				}
			}
			else if (	robot_info[i].gState == postOwnKickOff ||
						robot_info[i].gState == postOwnGoalKick ||
						robot_info[i].gState == postOwnCornerKick ||
						robot_info[i].gState == postOwnThrowIn ||
						robot_info[i].gState == postOwnFreeKick/* ||
						robot_info[i].gState == postOwnPenalty*/)
			{
				/*During postOwn, draw filled line to the receiver chosen by the replacer*/
				if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rReplacer)
				{
					int receiverIdx = (robot_info[i].coordinationFlag[0]-TryingToPass0);
					if (receiverIdx >=0 && receiverIdx <=5)
					{
						line(robot_info[i].PosBall.x, robot_info[i].PosBall.y, robot_info[receiverIdx].coordinationVec.x, robot_info[receiverIdx].coordinationVec.y, "black");
					}
				}
			}

		}
	}



	/* Field */
	//glLoadIdentity();
	glPushMatrix();
	glTranslated(0.0, 0.0, 0.0);
	glCallList(field);
	glPopMatrix();
}

//==================================================== resizeGL ==========================================

void FieldWidget::resizeGL(int width, int height)
{
	makeCurrent();

	//Adaptar o campo á imagem
	factorH=0; 
	factorV=0;

	double aux=0;

		//para uma determinada altura calcula-se o valor correspondente da largura (em pixeis)
		aux = height*(_FIELD_HEIGHT+2)/(_FIELD_WIDTH+2);

		//se o valor da largura for maior então tem que se acrescentar o valor correspondente ao campo (em metros)
		if (aux < width)
			factorH=(width-aux)*(_FIELD_HEIGHT+2)/aux;
		//se não, é a altura que tem que variar
		else
		{
			aux = width*(_FIELD_WIDTH+2)/(_FIELD_HEIGHT+2);
			factorV=(height-aux)*(_FIELD_WIDTH+2)/aux;
		}

	

glViewport(0, 0, width, height);


    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    

	glOrtho(flipH*(-(_FIELD_HEIGHT/2)-1.5-factorH/2),flipH*((_FIELD_HEIGHT/2)+1.5+factorH/2), flipV*((_FIELD_WIDTH/2)+1.5+factorV/2),flipV*( -(_FIELD_WIDTH/2)-1.5-factorV/2), -0.1, 0.1);
    glMatrixMode(GL_MODELVIEW);

	//printf("%f %f %f %f\n",flipH*(-(_FIELD_HEIGHT/2)-1-factorH/2),flipH*((_FIELD_HEIGHT/2)+1+factorH/2), flipV*((_FIELD_WIDTH/2)+1+factorV/2),flipV*( -(_FIELD_WIDTH/2)-1-factorV/2));
    

	glLoadIdentity();
	updateField();

}

//==================================================== makeField ==========================================

GLuint FieldWidget::makeField()
{
	makeCurrent();

    GLuint list = glGenLists(1);
    glNewList(list, GL_COMPILE);

	/* Pontos no campo */
	//baixo direita
	circ (	_FIELD_HEIGHT/2-_BLACK_POINT_LENGTH, 
		_FIELD_WIDTH/2-_BLACK_POINT_WIDTH, 
		_LINE_THICKNESS/2, FieldBlack);
	//baixo centro
	circ (	0, 
		_FIELD_WIDTH/2-_BLACK_POINT_WIDTH, 
		_LINE_THICKNESS/2, FieldBlack);
	//baixo esquerda
	circ (	-(_FIELD_HEIGHT/2-_BLACK_POINT_LENGTH), 
		_FIELD_WIDTH/2-_BLACK_POINT_WIDTH, 
		_LINE_THICKNESS/2, FieldBlack);
	//cima direita
	circ (	_FIELD_HEIGHT/2-_BLACK_POINT_LENGTH, 
		-(_FIELD_WIDTH/2-_BLACK_POINT_WIDTH), 
		_LINE_THICKNESS/2, FieldBlack);
	//cima centro
	circ (	0, 
		-(_FIELD_WIDTH/2-_BLACK_POINT_WIDTH), 
		_LINE_THICKNESS/2, FieldBlack);
	//cima esquerda
	circ (	-(_FIELD_HEIGHT/2-_BLACK_POINT_LENGTH), 
		-(_FIELD_WIDTH/2-_BLACK_POINT_WIDTH), 
		_LINE_THICKNESS/2, FieldBlack);


	/* Linhas horizontais */
	{GLdouble x_lfundo = _LINE_THICKNESS/2;
	GLdouble y_lfundo = _FIELD_WIDTH/2;
	//Fundo direito
	quad(	x_lfundo+(_FIELD_HEIGHT/2), y_lfundo, 
		-x_lfundo+(_FIELD_HEIGHT/2), y_lfundo,
		-x_lfundo+(_FIELD_HEIGHT/2), -y_lfundo, 
		x_lfundo+(_FIELD_HEIGHT/2), -y_lfundo,
		FieldWhite);
	//Fundo esquerdo
	quad(	x_lfundo-(_FIELD_HEIGHT/2), y_lfundo, 
		-x_lfundo-(_FIELD_HEIGHT/2), y_lfundo,
		-x_lfundo-(_FIELD_HEIGHT/2), -y_lfundo, 
		x_lfundo-(_FIELD_HEIGHT/2), -y_lfundo,
		FieldWhite);
	//central
	quad(	x_lfundo, y_lfundo, 
		-x_lfundo, y_lfundo, 
		-x_lfundo, -y_lfundo, 
		x_lfundo, -y_lfundo,
		FieldWhite);}


	/* Linhas laterais */
	{GLdouble x_llateral = _FIELD_HEIGHT/2+_LINE_THICKNESS/2;
	 GLdouble y_llateral = _LINE_THICKNESS/2;
	//cima
	quad(	x_llateral, y_llateral+(_FIELD_WIDTH/2), 
		-x_llateral, y_llateral+(_FIELD_WIDTH/2), 
		-x_llateral, -y_llateral+(_FIELD_WIDTH/2), 
		x_llateral, -y_llateral+(_FIELD_WIDTH/2),
		FieldWhite);
	//baixo
	quad(	x_llateral, y_llateral-(_FIELD_WIDTH/2), 
		-x_llateral, y_llateral-(_FIELD_WIDTH/2), 
		-x_llateral, -y_llateral-(_FIELD_WIDTH/2), 
		x_llateral, -y_llateral-(_FIELD_WIDTH/2),
		FieldWhite);}


	/* Linhas da pequena área */
	{GLdouble x_GareaV = _LINE_THICKNESS/2;
	 GLdouble y_GareaV = _GOAL_AREA_WIDTH/2;
	//esquerda
	quad(	x_GareaV-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), y_GareaV, 
		-x_GareaV-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), y_GareaV, 
		-x_GareaV-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), -y_GareaV,
		x_GareaV-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), -y_GareaV,
		FieldWhite);
	//direita
	quad(	x_GareaV+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), y_GareaV, 
		-x_GareaV+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), y_GareaV,
		-x_GareaV+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), -y_GareaV,
		x_GareaV+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH), -y_GareaV,
		FieldWhite);}

	{GLdouble x_GareaH = _GOAL_AREA_LENGTH/2+_LINE_THICKNESS/2;
	GLdouble y_GareaH = _LINE_THICKNESS/2;
	//esquerda cima
	quad(	x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH+(_GOAL_AREA_WIDTH/2), 
		-x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH+(_GOAL_AREA_WIDTH/2), 
		-x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH+(_GOAL_AREA_WIDTH/2),
		x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH+(_GOAL_AREA_WIDTH/2),
		FieldWhite);
	//direita cima
	quad(	x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH+(_GOAL_AREA_WIDTH/2), 
		-x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH+(_GOAL_AREA_WIDTH/2), 
		-x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH+(_GOAL_AREA_WIDTH/2),
		x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH+(_GOAL_AREA_WIDTH/2),
		FieldWhite);
	//esuqerda baixo
	quad(	x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH-(_GOAL_AREA_WIDTH/2), 
		-x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH-(_GOAL_AREA_WIDTH/2), 
		-x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH-(_GOAL_AREA_WIDTH/2),
		x_GareaH-(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH-(_GOAL_AREA_WIDTH/2),
		FieldWhite);
	//direita baixo
	quad(	x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH-(_GOAL_AREA_WIDTH/2), 
		-x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		y_GareaH-(_GOAL_AREA_WIDTH/2), 
		-x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH-(_GOAL_AREA_WIDTH/2),
		x_GareaH+(_FIELD_HEIGHT/2-_GOAL_AREA_LENGTH/2),
		-y_GareaH-(_GOAL_AREA_WIDTH/2),
		FieldWhite);}



	/* Linhas da grande área */
	{GLdouble x_PareaV = _LINE_THICKNESS/2;
	 GLdouble y_PareaV = _PENALTY_AREA_WIDTH/2;
	//esquerda
	quad(	x_PareaV-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), y_PareaV, 
		-x_PareaV-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), y_PareaV, 
		-x_PareaV-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), -y_PareaV,
		x_PareaV-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), -y_PareaV,
		FieldWhite);
	//direita
	quad(	x_PareaV+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), y_PareaV, 
		-x_PareaV+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), y_PareaV,
		-x_PareaV+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), -y_PareaV,
		x_PareaV+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH), -y_PareaV,
		FieldWhite);}

	{GLdouble x_PareaH = _PENALTY_AREA_LENGTH/2+_LINE_THICKNESS/2;
	 GLdouble y_PareaH = _LINE_THICKNESS/2;
	//esquerda cima
	quad(	x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH+(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH+(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH+(_PENALTY_AREA_WIDTH/2),
		x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH+(_PENALTY_AREA_WIDTH/2),
		FieldWhite);
	//direita cima
	quad(	x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH+(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH+(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH+(_PENALTY_AREA_WIDTH/2),
		x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH+(_PENALTY_AREA_WIDTH/2),
		FieldWhite);
	//esquerda baixo
	quad(	x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH-(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH-(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH-(_PENALTY_AREA_WIDTH/2),
		x_PareaH-(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH-(_PENALTY_AREA_WIDTH/2),
		FieldWhite);
	//direita baixo
	quad(	x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH-(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		y_PareaH-(_PENALTY_AREA_WIDTH/2), 
		-x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH-(_PENALTY_AREA_WIDTH/2),
		x_PareaH+(_FIELD_HEIGHT/2-_PENALTY_AREA_LENGTH/2),
		-y_PareaH-(_PENALTY_AREA_WIDTH/2),
		FieldWhite);}

	

	/* Ponto central do campo*/
	circ (0,0,_BALL_DIAMETER/4, FieldWhite);


	/* circulo central */
	circ (0,0, _CENTER_CIRCLE_RADIUS-_LINE_THICKNESS/2, FieldGreen);
	circ (0,0, _CENTER_CIRCLE_RADIUS+_LINE_THICKNESS/2, FieldWhite);

	

	/* circulos dos cantos */
	//esquerdo em cima
	for (int i = 0; i < NumSectors; ++i) 
	{
	double angle1 = (i * 2* Pi) / NumSectors;
		if (angle1 < Pi/2)
		{
		GLdouble x_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle1);
		GLdouble y_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle1);
		GLdouble x_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle1);
		GLdouble y_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle1);
		
		double angle2 = ((i + 1) * 2 * Pi) / NumSectors;
		GLdouble x_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle2);
        	GLdouble x_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle2);
	
        	quad(	x_cent1-(_FIELD_HEIGHT/2), y_cent1-(_FIELD_WIDTH/2), 
			x_cent2-(_FIELD_HEIGHT/2), y_cent2-(_FIELD_WIDTH/2), 
			x_cent3-(_FIELD_HEIGHT/2), y_cent3-(_FIELD_WIDTH/2), 
			x_cent4-(_FIELD_HEIGHT/2), y_cent4-(_FIELD_WIDTH/2),
			FieldWhite);
		}
	}


	//esquerdo em baixo
    	for (int i = 0; i < NumSectors; ++i) 
	{
      	double angle1 = (i * 2* Pi) / NumSectors;
	if ((angle1 > Pi/2) && (angle1 < Pi))
		{
       		GLdouble x_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle1);
       		GLdouble y_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle1);
        	GLdouble x_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle1);
       		GLdouble y_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle1);
		
       		double angle2 = ((i + 1) * 2 * Pi) / NumSectors;
       		GLdouble x_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle2);
        	GLdouble x_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle2);
	
        	quad(	x_cent1-(_FIELD_HEIGHT/2), y_cent1+(_FIELD_WIDTH/2), 
			x_cent2-(_FIELD_HEIGHT/2), y_cent2+(_FIELD_WIDTH/2), 
			x_cent3-(_FIELD_HEIGHT/2), y_cent3+(_FIELD_WIDTH/2), 
			x_cent4-(_FIELD_HEIGHT/2), y_cent4+(_FIELD_WIDTH/2),
			FieldWhite);
		}
	}
		

	//direito em baixo
    	for (int i = 0; i < NumSectors; ++i) 
	{
      	double angle1 = (i * 2* Pi) / NumSectors;
	if ( (angle1 > Pi) && (angle1 < (3*Pi/2)) )
	{
       		GLdouble x_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle1);
       		GLdouble y_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle1);
        	GLdouble x_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle1);
       		GLdouble y_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle1);
		
       		double angle2 = ((i + 1) * 2 * Pi) / NumSectors;
       		GLdouble x_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle2);
        	GLdouble x_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle2);
	
        	quad(	x_cent1+(_FIELD_HEIGHT/2), y_cent1+(_FIELD_WIDTH/2), 
			x_cent2+(_FIELD_HEIGHT/2), y_cent2+(_FIELD_WIDTH/2), 
			x_cent3+(_FIELD_HEIGHT/2), y_cent3+(_FIELD_WIDTH/2), 
			x_cent4+(_FIELD_HEIGHT/2), y_cent4+(_FIELD_WIDTH/2),
			FieldWhite);
		}
	}
			

	//direito em cima
    	for (int i = 0; i < NumSectors; ++i) 
	{
      	double angle1 = (i * 2* Pi) / NumSectors;
	if (angle1 > (3*Pi/2))
		{
       		GLdouble x_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle1);
       		GLdouble y_cent1 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle1);
        	GLdouble x_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle1);
       		GLdouble y_cent2 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle1);
		
       		double angle2 = ((i + 1) * 2 * Pi) / NumSectors;
       		GLdouble x_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent3 = (_CORNER_CIRCLE_RADIUS - _LINE_THICKNESS/2) * cos(angle2);
        	GLdouble x_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * sin(angle2);
       		GLdouble y_cent4 = (_CORNER_CIRCLE_RADIUS + _LINE_THICKNESS/2) * cos(angle2);
	
        	quad(	x_cent1+(_FIELD_HEIGHT/2), y_cent1-(_FIELD_WIDTH/2), 
			x_cent2+(_FIELD_HEIGHT/2), y_cent2-(_FIELD_WIDTH/2), 
			x_cent3+(_FIELD_HEIGHT/2), y_cent3-(_FIELD_WIDTH/2), 
			x_cent4+(_FIELD_HEIGHT/2), y_cent4-(_FIELD_WIDTH/2),
			FieldWhite);
				}
			}

		/* Marcas de Penalty */
		//esquerda
		circ (-(_FIELD_HEIGHT/2-_PENALTY_MARK_DISTANCE), 0, _LINE_THICKNESS/2, FieldWhite);
		//direita
		circ ((_FIELD_HEIGHT/2-_PENALTY_MARK_DISTANCE), 0, _LINE_THICKNESS/2, FieldWhite);


		/* seta */
		quad(	1, (_LINE_THICKNESS/2 - _FIELD_WIDTH/2 - 0.5), 
			-1, (_LINE_THICKNESS/2 - _FIELD_WIDTH/2 - 0.5), 
			-1, (-_LINE_THICKNESS/2 - _FIELD_WIDTH/2 - 0.5),
			1, (-_LINE_THICKNESS/2 - _FIELD_WIDTH/2 - 0.5),
			FieldWhite);

		quad(	(_LINE_THICKNESS/2+1 - _LINE_THICKNESS*2), 	(0.2- _FIELD_WIDTH/2 - 0.5), 
			(-_LINE_THICKNESS/2+1 - _LINE_THICKNESS*2), 	(0.2- _FIELD_WIDTH/2 - 0.5), 
			(-_LINE_THICKNESS/2+1) , 			(-0- _FIELD_WIDTH/2 - 0.5),
			(_LINE_THICKNESS/2+1), 				(-0- _FIELD_WIDTH/2 - 0.5),
			FieldWhite);
		

		quad(	(_LINE_THICKNESS/2+1), 				(0- _FIELD_WIDTH/2 - 0.5), 
			(-_LINE_THICKNESS/2+1), 				(0- _FIELD_WIDTH/2 - 0.5), 
			(-_LINE_THICKNESS/2+1 - _LINE_THICKNESS*2), 	(-0.2- _FIELD_WIDTH/2 - 0.5),
			(_LINE_THICKNESS/2+1 - _LINE_THICKNESS*2), 	(-0.2- _FIELD_WIDTH/2 - 0.5),
			FieldWhite);



		/* Balizas */
		//Azul (lado esquerdo)
		quad(	0.5+_FIELD_HEIGHT/2, _LINE_THICKNESS/2 + 1, 
			_FIELD_HEIGHT/2, _LINE_THICKNESS/2 + 1, 
			_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 + 1,
			0.5+_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 +1,
			FieldWhite);

		quad(	0.5+_FIELD_HEIGHT/2, _LINE_THICKNESS/2 - 1, 
			_FIELD_HEIGHT/2, _LINE_THICKNESS/2 - 1, 
			_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 - 1,
			0.5+_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 - 1,
			FieldWhite);

		quad(	0.5+_FIELD_HEIGHT/2+_LINE_THICKNESS/2, 1+_LINE_THICKNESS/2, 
			0.5+_FIELD_HEIGHT/2-_LINE_THICKNESS/2, 1+_LINE_THICKNESS/2, 
			0.5+_FIELD_HEIGHT/2-_LINE_THICKNESS/2, -1-_LINE_THICKNESS/2,
			0.5+_FIELD_HEIGHT/2+_LINE_THICKNESS/2, -1-_LINE_THICKNESS/2,
			FieldWhite);

			//preenchimento da baliza com cor Azul
			
		quad(	0.25+_FIELD_HEIGHT/2+0.5/2, 1+_LINE_THICKNESS/2, 
			0.25+_FIELD_HEIGHT/2-0.5/2, 1+_LINE_THICKNESS/2, 
			0.25+_FIELD_HEIGHT/2-0.5/2, -1-_LINE_THICKNESS/2,
			0.25+_FIELD_HEIGHT/2+0.5/2, -1-_LINE_THICKNESS/2,
			FieldBlue);
		




		//Amarela(lado direito)
		quad(	-0.5-_FIELD_HEIGHT/2, _LINE_THICKNESS/2 + 1, 
			-_FIELD_HEIGHT/2, _LINE_THICKNESS/2 + 1, 
			-_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 + 1,
			-0.5-_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 + 1,
			FieldWhite);

		quad(	-0.5-_FIELD_HEIGHT/2, _LINE_THICKNESS/2 - 1, 
			-_FIELD_HEIGHT/2, _LINE_THICKNESS/2 - 1, 
			-_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 - 1,
			-0.5-_FIELD_HEIGHT/2, -_LINE_THICKNESS/2 - 1,
			FieldWhite);

		quad(	-0.5-_FIELD_HEIGHT/2+_LINE_THICKNESS/2, 1+_LINE_THICKNESS/2, 
			-0.5-_FIELD_HEIGHT/2-_LINE_THICKNESS/2, 1+_LINE_THICKNESS/2, 
			-0.5-_FIELD_HEIGHT/2-_LINE_THICKNESS/2, -1-_LINE_THICKNESS/2,
			-0.5-_FIELD_HEIGHT/2+_LINE_THICKNESS/2, -1-_LINE_THICKNESS/2,
			FieldWhite);

			//preenchimento da baliza com cor Amarela

		quad(	-0.25-_FIELD_HEIGHT/2+0.5/2, 1+_LINE_THICKNESS/2, 
			-0.25-_FIELD_HEIGHT/2-0.5/2, 1+_LINE_THICKNESS/2, 
			-0.25-_FIELD_HEIGHT/2-0.5/2, -1-_LINE_THICKNESS/2,
			-0.25-_FIELD_HEIGHT/2+0.5/2, -1-_LINE_THICKNESS/2,
			FieldYellow);



    glEndList();
    return list;
}

//==================================================== makeRobot ==========================================

GLuint FieldWidget::makeRobot()
{
	makeCurrent();

	GLuint list1 = glGenLists(1);
	glNewList(list1, GL_COMPILE);

using namespace std;


	/* Criação do circulo do corpo do robot */
	Vec centro_r;
	centro_r.x=0;
	centro_r.y=0;
	double raio_r=_ROBOT_RADIUS;
	Circle circulo_robot(centro_r, raio_r);

	/* Criação do circulo da frente */
	Vec centro_b;
	centro_b.x=0;
	centro_b.y=_ROBOT_RADIUS;
	double raio_b=_BALL_DIAMETER/2;
	Circle circulo_bola(centro_b, raio_b);

	/* Vector de pontos de intesecção entre a recta e os circulos*/
	vector<Vec> vec_pontos_bola;
	vector<Vec> vec_pontos_robot;

glBegin(GL_TRIANGLE_FAN);
	//Ponto do centro
	glVertex2d(centro_r.x, centro_r.y);

	for (int i = 0; i < NumSectors+1; ++i)
	{
		double angle1 = (i * 2 * Pi) / NumSectors;

		//Ponto 2 da recta
		Vec p2;
		p2.x = _ROBOT_RADIUS/2*sin(angle1);
		p2.y = _ROBOT_RADIUS/2*cos(angle1);

		//Ao ponto 1 soma-se uma pequena componente para minimizar
		//alguns efeitos indesejados no desenho
		Vec p1;
		p1=centro_r;

		if(angle1<Pi/2)	p1.x+=_ROBOT_RADIUS/4;
		if(angle1>3*Pi/2)p1.x-=_ROBOT_RADIUS/4;

		//criação da linha que passa nos pontos 1 e 2
		Line linha(p1, p2);

		//intersecção entre os pontos e os circulos
		vec_pontos_bola = intersect (linha, circulo_bola);
		vec_pontos_robot= intersect (linha, circulo_robot);

		// em busca do ponto correcto
		Vec p, p_aux;
		p.x=p_aux.x=0;
		p.y=p_aux.x=0;
		unsigned count_points=0;
			if ( (angle1 <= Pi/2) || (angle1 > 3*Pi/2))
			{
				/* procura o y positivo no circulo grande*/
				for (unsigned j=0; j< vec_pontos_robot.size(); j++)
					if ( vec_pontos_robot[j].y > 0 )
							p = vec_pontos_robot[j];

				if(vec_pontos_bola.size() > 0)
				{
					/* Se o valor da bola for menor trocam os valores */
					for (unsigned j=0; j< vec_pontos_bola.size(); j++)
						if ( vec_pontos_bola[j].y < p.y )
						{
							p_aux = vec_pontos_bola[j];
							count_points++;
						}
						p=p_aux;
				}
			}

			else //se não o ponto é o do circulo grande
				for (unsigned j=0; j< vec_pontos_robot.size(); j++)
				{
					if ( vec_pontos_robot[j].y < 0 )
							p = vec_pontos_robot[j];

				}
		//outro ponto
		glVertex2d(p.x, p.y);
	}


glEnd();









	glEndList();
	return list1;
}


//==================================================== makeRobotContour ==========================================
GLuint FieldWidget::makeRobotContour()
{
	makeCurrent();

	GLuint list1 = glGenLists(1);
	glNewList(list1, GL_COMPILE);

	using namespace std;
//Contorno do robot
#if 1
	GLfloat size;
	GLfloat increment;

	glGetFloatv(GL_POINT_SIZE_RANGE,&size);
	glGetFloatv(GL_POINT_SIZE_GRANULARITY,&increment);

	//printf("size %f inc %f\n", size, increment);

	glPointSize(size+1*increment);

	glEnable(GL_POINT_SMOOTH);

	glBegin(GL_POINTS);

/* Criação do circulo do corpo do robot */
	Vec centro_r;
	centro_r.x=0;
	centro_r.y=0;
	double raio_r=_ROBOT_RADIUS;
	Circle circulo_robot(centro_r, raio_r);

	/* Criação do circulo da frente */
	Vec centro_b;
	centro_b.x=0;
	centro_b.y=_ROBOT_RADIUS;
	double raio_b=_BALL_DIAMETER/2;
	Circle circulo_bola(centro_b, raio_b);

	/* Vector de pontos de intesecção entre a recta e os circulos*/
	vector<Vec> vec_pontos_bola;
	vector<Vec> vec_pontos_robot;

	glBegin(GL_TRIANGLE_FAN);
	//Ponto do centro
	glVertex2d(centro_r.x, centro_r.y);

	for (int i = 0; i < NumSectors+1; ++i)
	{
		double angle1 = (i * 2 * Pi) / NumSectors;

		//Ponto 2 da recta
		Vec p2;
		p2.x = _ROBOT_RADIUS/2*sin(angle1);
		p2.y = _ROBOT_RADIUS/2*cos(angle1);

		//Ao ponto 1 soma-se uma pequena componente para minimizar
		//alguns efeitos indesejados no desenho
		Vec p1;
		p1=centro_r;

		if(angle1<Pi/2)	p1.x+=_ROBOT_RADIUS/4;
		if(angle1>3*Pi/2)p1.x-=_ROBOT_RADIUS/4;

		//criação da linha que passa nos pontos 1 e 2
		Line linha(p1, p2);

		//intersecção entre os pontos e os circulos
		vec_pontos_bola = intersect (linha, circulo_bola);
		vec_pontos_robot= intersect (linha, circulo_robot);

		// em busca do ponto correcto
		Vec p, p_aux;
		p.x=p_aux.x=0;
		p.y=p_aux.x=0;
		unsigned count_points=0;
			if ( (angle1 <= Pi/2) || (angle1 > 3*Pi/2))
			{
				/* procura o y positivo no circulo grande*/
				for (unsigned j=0; j< vec_pontos_robot.size(); j++)
					if ( vec_pontos_robot[j].y > 0 )
							p = vec_pontos_robot[j];

				if(vec_pontos_bola.size() > 0)
				{
					/* Se o valor da bola for menor trocam os valores */
					for (unsigned j=0; j< vec_pontos_bola.size(); j++)
						if ( vec_pontos_bola[j].y < p.y )
						{
							p_aux = vec_pontos_bola[j];
							count_points++;
						}
						p=p_aux;
				}
			}

			else //se não o ponto é o do circulo grande
				for (unsigned j=0; j< vec_pontos_robot.size(); j++)
				{
					if ( vec_pontos_robot[j].y < 0 )
							p = vec_pontos_robot[j];

				}
		//outro ponto
		glVertex2d(p.x, p.y);
	}


	glEnd();
	glDisable(GL_POINT_SMOOTH);
#endif

	glEndList();
	return list1;
}

//==================================================== makeBall ==========================================

GLuint FieldWidget::makeBall()
{
	makeCurrent();
	//a bola é um circulo normal mas sem cor
    GLuint list1 = glGenLists(1);
    glNewList(list1, GL_COMPILE);

	glBegin(GL_TRIANGLE_FAN);

	glVertex2d(0, 0);

	for (int i = 0; i < NumSectors+1; ++i) 
	{
		double angle1 = (i * 2 * Pi) / NumSectors;
		GLdouble x = _BALL_DIAMETER/2 * sin(angle1);
		GLdouble y = _BALL_DIAMETER/2 * cos(angle1);

		glVertex2d(x, y);
	}


glEnd();


    glEndList();
    return list1;
}

//==================================================== makeDebugPoint ==========================================

GLuint FieldWidget::makeDebugPoint()
{

	makeCurrent();
	//o Debug_point é um circulo normal mas sem cor
    GLuint list1 = glGenLists(1);
    glNewList(list1, GL_COMPILE);



glBegin(GL_QUADS);

    glVertex2d((0.15), (-(0.05)));
    glVertex2d((-(0.15)), (-(0.05)));
    glVertex2d((-(0.15)), (0.05));
    glVertex2d((0.15), (0.05));

glEnd();

glBegin(GL_QUADS);

    glVertex2d((0.05), (-(0.15)));
    glVertex2d((-(0.05)), (-(0.15)));
    glVertex2d((-(0.05)), (0.15));
    glVertex2d((0.05), (0.15));

glEnd();

/*
	glBegin(GL_TRIANGLE_FAN);

	glVertex2d(0, 0);

	for (int i = 0; i < NumSectors+1; ++i) 
	{
		double angle1 = (i * 2 * Pi) / NumSectors;
		GLdouble x = _BALL_DIAMETER/3 * sin(angle1);
		GLdouble y = _BALL_DIAMETER/3 * cos(angle1);

		glVertex2d(x, y);
	}


glEnd();
*/

    glEndList();
    return list1;


}




//==================================================== makeBallEngaged ==========================================

GLuint FieldWidget::makeBallEngaged()
{
	makeCurrent();

    GLuint list1 = glGenLists(1);
    glNewList(list1, GL_COMPILE);




GLfloat size;
GLfloat increment;
glGetFloatv(GL_POINT_SIZE_RANGE,&size);
glGetFloatv(GL_POINT_SIZE_GRANULARITY,&increment);
glPointSize(size+2*increment);


	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);

	for (int i = 0; i < NumSectors+1; ++i) 
	{
		double angle1 = (i * 2 * Pi) / NumSectors;
		{
		GLdouble x = _BALL_DIAMETER/2 * sin(angle1);
		GLdouble y = _BALL_DIAMETER/2 * cos(angle1);

		glVertex2d(x, (y+_ROBOT_RADIUS));
		}
	}
	glEnd();
	glDisable(GL_POINT_SMOOTH);

    glEndList();
    return list1;
}

//==================================================== makeSelect ==========================================

GLuint FieldWidget::makeSelect()
{
	makeCurrent();

	/* desenha um quadrado sem cor á volta do robot*/
    GLuint list1 = glGenLists(1);
    glNewList(list1, GL_COMPILE);

glBegin(GL_LINE_LOOP);

    glVertex2d((_ROBOT_RADIUS+_ROBOT_RADIUS/2), (-(_ROBOT_RADIUS+_ROBOT_RADIUS/2)));
    glVertex2d((-(_ROBOT_RADIUS+_ROBOT_RADIUS/2)), (-(_ROBOT_RADIUS+_ROBOT_RADIUS/2)));
    glVertex2d((-(_ROBOT_RADIUS+_ROBOT_RADIUS/2)), (_ROBOT_RADIUS+_ROBOT_RADIUS/2));
    glVertex2d((_ROBOT_RADIUS+_ROBOT_RADIUS/2), (_ROBOT_RADIUS+_ROBOT_RADIUS/2));

glEnd();
    glEndList();
    return list1;
}

//==================================================== makeRobotAngle ==========================================

GLuint FieldWidget::makeRobotAngle()
{
	makeCurrent();

	/* desenha um quadrado sem cor á volta do robot*/
    GLuint list1 = glGenLists(1);
    glNewList(list1, GL_COMPILE);
/*
GLfloat size;
GLfloat increment;
glGetFloatv(GL_POINT_SIZE_RANGE,&size);
glGetFloatv(GL_POINT_SIZE_GRANULARITY,&increment);
glPointSize(size+2*increment);


glEnable(GL_POINT_SMOOTH);
*/
glBegin(GL_LINES);

	glVertex2d(0,-_ROBOT_RADIUS);
	glVertex2d(0,_ROBOT_RADIUS-_BALL_DIAMETER/2);

glEnd();

    glEndList();

    return list1;
}

//==================================================== makeObstaclePoint ==========================================

GLuint FieldWidget::makeObstaclePoint()
{
	makeCurrent();

	/* desenha um quadrado da cor do robot*/
    GLuint list1 = glGenLists(1);
    glNewList(list1, GL_COMPILE);


GLfloat size;
GLfloat increment;
glGetFloatv(GL_POINT_SIZE_RANGE,&size);
glGetFloatv(GL_POINT_SIZE_GRANULARITY,&increment);
glPointSize(size+3*increment);


	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);

	glVertex2d(0, 0);

	for (int i = 0; i < NumSectors+1; ++i) 
	{
		double angle1 = (i * 2 * Pi) / NumSectors;
		GLdouble x = 0.005 * sin(angle1);
		GLdouble y = 0.005 * cos(angle1);

		glVertex2d(x, y);
	}


glEnd();






/*
glBegin(GL_QUADS);

    glVertex2d((0.005), (-(0.005)));
    glVertex2d((-(0.005)), (-(0.005)));
    glVertex2d((-(0.005)), (0.005));
    glVertex2d((0.005), (0.005));

glEnd();
*/
    glEndList();

    return list1;
}

//==================================================== makeBallVel ==========================================	added by Joao

GLuint FieldWidget::makeBallVel()
{
	makeCurrent();

	GLuint list1 = glGenLists(1);
	glNewList(list1, GL_COMPILE);

	glBegin(GL_LINES);
		glVertex2d(0.0,0.0);
		glVertex2d(1.0,0.0);
	glEnd();

	glEndList();

	return list1;
}

//==================================================== quad ==========================================

void FieldWidget::quad(	GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2,
                    	GLdouble x3, GLdouble y3, GLdouble x4, GLdouble y4, 
			QColor color)
{
/* Função para desenhar quadrados através dos pontos dos vértices */

glBegin(GL_QUADS);

    qglColor(color);

    glVertex2d(x4, y4);
    glVertex2d(x3, y3);
    glVertex2d(x2, y2);
    glVertex2d(x1, y1);

glEnd();
}

//==================================================== circ ==========================================

void FieldWidget::circ(	GLdouble xcent, GLdouble ycent, 
			GLdouble radius, QColor color)
{
	/* Função para desenhar um circulo através do ponto central e do raio*/

glBegin(GL_TRIANGLE_FAN);

	qglColor(color);

	glVertex2d(xcent, ycent);

	for (int i = 0; i < NumSectors+1; ++i) 
	{
		double angle1 = (i * 2 * Pi) / NumSectors;
		GLdouble x = radius * sin(angle1);
		GLdouble y = radius * cos(angle1);

		glVertex2d((x+xcent), (y+ycent));
	}



glEnd();
}

//==================================================== line ==========================================
void FieldWidget::line(	GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2,
			QColor color)
{
	/* Função para desenhar uma linha */
	glLineWidth(3.0);
	glBegin(GL_LINES);
		glPointSize(100);
		qglColor(color);
		glVertex2d(x2, y2);
		glVertex2d(x1, y1);
	glEnd();
	glLineWidth(1.0);
}

//==================================================== dashedLine ==========================================
void FieldWidget::dashedLine(	GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2,
			QColor color)
{
	/* Função para desenhar uma linha tracejada*/
	int nDashes = 12;
	Vec pi = Vec((float)x1, (float)y1);
	Vec pf = Vec((float)x2, (float)y2);
	Vec lineVector = pf-pi;
	float size = lineVector.length();
	float increment = size/nDashes;

	for (int i=0; i<nDashes; i+=2)
	{
		Vec p1 = pi+lineVector.setLength(i * increment);
		Vec p2 = pi+lineVector.setLength((i+1) * increment);
		line(p1.x, p1.y, p2.x, p2.y, color);
	}
}


//==================================================== mousePressEvent ==========================================
void FieldWidget::mousePressEvent(QMouseEvent *event)
{
//O mecanismo de selecção funciona da seguinte maneira:
//	-Se o ponto onde clicamos corresponde a uma determinada área em volta
//do robot respectivo então este fica seleccionado.
//	-Para seleccionar o robot actualiza-se a variável IsAnySelected e 
//o campo selected dentro da informação do robot => 1
//	-Para dar uma ordem de deslocação ao robot é necessário voltar a clicar no
//campo, usando a variável IsAnySelected consegue-se distinguir estas duas acções 
//(seleccionar e movimentar)
//	-A flag IsAnySelected vai ser usada também para a função mouseMoveEvent pois permite
//identificar se estamos a arrastar uma zona válida

	makeCurrent();

	/* Aquisição das coordenadas (em pixeis) */
	Vec selpos;
	selpos.x=double(event->x());	
	selpos.y=double(event->y());

	int height = QWidget::height();
	int width = QWidget::width();

	/* Transformar pixeis em metros */
	selpos.x=((selpos.x-(width/2))*((_FIELD_HEIGHT/2)+1+factorH/2)/(width/2))*flipH;
	selpos.y=((selpos.y-(height/2))*((_FIELD_WIDTH/2)+1+factorV/2)/(height/2))*flipV;

	if (selpos.x > (_FIELD_HEIGHT/2)+1) selpos.x = (_FIELD_HEIGHT/2)+1;
	if (selpos.x < -(_FIELD_HEIGHT/2+1)) selpos.x = -((_FIELD_HEIGHT/2)+1);
	if (selpos.y > (_FIELD_WIDTH/2)+1) selpos.y = (_FIELD_WIDTH/2)+1;
	if (selpos.y < -(_FIELD_WIDTH/2+1)) selpos.y = -(_FIELD_WIDTH/2+1);


	/* Se o robot já tiver sido seleccionado */
	if(IsAnySelected)
		for (unsigned i=0; i<NROBOTS; i++)
		{ 
			if(robot_info[i].selected) 
			{
				move_robot(i,selpos);
				IsAnySelected = 0;
				robot_info[i].selected=0;
				break;//o mais prioritário é o robot com menor nº
			}
		}


	else //se não vamos ver se onde clicamos é um robot
	{
		IsAnySelected=0;
		for (unsigned i=0; i<NROBOTS; i++)
		{
			// Deselecciona todos os robots
			for (unsigned j=0; j<NROBOTS; j++) robot_info[j].selected=0;
		
			//Selecciona o primeiro dentro dos limites impostos
			if ( 	( selpos.x < robot_info[i].Pos.x+_ROBOT_RADIUS) &&
				( selpos.x > robot_info[i].Pos.x-_ROBOT_RADIUS) &&
				( selpos.y < robot_info[i].Pos.y+_ROBOT_RADIUS) &&
				( selpos.y > robot_info[i].Pos.y-_ROBOT_RADIUS) &&
				(robot_info[i].visible == 1) )

				{	
				IsAnySelected=1;
				robot_info[i].selected=1;
				Sel_Pos=robot_info[i].Pos;
				break;
				}

		}

	}
	//actualiza a janela
	updateGL();
}

//==================================================== mouseMoveEvent ==========================================

void FieldWidget::mouseMoveEvent(QMouseEvent *event)
{
	makeCurrent();

	/* Passos identicos aos anteriores */
	Vec selpos;

	selpos.x=double(event->x());	
	selpos.y=double(event->y());

	int height = QWidget::height();
	int width = QWidget::width();

	selpos.x=((selpos.x-(width/2))*((_FIELD_HEIGHT/2)+1+factorH/2)/(width/2))*flipH;
	selpos.y=((selpos.y-(height/2))*((_FIELD_WIDTH/2)+1+factorV/2)/(height/2))*flipV;

	if (selpos.x > (_FIELD_HEIGHT/2)+1) selpos.x = (_FIELD_HEIGHT/2)+1;
	if (selpos.x < -(_FIELD_HEIGHT/2+1)) selpos.x = -((_FIELD_HEIGHT/2)+1);
	if (selpos.y > (_FIELD_WIDTH/2)+1) selpos.y = (_FIELD_WIDTH/2)+1;
	if (selpos.y < -(_FIELD_WIDTH/2+1)) selpos.y = -(_FIELD_WIDTH/2+1);

	/* Se algum tiver sido seleccionado vamos mover o robot para o sítio certo*/
	if(IsAnySelected)
		for (unsigned i=0; i<NROBOTS; i++)
		{ 
			if(robot_info[i].selected) 
			{
				move_robot(i,selpos);
				break;
			}
		}

	updateGL();
}

//==================================================== keyPressEvent ==========================================

void FieldWidget::keyPressEvent(QKeyEvent *event)
{
	makeCurrent();

	/* Pelo sim pelo não mais vale ter uma tecla para deseleccionar todos os robots */

	if (event->key () == Qt::Key_Escape) 
	{
		for (unsigned j=0; j<NROBOTS; j++) robot_info[j].selected=0;
		IsAnySelected=0;
		updateField();
	}

/*
	if (event->key () == Qt::Key_A) 
	{
		flip_horizontal ();
	}

	if (event->key () == Qt::Key_D) 
	{
		flip_horizontal ();
	}
	
	if (event->key () == Qt::Key_W) 
	{
		flip_vertical ();
	}

	if (event->key () == Qt::Key_S) 
	{
		flip_vertical ();
	}
*/

	if (event->key () == Qt::Key_R) 
	{
		updateField ();
	}
	
	if (event->key () == Qt::Key_U) 
	{
		update_robot_info();
	}

}


//==================================================== move_robot ==========================================

void FieldWidget::move_robot(unsigned robotnum, Vec pos)
{

robotnum = 3;
	/* Função de movimentação*/

	//NOTA:no futuro esta função vai representar uma ordem para o robot ir para o ponto pos
	// Pode eventualmente ser tranformada num sinal


	Sel_Pos = pos;
	//robot_info[robotnum].Pos=pos;

}

//==================================================== robot_text ==========================================

void FieldWidget::robot_text (unsigned robotnum)
{

	makeCurrent();

	char c = '0'+robotnum;

	
#if OFFICIAL_FIELD
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c);
#else
	glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
#endif



}

//==================================================== updateField ==========================================

void FieldWidget::updateField(void)
{

//	makeCurrent();
//	int height = QWidget::height();
//	int width = QWidget::width();
//	resizeGL(width, height);

		updateGL();
		paintGL();
}



//==================================================== flip ==========================================
void FieldWidget::flip(void)
{
	makeCurrent();
	int height = QWidget::height();
	int width = QWidget::width();

	flipV *= -1;
	flipH *= -1;

	resizeGL(width, height);
	updateField();
}


//==================================================== update_robot_info ==========================================

void FieldWidget::update_robot_info(void)
{

//Na actualização da informação tem que se ter em atenção que o referencial do campo não é o que se está a ver, o factor de conversão é o seguinte:
// x = -y e y = -x

	if ( DB_Info != NULL )
	{
		for (unsigned i=0; i<NROBOTS;i++) //mudar para o número de robots no .h
		{
			Robot nowRobot = DB_Info->Robot_info[i];

			robot_info[i].Pos.x = -nowRobot.pos.y;
			robot_info[i].Pos.y = -nowRobot.pos.x;

			robot_info[i].angle = -(int)(nowRobot.orientation*180/Pi)+90;
			
			if (DB_Info->Robot_status[i] == STATUS_OK || DB_Info->Robot_status[i] == STATUS_SB)
				robot_info[i].visible = 1;
			else
				robot_info[i].visible = 0;

			robot_info[i].PosBall.x = -nowRobot.ball.pos.y;
			robot_info[i].PosBall.y = -nowRobot.ball.pos.x;

			robot_info[i].ballengaged = nowRobot.ball.engaged;
			robot_info[i].ballvisible = nowRobot.ball.visible;
			robot_info[i].velBall.x = -nowRobot.ball.vel.y;	//added by Joao
			robot_info[i].velBall.y = -nowRobot.ball.vel.x;	//added by Joao

			/* Debug Point */
			for (unsigned int dp=0; dp < NDEBUG_POINTS; dp++)
				{
					robot_info[i].Debug_point[dp].x = -nowRobot.debugPoints[dp].y;
					robot_info[i].Debug_point[dp].y = -nowRobot.debugPoints[dp].x;
				}

			/* Obstacles */
			robot_info[i].nObst = nowRobot.nObst;
			
// FIXME fprintf(stderr,"agent :%d nObst: %d\n", i, robot_info[i].nObst);

//			if ((robot_info[i].nObst > 0) && (robot_info[i].nObst <= 12))
//			{
				for (unsigned int obj=0; obj < robot_info[i].nObst; obj++)
				{
					ObstacleInfo nowObs = nowRobot.obstacles[obj];

					robot_info[i].obstacleAbsCenter[obj].x = -nowObs.absCenter.y;
					robot_info[i].obstacleAbsCenter[obj].y = -nowObs.absCenter.x;
					robot_info[i].obstacleWidth[obj] = 0.5; //TODO DB_Info->Robot_info[i].obstacles[obj].width;
					robot_info[i].obstacleteamMate[obj] = nowObs.id;
				}
//			}
//fprintf(stderr,"total obstacles: %d",robot_info[i].nObst);

			/* Stuck */
			robot_info[i].stuck = nowRobot.stuck;

			/* Flag for detection of opponent dribbling */
			robot_info[i].opponentDribbling = nowRobot.opponentDribbling;

			robot_info[i].coordinationFlag[0] = nowRobot.coordinationFlag[0];
			robot_info[i].coordinationFlag[1] = nowRobot.coordinationFlag[1];
			robot_info[i].coordinationVec.x = -nowRobot.coordinationVec.y;
			robot_info[i].coordinationVec.y = -nowRobot.coordinationVec.x;
			robot_info[i].gState = nowRobot.currentGameState;

			/* Flip com a cor */
			if (DB_Info->Robot_info[i].goalColor == Yellow)
			{
				robot_info[i].Pos.x = -robot_info[i].Pos.x;
				robot_info[i].Pos.y = -robot_info[i].Pos.y;
				robot_info[i].angle = robot_info[i].angle+180 ;

				robot_info[i].PosBall.x = -robot_info[i].PosBall.x;
				robot_info[i].PosBall.y = -robot_info[i].PosBall.y;
				robot_info[i].velBall.x = -robot_info[i].velBall.x;	//added by Joao
				robot_info[i].velBall.y = -robot_info[i].velBall.y;	//added by Joao
			
				for (unsigned int obj=0; obj < robot_info[i].nObst; obj++)
				{
					robot_info[i].obstacleAbsCenter[obj].x = -robot_info[i].obstacleAbsCenter[obj].x;
					robot_info[i].obstacleAbsCenter[obj].y = -robot_info[i].obstacleAbsCenter[obj].y;
				}
				
				for (unsigned int dp=0; dp < NDEBUG_POINTS; dp++)
				{
					robot_info[i].Debug_point[dp].x = -robot_info[i].Debug_point[dp].x;
					robot_info[i].Debug_point[dp].y = -robot_info[i].Debug_point[dp].y;
				}

				robot_info[i].coordinationVec.x = -robot_info[i].coordinationVec.x;
				robot_info[i].coordinationVec.y = -robot_info[i].coordinationVec.y;
			}

			robot_info[i].dirBall = robot_info[i].velBall.angle().get_deg_180();	//added by Joao (calculate angle only after having the correct goal colour coordinates

			// If Robot Status = SB force Position on field
/*			if(db_coach_info != NULL)
			{
				if( (db_coach_info->GamePart == 1) || (db_coach_info->GamePart == 2))
				{
					if (DB_Info->Robot_status[i]==STATUS_SB) 
					{
						robot_info[i].Pos.x = -_FIELD_HEIGHT/2-0.6-_ROBOT_RADIUS;
						robot_info[i].Pos.y = -((double)(i)*2.1*_ROBOT_RADIUS)+(2.5*2.1*_ROBOT_RADIUS);
						robot_info[i].angle = -90;
					}
				}
			}*/


		}	

	}
	updateField();
	//printf("3 - %f %f\n", robot_info[2].Pos.x, robot_info[2].Pos.y);
}

void FieldWidget::get_info_pointer( DB_Robot_Info * rw)
{
	DB_Info = rw;
}

void FieldWidget::get_coach_pointer( DB_Coach_Info * ci)
{
	db_coach_info = ci;
}

void FieldWidget::debug_point_flip (void)
{
	if (Debug_point_visible) Debug_point_visible =0;
		else Debug_point_visible=1;

}

void FieldWidget::obstacles_point_flip (unsigned int Robot_no, bool on_off)
{
	if (Robot_no < NROBOTS)
		robot_info[Robot_no].obstacleVisible=on_off;

}

void FieldWidget::obstacles_point_flip_all (bool on_off)
{
	for(unsigned int i=0; i<NROBOTS ; i++)
		obstacles_point_flip (i, on_off);
}

void FieldWidget::debug_point_flip (unsigned int Robot_no, bool on_off)
{
	if (Robot_no < NROBOTS)
	for(int i=0;i<NDEBUG_POINTS;i++)
		robot_info[Robot_no].Debug_point_visible[i]=on_off;

}

void FieldWidget::debug_point_flip_all (bool on_off)
{
	for(unsigned int i=0; i<NROBOTS ; i++)
		debug_point_flip (i, on_off);
}
