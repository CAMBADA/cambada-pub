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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>

#include <geometry.h>
#include "Robot.h"
#include "DB_Robot_info.h"

#define NDEBUG_POINTS 4

class FieldWidget : public QGLWidget
{
    Q_OBJECT

public:
    FieldWidget(QWidget *parent = 0);
    ~FieldWidget();
    void get_info_pointer( DB_Robot_Info * rw);
    void get_coach_pointer( DB_Coach_Info * ci);

	int flipV;
	int flipH;
	bool IsAnySelected;
	bool Debug_point_visible;


private:
	float _FIELD_HEIGHT;
	float _FIELD_WIDTH;
	float _LINE_THICKNESS;
	float _GOAL_AREA_LENGTH;
	float _GOAL_AREA_WIDTH;
	float _PENALTY_AREA_LENGTH;
	float _PENALTY_AREA_WIDTH;
	float _CENTER_CIRCLE_RADIUS;
	float _BALL_DIAMETER;
	float _CORNER_CIRCLE_RADIUS;
	float _PENALTY_MARK_DISTANCE;
	float _BLACK_POINT_WIDTH;
	float _BLACK_POINT_LENGTH;
	float _ROBOT_RADIUS;	

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);

private:
	GLuint makeField();
	GLuint makeRobot();
	GLuint makeBall();
	GLuint makeSelect();
	GLuint makeBallEngaged();
	GLuint makeRobotAngle();
	GLuint makeDebugPoint();
	GLuint makeObstaclePoint();
	GLuint makeBallVel();		//added by Joao
	GLuint makeRobotContour();		//added by Joao

	void quad(	GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2,
              		GLdouble x3, GLdouble y3, GLdouble x4, GLdouble y4, QColor color);
	void circ(	GLdouble xcent, GLdouble ycent, GLdouble radius, QColor color);	
	void line( GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2,	QColor color);		//added by Joao
	void dashedLine( GLdouble x1, GLdouble y1, GLdouble x2, GLdouble y2, QColor color);	//added by Joao
	void move_robot(unsigned robotnum, Vec pos);
	void robot_text (unsigned robotnum);


   	GLuint field;
	GLuint robot;
	GLuint ball;
	GLuint select;
	GLuint ball_engaged;
	GLuint robotAngle;
	GLuint debug_point;
	GLuint obstacle_point;
	GLuint ballVel;		//added by Joao
	GLuint robotContour;	//added by Joao

	QColor FieldGreen;
	QColor FieldWhite;
	QColor FieldBlack;
	QColor FieldBlue;
	QColor FieldYellow;


	double factorH, factorV;

	 double Pi;
    	 int NumSectors;

	struct ROBOT_INFO
	{
		Vec Pos;
		int angle;
		Vec PosBall;	
		Vec velBall;	//added by Joao
		float dirBall;	//added by Joao
		QColor color;
		bool selected;
		bool visible;
		bool ballvisible;
		bool ballengaged;

		Vec Debug_point[4];
		bool Debug_point_visible[4];

		/* Obstacles */
		unsigned int	nObst;
		Vec obstacleAbsCenter[MAX_SHARED_OBSTACLES];
		double obstacleWidth[MAX_SHARED_OBSTACLES];	
		int obstacleteamMate[MAX_SHARED_OBSTACLES];
		bool obstacleVisible;

		char stuck;	
		bool opponentDribbling;

		//added by Joao... Why the hell is not robot info a DB_Robot_info, where everything is available??? The need to increase this structure just for drawing is annoying and VERY error prone...
		int coordinationFlag[2];
		Vec coordinationVec;
		WSGameState gState;
	};

	ROBOT_INFO robot_info[NROBOTS];
	DB_Robot_Info *DB_Info;
	DB_Coach_Info *db_coach_info;

	QTimer *Update_timer;



	Vec Sel_Pos;


public slots:
	void updateField(void);
	void flip(void);
	void update_robot_info(void);
	void debug_point_flip (void);

	//Obstacles
	void obstacles_point_flip (unsigned int Robot_no, bool on_off);
	void obstacles_point_flip_r0 (bool on_off)
		{obstacles_point_flip (0, on_off);}
	void obstacles_point_flip_r1 (bool on_off)
		{obstacles_point_flip (1, on_off);}
	void obstacles_point_flip_r2 (bool on_off)
		{obstacles_point_flip (2, on_off);}
	void obstacles_point_flip_r3 (bool on_off)
		{obstacles_point_flip (3, on_off);}
	void obstacles_point_flip_r4 (bool on_off)
		{obstacles_point_flip (4, on_off);}
	void obstacles_point_flip_r5 (bool on_off)
		{obstacles_point_flip (5, on_off);}
	void obstacles_point_flip_r6 (bool on_off)
		{obstacles_point_flip (6, on_off);}
	void obstacles_point_flip_all (bool on_off);

	//Debug Points
	void debug_point_flip (unsigned int Robot_no, bool on_off);
	void debug_point_flip_r0 (bool on_off)
		{debug_point_flip (0, on_off);}
	void debug_point_flip_r1 (bool on_off)
		{debug_point_flip (1, on_off);}
	void debug_point_flip_r2 (bool on_off)
		{debug_point_flip (2, on_off);}
	void debug_point_flip_r3 (bool on_off)
		{debug_point_flip (3, on_off);}
	void debug_point_flip_r4 (bool on_off)
		{debug_point_flip (4, on_off);}
	void debug_point_flip_r5 (bool on_off)
		{debug_point_flip (5, on_off);}
	void debug_point_flip_r6 (bool on_off)
		{debug_point_flip (6, on_off);}
	void debug_point_flip_all (bool on_off);


   
};

#endif
