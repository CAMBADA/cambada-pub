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

#ifndef FIELDWIDGET3D_H
#define FIELDWIDGET3D_H

#include <QTimer>
#include <QFile>

#include <QVTKWidget.h>

#include <vtkCallbackCommand.h>
#include <vtkSmartPointer.h>
#include <vtkLineSource.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextSource.h>
#include <vtkCylinderSource.h>
#include <vtkVectorText.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkOBJReader.h>
#include <vtkTriangleStrip.h>
#include <vtkCellArray.h>
#include <vtkDataSetMapper.h>
#include <vtkFloatArray.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTextWidget.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkViewport.h>
#include <vtkPointWidget.h>
#include <vtkPolygon.h>
#include <vtkPNGReader.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageData.h>
#include <vtkImageMapper.h>
#include <vtkPropPicker.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPropCollection.h>
#include <vtkDelaunay2D.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkPointData.h>

#include <QVTKInteractor.h>

#include "DB_Robot_info.h"
#include "GridView.h"
#include "Robot.h"
#include "Vec.h"

#define OBSTACLE_HEIGHT 0.2

class FieldWidget3D : public QVTKWidget
{
    Q_OBJECT
public:
    explicit FieldWidget3D(QWidget *parent = 0);
    void get_info_pointer( DB_Robot_Info * rw);
    void get_coach_pointer( DB_Coach_Info * ci);

    vtkRenderWindow *renderWindow;
    vtkRenderer *renderer;
    vtkCamera* camera;

    vtkActor* field;
    vtkActor* robots[6];
    vtkActor* robotNum[6];
    vtkActor* balls[6];
    vtkActor* taxiLine;
    vtkLineSource* taxiSource;

    DB_Robot_Info *DB_Info;
    DB_Coach_Info *db_coach_info;

    bool taxiFollow;
    bool heightVisible;
    bool heightColor;
    bool height3D;
    bool top;
    bool lockCam;
    void setTop(bool top);


private:
    float _FIELD_LENGTH;
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


    vtkSmartPointer<vtkActor> createLine(float x1, float y1, float z1, float x2, float y2, float z2);
    void addArc(vtkRenderer* renderer, float x, float y, float radius, float startDeg, float endDeg);
    void drawField(vtkRenderer* renderer);
    void drawGoals(vtkRenderer* renderer);
    void initBalls(vtkRenderer* renderer);
    void initGridView();
    void updateGridView();
    void deleteGridView();

    vtkActor* createText(QString text);
    vtkActor* createObstacle();
    vtkActor* createDebugPt();
    vtkActor* createDashedLine(float x1, float y1, float z1, float x2, float y2, float z2);
    void createDot(vtkRenderer* renderer, float x, float y, bool black, float radius=0.05);



    // Score board
    vtkActor2D* score_board;
    vtkTextActor* score_cambada;
    vtkTextActor* score_other;

    vtkActor* testActor;

    vtkLineSource* velocityLineSrc;
    vtkActor* velocityLine;

    vector<vtkActor*> toDeleteActors;

    vtkPoints* heightPoints;
    vtkPolyData* heightPolyData;
    vtkDelaunay2D* heightDelaunay;
    vtkPolyData* heightPolyDataAfterInterp;
    vtkActor* heightActor;

    float robotsColorR[6];
    float robotsColorG[6];
    float robotsColorB[6];

    // Timer to update objects positions
    QTimer *Update_timer;

    bool option_draw_debug[NROBOTS];
    bool option_draw_obstacles[NROBOTS];

signals:
    
public slots:
    void flip(void);
    void lock(bool);
    void update_robot_info(void);

    void obstacles_point_flip(unsigned int Robot_no, bool on_off);
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

    // Heightmap
    void setHeightMapVisible(bool v){
        heightVisible = v;
    }
    void setHeightMap3D(bool v){
        height3D = v;
    }
    void setHeightMapColor(bool v){
        heightColor = v;
    }

    
};

#endif // FIELDWIDGET3D_H
