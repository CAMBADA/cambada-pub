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

#include "FieldWidget3D.h"
#include <vtkLineWidget.h>

#include "ConfigXML.h"

using namespace cambada;
using namespace cambada::util;

class MouseInteractorStyle : public vtkInteractorStyleTerrain
{
public:
    static MouseInteractorStyle* New();
    vtkTypeMacro(MouseInteractorStyle, vtkInteractorStyleTerrain);

    // Initialize internal variables
    MouseInteractorStyle()
    {
        LastPickedActor = NULL;
        LastPickedProperty = vtkProperty::New();
        robotIdx = -1;
        FieldCollection = vtkPropCollection::New();

    }
    virtual ~MouseInteractorStyle()
    {
        LastPickedProperty->Delete();
        FieldCollection->Delete();
    }

    void setParent(FieldWidget3D* p)
    {
        this->parent = p;
        FieldCollection->AddItem(parent->field);
    }

    // When the left button of the mouse is pressed
    virtual void OnLeftButtonDown()
    {
        // Decide if drag of a robot or field orientation change
        fprintf(stderr,"LEFT MOUSE BUTTON PRESSED\n");
        // Get the position where the mouse pointer was
        int* clickPos = this->GetInteractor()->GetEventPosition();
        // Create a picker with the clickPos information
        vtkSmartPointer<vtkPropPicker>  picker = vtkSmartPointer<vtkPropPicker>::New();
        picker->Pick(clickPos[0], clickPos[1], 0, parent->renderer);
        // Check if the picker returns an actor
        this->LastPickedActor = picker->GetActor();
        if(LastPickedActor != NULL)
        {
            robotIdx = -1;
            for(int i=0;i<NROBOTS;i++)
            {
                if(this->LastPickedActor == parent->robots[i])
                {
                    fprintf(stderr,"ROBOT PICKED = %d\n",i);
                    robotIdx = i;
                    break;
                }
            }

            if(robotIdx < 0)
            {
                // Foward the event as a vtkInteractorStyleTerrain event
                if(!parent->lockCam && !parent->top)
                    vtkInteractorStyleTerrain::OnLeftButtonDown();
            }
            else
            {
                // do something with the robot (drag, etc..)
            }
        }else{
            if(!parent->lockCam && !parent->top)
                vtkInteractorStyleTerrain::OnLeftButtonDown();
        }
    }

    virtual void OnLeftButtonUp()
    {
        fprintf(stderr,"LEFT MOUSE BUTTON RELEASED\n");

        // Get the position where the mouse pointer was
        int* clickPos = this->GetInteractor()->GetEventPosition();
        // Create a picker with the clickPos information
        vtkSmartPointer<vtkPropPicker>  picker = vtkSmartPointer<vtkPropPicker>::New();
        picker->PickProp(clickPos[0], clickPos[1], parent->renderer, FieldCollection);


        //Check if the picker returns an actor
        if(picker->GetActor() != NULL && picker->GetActor() == parent->field)
        {
            double* position = picker->GetPickPosition();
            if(robotIdx>= 0 && robotIdx < NROBOTS)
            {
                Vec pos = Vec(position[0], position[1]);
                parent->db_coach_info->Coach_Info.taxiRobotSN[robotIdx]++;
                parent->db_coach_info->Coach_Info.taxiPos = pos;
            }
            fprintf(stderr, "POS: (%.1lf, %.1lf, %.1lf)\n", position[0], position[1], position[2]);
        }

        robotIdx = -1;

        if(!parent->lockCam && !parent->top)
            vtkInteractorStyleTerrain::OnLeftButtonUp();
    }

    virtual void OnMouseMove()
    {
        // Taxi Follow Test
        if(robotIdx >= 0)
        {
            int* clickPos = this->GetInteractor()->GetEventPosition();
            // Create a picker with the clickPos information
            vtkSmartPointer<vtkPropPicker>  picker = vtkSmartPointer<vtkPropPicker>::New();
            picker->Pick(clickPos[0], clickPos[1], 0, parent->renderer);

            if(picker->GetActor() != NULL && picker->GetActor() == parent->field)
            {

                if(parent->taxiFollow)
                {
                    // If taxiFollow is activated
                    // If the picker returns a position within the field
                    double* position = picker->GetPickPosition();
                    // Pass the new vec to the db_coach_info struct
                    Vec pos = Vec(position[0],position[1]);
                    parent->db_coach_info->Coach_Info.taxiRobotSN[robotIdx]++;
                    parent->db_coach_info->Coach_Info.taxiPos = pos;
                    // Create the vector (actor)
                    //double* startPosition = parent->robots[robotIdx]->GetPosition();


                }
            }
        }
        else
        {
            if(!parent->lockCam && !parent->top)
                vtkInteractorStyleTerrain::OnMouseMove();

            if(parent->camera->GetPosition()[2] < 0.0)
                parent->camera->SetPosition(parent->camera->GetPosition()[0], parent->camera->GetPosition()[1], 0.0);
        }
    }


private:
    int robotIdx;
    double* lastFollowPosition;
    vtkActor    *LastPickedActor; 
    FieldWidget3D* parent;
    vtkProperty *LastPickedProperty;
    vtkPropCollection *FieldCollection;

};
// define the previous class as a new vtk standard
vtkStandardNewMacro(MouseInteractorStyle);

FieldWidget3D::FieldWidget3D(QWidget *parent) :
    QVTKWidget(parent)
{
    ConfigXML config;
    if( config.parse("../config/cambada.conf.xml") == false )
    {
        cerr << "ERROR " << endl;
        exit(1);
    }

    Update_timer = new QTimer();
    connect(Update_timer, SIGNAL(timeout()), this, SLOT(update_robot_info()));

    /* Dimensions */
    _FIELD_LENGTH			= config.getField("field_length")/1000.0;
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

    /* Init Colors */
    float robotsColorR[] = { 244.0/255.0, 1, 1, 188.0/255.0, 201.0/255.0, 115.0/255.0};
    float robotsColorG[] = { 194.0/255.0, 216.0/255.0, 153.0/255.0, 143.0/255.0, 160.0/255.0, 194.0/255.0};
    float robotsColorB[] = { 194.0/255.0, 0, 153.0/255.0, 143.0/255.0, 220.0/255.0, 251.0/255.0};
    for(int i = 0; i < 6; i++)
    {
        this->robotsColorR[i] = robotsColorR[i];
        this->robotsColorG[i] = robotsColorG[i];
        this->robotsColorB[i] = robotsColorB[i];
    }

    renderWindow = vtkRenderWindow::New();
    renderer = vtkRenderer::New();
    renderer->SetBackground(72.0/255.0,72.0/255.0,72.0/255.0);

    renderWindow->AddRenderer(renderer);
    this->SetRenderWindow(renderWindow);

    drawField(renderer);
    drawGoals(renderer);
    initBalls(renderer);

    // Camera properties
    camera = vtkCamera::New();
    camera->SetPosition(_FIELD_WIDTH, 0, 22);
    camera->SetFocalPoint(0, 0, 0);
    camera->SetViewUp(0,0,1);
    renderer->SetActiveCamera(camera);

    // Interactor
    QVTKInteractor* iren = this->GetInteractor();
    vtkSmartPointer<MouseInteractorStyle> intStyle = vtkSmartPointer<MouseInteractorStyle>::New();
    intStyle->setParent(this);
    //vtkSmartPointer<vtkInteractorStyleTerrain> intStyle = vtkSmartPointer<vtkInteractorStyleTerrain>::New();
    iren->SetInteractorStyle(intStyle);
    renderWindow->SetInteractor(iren);
    // WIPFIX renderer->Render();

    /* Read CAMBADA model */
    vtkSmartPointer<vtkOBJReader> readerCbd = vtkSmartPointer<vtkOBJReader>::New();
    readerCbd->SetFileName("../config/3DModels/cambada_base.obj");

    vtkSmartPointer<vtkPolyDataMapper> actorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    actorMapper->SetInput(readerCbd->GetOutput());

    // WIPFIX renderer->Render();

    for(int i = 0; i < NROBOTS; i++)
    {
        option_draw_debug[i] = false;
        option_draw_obstacles[i] = false;

        robots[i] = vtkActor::New();
        //robots[i]->SetOrigin(0.25,0.20,0.0);
        robots[i]->SetMapper(actorMapper);
        robots[i]->GetProperty()->SetRepresentationToSurface();
        robots[i]->GetProperty()->SetColor(robotsColorR[i],robotsColorG[i],robotsColorB[i]);
        renderer->AddActor(robots[i]);
        robots[i]->SetPosition(1000,1000,1000);

        // Text widgets

        // Robot text with number
        char text[2] = {'1' + i, '\0'};

        vtkTextProperty* tp = vtkTextProperty::New();
        tp->BoldOn();
        tp->SetJustificationToCentered();
        tp->SetFontSize(12);

        /*vtkTextActor* ta = vtkTextActor::New();
        ta->SetTextProperty(tp);
        ta->SetInput(text);
        robotNum[i] = vtkActor::New();
        robotNum[i]->SetMapper(ta->GetMapper());
        robotNum[i]->SetScale(0.35);
        robotNum[i]->GetProperty()->SetAmbient(1.0);
        renderer->AddActor(robotNum[i]);*/

        robotNum[i] = vtkActor::New();
        vtkVectorText* txt_robotNum = vtkVectorText::New();
        txt_robotNum->SetText(text);
        vtkPolyDataMapper *txtRobotMapper = vtkPolyDataMapper::New();
        txtRobotMapper->SetInput(txt_robotNum->GetOutput());
        robotNum[i]->SetMapper(txtRobotMapper);
        robotNum[i]->SetScale(0.35);
        robotNum[i]->SetPosition(1000,1000,1000);
        robotNum[i]->GetProperty()->SetColor(0.0,0.0,0.0);
        robotNum[i]->GetProperty()->SetAmbient(1.0);
        robotNum[i]->SetOrientation(0,0,90);

        renderer->AddActor(robotNum[i]);
    }

    velocityLineSrc = vtkLineSource::New();
    vtkSmartPointer<vtkPolyDataMapper> velLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    velLineMapper->SetInput(velocityLineSrc->GetOutput());
    velocityLine = vtkActor::New();
    velocityLine->SetMapper(velLineMapper);
    velocityLine->GetProperty()->SetColor(1.0,0,0);
    velocityLine->GetProperty()->SetLineWidth(2);
    velocityLine->GetProperty()->SetOpacity(0);
    renderer->AddActor(velocityLine);

    taxiSource = vtkLineSource::New();
    vtkSmartPointer<vtkPolyDataMapper> followLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    taxiLine = vtkActor::New();
    taxiSource->SetPoint1(0,0,0);
    taxiSource->SetPoint2(1,1,1);
    followLineMapper->SetInputConnection(taxiSource->GetOutputPort());
    taxiLine->SetMapper(followLineMapper);
    taxiLine->SetPosition(1000,1000,1000);
    renderer->AddActor(taxiLine);
    //renderer->AddLight();

    // Score board
    vtkSmartPointer<vtkPNGReader> scoreBoard = vtkSmartPointer<vtkPNGReader>::New();
    string filename = "../src/tools/basestation/icons/score_board.png";
    //string filename = ":/icons/score_board.png";
    scoreBoard->SetFileName(filename.c_str());
    if(scoreBoard->CanReadFile(filename.c_str()))
    {
        vtkSmartPointer<vtkImageMapper> imageMapper = vtkSmartPointer<vtkImageMapper>::New();
        imageMapper->SetInput(scoreBoard->GetOutput());
        //imageMapper->SetInputConnection(colorImage->GetProducerPort());
        imageMapper->SetColorWindow(255);
        imageMapper->SetColorLevel(127.5);


        score_board = vtkActor2D::New();
        score_board->SetMapper(imageMapper);
        score_board->SetLayerNumber(4);
        renderer->AddActor2D(score_board);

    }else{
        fprintf(stderr,"Could not load score-board\n");
    }

    // Cambada / other score
    score_cambada = vtkTextActor::New();
    score_cambada->SetInput("000");
    score_cambada->GetTextProperty()->SetFontSize ( 24 );
    score_cambada->GetTextProperty()->SetFontFamilyToCourier();
    score_cambada->GetTextProperty()->SetJustificationToCentered();
    score_cambada->GetTextProperty()->SetColor(0,0,0);
    score_cambada->GetTextProperty()->BoldOn();
    score_cambada->SetLayerNumber(5);
    renderer->AddActor2D(score_cambada);

    score_other = vtkTextActor::New();
    score_other->SetInput("000");
    score_other->GetTextProperty()->SetFontSize ( 24 );
    score_other->GetTextProperty()->SetFontFamilyToCourier();
    score_other->GetTextProperty()->SetJustificationToCentered();
    score_other->GetTextProperty()->SetColor(0,0,0);
    score_other->GetTextProperty()->BoldOn();
    score_other->SetLayerNumber(5);
    renderer->AddActor2D(score_other);



    //vtkTextWidget* textCbd = vtkTextWidget::New();
    //textCbd->SetInteractor;

    // View heightmap
    heightVisible = false;
    heightColor = true;
    height3D = false;
    heightActor = NULL;

    lockCam = false;
    top = false;

    Update_timer->start(50);
}

/**
  Actualiza a informação dos objectos
  */
void FieldWidget3D::update_robot_info(void)
{
    if(DB_Info == NULL)
        return;

    // Remove all actors that need to be deleted
    for(unsigned int i = 0; i < toDeleteActors.size(); i++)
    {
        renderer->RemoveActor(toDeleteActors.at(i));
        toDeleteActors.at(i)->Delete();
    }
    toDeleteActors.clear();

    int countFreePlay=0;
    int countOther=0;
    int minDistId = 0;
    float minDist = 2013.11;
    for (unsigned int i=0; i < NROBOTS; i++)
    {
        Robot nowRobot = DB_Info->Robot_info[i];
        char status = DB_Info->Robot_status[i];
        bool robotVisible = false;
        float flipVal = 1.0f;
        if(nowRobot.goalColor == Yellow)
            flipVal = -1.0f;

        /* Set Robot Position */
        robots[i]->SetPosition(flipVal*nowRobot.pos.x, flipVal*nowRobot.pos.y , 0.02);
        robots[i]->SetOrientation(0,0,(nowRobot.orientation/M_PI*180.0) + ((flipVal > 0)?180:0));

        // Robot color
        if(nowRobot.role == rTaxi)
        	robots[i]->GetProperty()->SetColor(1,0,0);
        else
        	robots[i]->GetProperty()->SetColor(robotsColorR[i],robotsColorG[i],robotsColorB[i]);

        /* Set Robot Number Position */
        if(camera->GetPosition()[0] > 0)
        {
            robotNum[i]->SetOrientation(0,0,90);
            robotNum[i]->SetPosition(flipVal*nowRobot.pos.x - 0.2, flipVal*nowRobot.pos.y - 0.2, 0.3);
        }else{
            robotNum[i]->SetOrientation(0,0,-90);
            robotNum[i]->SetPosition(flipVal*nowRobot.pos.x + 0.2, flipVal*nowRobot.pos.y + 0.2, 0.3);
        }

        /* Set Actor Opacity */
        if( status == STATUS_NA || status == STATUS_KO ){
            robots[i]->SetVisibility(0);
            robotNum[i]->SetVisibility(0);
        }else if( status == STATUS_SB ){ // If in stand-by => Semi-transparent
            robots[i]->SetVisibility(1);
            robotNum[i]->SetVisibility(1);
            robots[i]->GetProperty()->SetOpacity(0.4);
            robotNum[i]->GetProperty()->SetOpacity(0.8);
            robotVisible = true;
        }else{
            robots[i]->SetVisibility(1);
            robotNum[i]->SetVisibility(1);
            robots[i]->GetProperty()->SetOpacity(1.0);
            robotNum[i]->GetProperty()->SetOpacity(1.0);
            robotVisible = true;
        }

        if(robotVisible && nowRobot.ball.visible)
        {
            float height = 0.11;
            if(nowRobot.ball.airborne && nowRobot.ball.height > 0.11)
                height = nowRobot.ball.height;

            balls[i]->SetPosition(flipVal*nowRobot.ball.pos.x, flipVal*nowRobot.ball.pos.y, height);

            if(nowRobot.ball.engaged)
            {
                balls[i]->GetProperty()->SetColor(1,0,0);
                balls[i]->SetScale(1.5);
                balls[i]->GetProperty()->SetOpacity(1.0);
            }else{
                balls[i]->GetProperty()->SetColor(robotsColorR[i],robotsColorG[i],robotsColorB[i]);
                balls[i]->SetScale(1.0);

                if(heightVisible)
                    balls[i]->GetProperty()->SetOpacity(1.0); // has to be opaque, due to VTK issues in visualization
                else
                    balls[i]->GetProperty()->SetOpacity(0.8);
            }

            // Find closest robot to the ball -> to draw velocity later
            if(robotVisible && nowRobot.ball.posRel.length() < minDist)
            {
                minDist = nowRobot.ball.posRel.length();
                minDistId = i;
            }
        }else{
            balls[i]->SetPosition(1000, 1000, 1000);
        }



        // Debug Points
        if(robotVisible && option_draw_debug[i])
        {
            for (int dp=0; dp < 4; dp++)
            {
                double xPos = nowRobot.debugPoints[dp].x;
                double yPos = nowRobot.debugPoints[dp].y;

                if(isnan(xPos) || isnan(yPos))
                {
                    xPos = -_FIELD_WIDTH/2;
                    yPos = -_FIELD_LENGTH/2;
                }

                if(isinf(xPos) || isinf(yPos))
                {
                    xPos = _FIELD_WIDTH/2;
                    yPos = _FIELD_LENGTH/2;
                }


                vtkActor* dbgPt = createDebugPt();
                dbgPt->SetPosition(flipVal*xPos, flipVal*yPos, 0.02);
                dbgPt->GetProperty()->SetAmbient(1.0);
                dbgPt->GetProperty()->SetDiffuse(0.0);
                dbgPt->GetProperty()->SetSpecular(0.0);
                dbgPt->GetProperty()->SetColor(robotsColorR[i],robotsColorG[i],robotsColorB[i]);
                renderer->AddActor(dbgPt);
                toDeleteActors.push_back(dbgPt);

                vtkActor* dbgText = createText(QString().sprintf("%d",dp));
                dbgText->GetProperty()->SetColor(1,1,1);
                dbgText->SetScale(0.25);
                dbgText->GetProperty()->SetAmbient(1.0);
                dbgText->GetProperty()->SetColor(1.0,1.0,1.0);

                if(camera->GetPosition()[0] > 0)
                {
                    dbgText->SetOrientation(0,0,90);
                    dbgText->SetPosition(flipVal*(xPos + 0.4), flipVal*(yPos - 0.14), 0.04);
                }else{
                    dbgText->SetOrientation(0,0,-90);
                    dbgText->SetPosition(flipVal*(xPos - 0.4), flipVal*(yPos + 0.14), 0.04);
                }

                renderer->AddActor(dbgText);
                toDeleteActors.push_back(dbgText);
            }
        }

        // Draw obstacles JLS: either because obstacle points are active or if the robot detects an opponent dribbling
		if ( (nowRobot.nObst > 0) && robotVisible )
		{
			for (unsigned int obs=0; obs < nowRobot.nObst; obs++)
			{
				ObstacleInfo nowObs = nowRobot.obstacles[obs];

				if ( option_draw_obstacles[i] || (nowObs.id == nowRobot.opponentDribbling) )
				{
					vtkActor* obstacle = createObstacle();
                    obstacle->SetPosition(flipVal*nowObs.absCenter.x, flipVal*nowObs.absCenter.y, OBSTACLE_HEIGHT/2);

					bool draw = option_draw_obstacles[i];
					bool dribbling = nowObs.id == nowRobot.opponentDribbling;
					// Set color according to the situation
					if ( (!draw && dribbling) || (draw && !dribbling) )
						obstacle->GetProperty()->SetColor(robotsColorR[i],robotsColorG[i],robotsColorB[i]);
					else
						obstacle->GetProperty()->SetColor(0,0,0);

					renderer->AddActor(obstacle);
					toDeleteActors.push_back(obstacle);

					//JLS: DRAW OBSTACLE TEXT
					vtkActor* dbgText = createText(QString().sprintf("%d",nowObs.id));
					dbgText->GetProperty()->SetColor(1,1,1);
					dbgText->SetScale(0.15);
					dbgText->GetProperty()->SetAmbient(1.0);
					dbgText->GetProperty()->SetColor(1.0,1.0,1.0);

					if(camera->GetPosition()[0] > 0)
					{
						dbgText->SetOrientation(0,0,90);
                        dbgText->SetPosition(flipVal*(nowObs.absCenter.x + 0.1), flipVal*(nowObs.absCenter.y - 0.15), 0.3);
					}else{
						dbgText->SetOrientation(0,0,-90);
                        dbgText->SetPosition(flipVal*(nowObs.absCenter.x - 0.1), flipVal*(nowObs.absCenter.y + 0.15), 0.3);
					}

					renderer->AddActor(dbgText);
					toDeleteActors.push_back(dbgText);
				}
			}
		}

        if (DB_Info->Robot_status[i] == STATUS_OK)
        {
            if (nowRobot.currentGameState == freePlay)
                countFreePlay++;
            else
                countOther++;
        }

    }

    // Draw ball velocity from the closest robot
    if(minDistId >= 0 && minDistId < NROBOTS)
    {
        Robot nowRobot = DB_Info->Robot_info[minDistId];
        float flipVal = 1.0f;
        if(nowRobot.goalColor == Yellow)
            flipVal = -1.0f;

        velocityLineSrc->SetPoint1(flipVal*(nowRobot.ball.pos.x), flipVal*nowRobot.ball.pos.y, 0.10);
        velocityLineSrc->SetPoint2(flipVal*(nowRobot.ball.pos.x + nowRobot.ball.vel.x), flipVal*(nowRobot.ball.pos.y + nowRobot.ball.vel.y), 0.10);
        velocityLine->GetProperty()->SetOpacity(1);
    }else{
        velocityLineSrc->SetPoint1(1000, 1100, 0.10);
        velocityLineSrc->SetPoint2(1001, 1101, 0.10);
        velocityLine->GetProperty()->SetOpacity(0);
    }

    // Check for active roles during freePlay
    if (countFreePlay > countOther)
    {
        vector<int> strikers;
        for (unsigned i=0; i<NROBOTS; i++)
        {
            if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rStriker)
                strikers.push_back(i);
        }

        for (unsigned i = 0; i < NROBOTS; i++)
        {
            float flipVal = 1.0f;
            if(DB_Info->Robot_info[i].goalColor == Yellow)
                flipVal = -1.0f;

            // If the robot has any role that is not supposed to be active during Freeplay, outline it in red
            if ( DB_Info->Robot_status[i] == STATUS_OK &&
                 DB_Info->Robot_info[i].role != rGoalie &&
                 DB_Info->Robot_info[i].role != rStriker &&
                 DB_Info->Robot_info[i].role != rMidfielder )
            {
                robots[i]->GetProperty()->SetColor(1,0,0);
            }

            fprintf(stderr,"DBG aki 1  - %d %d\n", (int)(DB_Info->Robot_info[i].role == rMidfielder) , (int)(DB_Info->Robot_status[i] == STATUS_OK));

            if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rMidfielder)
            {
                fprintf(stderr,"DBG aki 2\n");
                for (unsigned int str=0; str<strikers.size(); str++)
                {
                    fprintf(stderr,"DBG aki 3\n");
                    if (DB_Info->Robot_info[i].coordinationFlag[0] == (int)LineClear ||
                            DB_Info->Robot_info[i].coordinationFlag[0] == (int)NotClear)
                    {
                        fprintf(stderr,"DBG aki 4\n");
                        vtkActor* line = createDashedLine(
                                    flipVal*DB_Info->Robot_info[i].coordinationVec.x, flipVal*DB_Info->Robot_info[i].coordinationVec.y, 0.05,
                                    flipVal*DB_Info->Robot_info[strikers.at(str)].pos.x, flipVal*DB_Info->Robot_info[strikers.at(str)].pos.y, 0.05);

                        if (DB_Info->Robot_info[i].coordinationFlag[0] == (int)LineClear)
                        {
                            line->GetProperty()->SetColor(0,0.5,0); // green
                        }
                        else if(DB_Info->Robot_info[i].coordinationFlag[0] == (int)NotClear)
                        {
                            line->GetProperty()->SetColor(1,0,0); // red
                        }

                        renderer->AddActor(line);
                        toDeleteActors.push_back(line);
                    }
                }
            }

            // Draw Striker/Replacer PassLine
            if (DB_Info->Robot_status[i] == STATUS_OK && (DB_Info->Robot_info[i].role == rStriker || DB_Info->Robot_info[i].role == rReplacer))
            {
                if (DB_Info->Robot_info[i].coordinationFlag[0] >= (int)BallPassed0 && DB_Info->Robot_info[i].coordinationFlag[0] <= (int)BallPassed5)
                {
                    // Ignore default line
                    if(DB_Info->Robot_info[i].passLine.p1 != Line::def.p1 && DB_Info->Robot_info[i].passLine.p2 != Line::def.p2 )
                    {
                        vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
                        vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                        vtkActor* lineActor = vtkActor::New();

                        line->SetPoint1(flipVal*DB_Info->Robot_info[i].passLine.p1.x, flipVal*DB_Info->Robot_info[i].passLine.p1.y, 0.05);
                        line->SetPoint2(flipVal*DB_Info->Robot_info[i].passLine.p2.x, flipVal*DB_Info->Robot_info[i].passLine.p2.y, 0.05);
                        lineMapper->SetInputConnection(line->GetOutputPort());
                        lineActor->SetMapper(lineMapper);
                        lineActor->GetProperty()->SetLineWidth(3);
                        lineActor->GetProperty()->SetColor(0,0,0); // black line

                        lineActor->GetProperty()->SetColor(0.3,0.3,1); // red
                        renderer->AddActor(lineActor);
                        toDeleteActors.push_back(lineActor);
                    }
                }
            }
        }
    }
    else
    {
        vector<int> replacers;
        vector<int> barriers;

        // check which robots are replacer or barrier
        for (unsigned i=0; i<NROBOTS; i++)
        {
            if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rReplacer)
            {
                replacers.push_back(i);
                // TODO: Contour white around replacers
            }

            if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rBarrier)
                barriers.push_back(i);
        }


        for (unsigned i=0; i<NROBOTS; i++)
        {
            float flipVal = 1.0f;
            if(DB_Info->Robot_info[i].goalColor == Yellow)
                flipVal = -1.0f;

            // Check Own setplay states
            if (	DB_Info->Robot_info[i].currentGameState == preOwnKickOff ||
                    DB_Info->Robot_info[i].currentGameState == preOwnGoalKick ||
                    DB_Info->Robot_info[i].currentGameState == preOwnCornerKick ||
                    DB_Info->Robot_info[i].currentGameState == preOwnThrowIn ||
                    DB_Info->Robot_info[i].currentGameState == preOwnFreeKick) // || robot_info[i].gState == preOwnPenalty)
            {

                // During preOwn state, draw dotted lines to the replacer(s) with free indication (also indicate receiver priority?)
                if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rReceiver)
                {
                    for (unsigned int rep=0; rep<replacers.size(); rep++)
                    {
                        vtkActor* line = createDashedLine(
                                    flipVal*DB_Info->Robot_info[i].coordinationVec.x, flipVal*DB_Info->Robot_info[i].coordinationVec.y, 0.05,
                                    flipVal*DB_Info->Robot_info[replacers.at(rep)].ball.pos.x, flipVal*DB_Info->Robot_info[replacers.at(rep)].ball.pos.y, 0.05);
                        if (DB_Info->Robot_info[i].coordinationFlag[0] == (int)LineClear)
                        {
                            line->GetProperty()->SetColor(0,0.5,0); // green
                        }
                        else
                        {
                            line->GetProperty()->SetColor(1,0,0); // red
                        }

                        renderer->AddActor(line);
                        toDeleteActors.push_back(line);
                    }
                }
            }
            else if (	DB_Info->Robot_info[i].currentGameState == postOwnKickOff ||
                        DB_Info->Robot_info[i].currentGameState == postOwnGoalKick ||
                        DB_Info->Robot_info[i].currentGameState == postOwnCornerKick ||
                        DB_Info->Robot_info[i].currentGameState == postOwnThrowIn ||
                        DB_Info->Robot_info[i].currentGameState == postOwnFreeKick)// || robot_info[i].gState == postOwnPenalty)
            {
                // During postOwn, draw filled line to the receiver chosen by the replacer
                if (DB_Info->Robot_status[i] == STATUS_OK && DB_Info->Robot_info[i].role == rReplacer)
                {
                    int receiverIdx = (DB_Info->Robot_info[i].coordinationFlag[0]-TryingToPass0);
                    if (receiverIdx >=0 && receiverIdx <=5)
                    {
                        vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
                        vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                        vtkActor* lineActor = vtkActor::New();

                        line->SetPoint1(flipVal*DB_Info->Robot_info[i].ball.pos.x, flipVal*DB_Info->Robot_info[i].ball.pos.y, 0.05);
                        line->SetPoint2(flipVal*DB_Info->Robot_info[receiverIdx].coordinationVec.x, flipVal*DB_Info->Robot_info[receiverIdx].coordinationVec.y, 0.05);
                        lineMapper->SetInputConnection(line->GetOutputPort());
                        lineActor->SetMapper(lineMapper);
                        lineActor->GetProperty()->SetLineWidth(3);
                        lineActor->GetProperty()->SetColor(0,0,0); // black line

                        renderer->AddActor(lineActor);
                        toDeleteActors.push_back(lineActor);
                    }
                }
            }

        }
    }

    updateGridView();

    // Score board update
    int width = renderWindow->GetSize()[0];
    int height = renderWindow->GetSize()[1];
    score_board->SetPosition(width/2 - 124, height - 50);
    QString ourGoals = QString().sprintf("%d%c",db_coach_info->Coach_Info.ourGoals, '\0');
    QString theirGoals = QString().sprintf("%d%c",db_coach_info->Coach_Info.theirGoals,'\0');
    score_cambada->SetInput(ourGoals.toStdString().c_str());
    score_other->SetInput(theirGoals.toStdString().c_str());
    score_cambada->SetPosition(width/2 - 20, height - 35);
    score_other->SetPosition(width/2 + 20, height - 35);
    // Force render frame
    if(!renderWindow->CheckInRenderStatus())
        renderWindow->Render();

    // Actualize DB_Coach_Info if a robot as been selected or if a number of ticks have passed
    /*if(taxiRole == true)
    {
        taxiRoleCounter = 0;
        taxiRole = false;
        for(int i = 0; i < NROBOTS; i++)
            db_coach_info->Coach_Info.taxiRobotSN[i] = taxiRobotSN[i];

        db_coach_info->Coach_Info.taxiPos = taxiPos;
    }
    if(taxiRoleCounter == 10)
    {
        taxiRobotIdx = -1;
        db_coach_info->Coach_Info.taxiRobotIdx = (char)taxiRobotIdx;
    }
    else
    {
        taxiRoleCounter++;
    }*/
}

void FieldWidget3D::flip(void)
{
    if(top)
    {
        camera->SetPosition( (camera->GetPosition()[0] < 0)?0.001:-0.001 , 0 , 25);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp((camera->GetPosition()[0] < 0)?1:-1,0,0);
    }else{
        if(camera->GetPosition()[0] < 0)
            camera->SetPosition(_FIELD_WIDTH, 0, 22);
        else
            camera->SetPosition(-_FIELD_WIDTH, 0, 22);

        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0,0,1);
    }
}

void FieldWidget3D::obstacles_point_flip (unsigned int Robot_no, bool on_off)
{
    if (Robot_no < NROBOTS)
        option_draw_obstacles[Robot_no] = on_off;
}

void FieldWidget3D::obstacles_point_flip_all (bool on_off)
{
    for(unsigned int i=0; i<NROBOTS ; i++)
        obstacles_point_flip (i, on_off);
}

void FieldWidget3D::debug_point_flip (unsigned int Robot_no, bool on_off)
{
    if (Robot_no < NROBOTS)
        option_draw_debug[Robot_no] = on_off;
}

void FieldWidget3D::debug_point_flip_all (bool on_off)
{
    for(unsigned int i=0; i<NROBOTS ; i++)
        debug_point_flip (i, on_off);
}

vtkActor* FieldWidget3D::createDashedLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkActor* lineActor = vtkActor::New();
    line->SetPoint1(x1, y1, z1);
    line->SetPoint2(x2, y2, z2);
    lineMapper->SetInputConnection(line->GetOutputPort());
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetLineWidth(3);
    lineActor->GetProperty()->SetLineStipplePattern(0xf0f0);
    lineActor->GetProperty()->SetLineStippleRepeatFactor(1);
    lineActor->GetProperty()->SetPointSize(1);
    lineActor->GetProperty()->SetLineWidth(3);
    return lineActor;
}

vtkSmartPointer<vtkActor> FieldWidget3D::createLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
    line->SetPoint1(x1, y1, z1);
    line->SetPoint2(x2, y2, z2);
    lineMapper->SetInputConnection(line->GetOutputPort());
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetLineWidth(3);

    return lineActor;
}

void FieldWidget3D::addArc(vtkRenderer *renderer, float x, float y, float radius, float startDeg, float endDeg)
{
    float x1,y1,x2,y2;
    x2 = x + radius*cos(startDeg*M_PI/180);
    y2 = y + radius*sin(startDeg*M_PI/180);
    for(int i = startDeg + 10; i <= endDeg; i+= 10)
    {
        x1 = x + radius*cos(i*M_PI/180);
        y1 = y + radius*sin(i*M_PI/180);
        renderer->AddActor(createLine(x1,y1,0,x2,y2,0));
        x2 = x1;
        y2 = y1;
    }
}

void FieldWidget3D::get_info_pointer( DB_Robot_Info * rw)
{
    DB_Info = rw;
}

void FieldWidget3D::get_coach_pointer( DB_Coach_Info * ci)
{
    db_coach_info = ci;
}

void FieldWidget3D::drawGoals(vtkRenderer* renderer)
{
    // Goals
    vtkSmartPointer<vtkOBJReader> reader = vtkSmartPointer<vtkOBJReader>::New();
    reader->SetFileName("../config/3DModels/goal.obj");
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> goalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    goalMapper->SetInput(reader->GetOutput());

    vtkSmartPointer<vtkActor> goalBlue = vtkSmartPointer<vtkActor>::New();
    goalBlue->SetMapper(goalMapper);
    goalBlue->RotateX(90);
    goalBlue->SetPosition(0,-_FIELD_LENGTH/2,0);
    goalBlue->GetProperty()->SetColor(0.2,0.2,1);
    goalBlue->GetProperty()->SetDiffuse(0.4);
    goalBlue->GetProperty()->SetAmbient(0.8);
    renderer->AddActor(goalBlue);

    vtkSmartPointer<vtkActor> goalYellow = vtkSmartPointer<vtkActor>::New();
    goalYellow->SetMapper(goalMapper);
    goalYellow->RotateX(90);
    goalYellow->SetPosition(0,_FIELD_LENGTH/2,0);
    goalYellow->GetProperty()->SetColor(1,1,0.2);
    goalYellow->GetProperty()->SetDiffuse(0.4);
    goalYellow->GetProperty()->SetAmbient(0.8);
    renderer->AddActor(goalYellow);
}

void FieldWidget3D::drawField(vtkRenderer* renderer)
{
    // Draw plane
    vtkSmartPointer<vtkPlaneSource> planeSrc = vtkSmartPointer<vtkPlaneSource>::New();
    planeSrc->SetOrigin(0,0,0);
    planeSrc->SetPoint1(+_FIELD_WIDTH + 2.0,0,0);
    planeSrc->SetPoint2(0,_FIELD_LENGTH + 2.0,0);
    vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    planeMapper->SetInput(planeSrc->GetOutput());
    this->field = vtkActor::New();
    this->field->SetMapper(planeMapper);
    this->field->GetProperty()->SetColor(0.278,0.64,0.196);
    this->field->SetPosition(-_FIELD_WIDTH/2 - 1.0, -_FIELD_LENGTH/2 - 1.0, -0.02);
    this->field->GetProperty()->SetAmbient(1);
    this->field->GetProperty()->SetDiffuse(0);
    this->field->GetProperty()->SetSpecular(0);
    renderer->AddActor(field);

    // Draw Field
    renderer->AddActor(createLine(-_FIELD_WIDTH/2, 0.0, 0.0, _FIELD_WIDTH/2, 0.0, 0.0));
    renderer->AddActor(createLine(-_FIELD_WIDTH/2, -_FIELD_LENGTH/2, 0.0, -_FIELD_WIDTH/2, _FIELD_LENGTH/2, 0.0));
    renderer->AddActor(createLine(_FIELD_WIDTH/2, -_FIELD_LENGTH/2, 0.0, _FIELD_WIDTH/2, _FIELD_LENGTH/2, 0.0));
    renderer->AddActor(createLine(-_FIELD_WIDTH/2, _FIELD_LENGTH/2, 0.0, _FIELD_WIDTH/2, _FIELD_LENGTH/2, 0.0));
    renderer->AddActor(createLine(-_FIELD_WIDTH/2, -_FIELD_LENGTH/2, 0.0, _FIELD_WIDTH/2, -_FIELD_LENGTH/2, 0.0));

    // Goal Areas
    renderer->AddActor(createLine(-_GOAL_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _GOAL_AREA_LENGTH, 0.0, _GOAL_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _GOAL_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(-_GOAL_AREA_WIDTH/2, -_FIELD_LENGTH/2, 0.0, -_GOAL_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _GOAL_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(_GOAL_AREA_WIDTH/2, -_FIELD_LENGTH/2, 0.0, _GOAL_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _GOAL_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(-_GOAL_AREA_WIDTH/2, _FIELD_LENGTH/2 - _GOAL_AREA_LENGTH, 0.0, _GOAL_AREA_WIDTH/2, _FIELD_LENGTH/2 - _GOAL_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(-_GOAL_AREA_WIDTH/2, _FIELD_LENGTH/2, 0.0, -_GOAL_AREA_WIDTH/2, _FIELD_LENGTH/2 - _GOAL_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(_GOAL_AREA_WIDTH/2, _FIELD_LENGTH/2, 0.0, _GOAL_AREA_WIDTH/2, _FIELD_LENGTH/2 - _GOAL_AREA_LENGTH, 0.0));

    // Penalty Areas
    renderer->AddActor(createLine(-_PENALTY_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _PENALTY_AREA_LENGTH, 0.0,
                                  _PENALTY_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _PENALTY_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(-_PENALTY_AREA_WIDTH/2, -_FIELD_LENGTH/2, 0.0,
                                  -_PENALTY_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _PENALTY_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(_PENALTY_AREA_WIDTH/2, -_FIELD_LENGTH/2, 0.0,
                                  _PENALTY_AREA_WIDTH/2, -_FIELD_LENGTH/2 + _PENALTY_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(-_PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2 - _PENALTY_AREA_LENGTH, 0.0,
                                  _PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2 - _PENALTY_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(-_PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2, 0.0,
                                  -_PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2 - _PENALTY_AREA_LENGTH, 0.0));
    renderer->AddActor(createLine(_PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2, 0.0,
                                  _PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2 - _PENALTY_AREA_LENGTH, 0.0));

    // Corner Circles
    addArc(renderer, _FIELD_WIDTH/2, _FIELD_LENGTH/2, _CORNER_CIRCLE_RADIUS, 180, 270);
    addArc(renderer, -_FIELD_WIDTH/2, _FIELD_LENGTH/2, _CORNER_CIRCLE_RADIUS, 270, 360);
    addArc(renderer, -_FIELD_WIDTH/2, -_FIELD_LENGTH/2, _CORNER_CIRCLE_RADIUS, 0, 90);
    addArc(renderer, _FIELD_WIDTH/2, -_FIELD_LENGTH/2, _CORNER_CIRCLE_RADIUS, 90, 180);

    // Center Circle
    addArc(renderer, 0, 0, _CENTER_CIRCLE_RADIUS, 0, 360);

    // Black Dots
    createDot(renderer, _FIELD_WIDTH/4, 0, true);
    createDot(renderer, -_FIELD_WIDTH/4, 0, true);

    createDot(renderer, _FIELD_WIDTH/4, _FIELD_LENGTH/2 - _PENALTY_MARK_DISTANCE, true);
    createDot(renderer, -_FIELD_WIDTH/4, _FIELD_LENGTH/2 - _PENALTY_MARK_DISTANCE, true);
    createDot(renderer, 0, _FIELD_LENGTH/2 - _PENALTY_MARK_DISTANCE, false);

    createDot(renderer, _FIELD_WIDTH/4, -_FIELD_LENGTH/2 + _PENALTY_MARK_DISTANCE, true);
    createDot(renderer, -_FIELD_WIDTH/4, -_FIELD_LENGTH/2 + _PENALTY_MARK_DISTANCE, true);
    createDot(renderer, 0, -_FIELD_LENGTH/2 + _PENALTY_MARK_DISTANCE, false);

    createDot(renderer, 0, 0, false, 0.1);

    renderer->AddActor(createLine(_PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2, 0.0,
                                  _PENALTY_AREA_WIDTH/2, _FIELD_LENGTH/2 - _PENALTY_AREA_LENGTH, 0.0));
}

void FieldWidget3D::createDot(vtkRenderer* renderer, float x, float y, bool black, float radius)
{
    vtkSmartPointer<vtkCylinderSource> dot = vtkSmartPointer<vtkCylinderSource>::New();
    dot->SetRadius(radius);
    dot->SetHeight(0.001);
    dot->SetResolution(32);
    vtkSmartPointer<vtkPolyDataMapper> dotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    dotMapper->SetInput(dot->GetOutput());

    vtkSmartPointer<vtkActor> blackDot1 = vtkSmartPointer<vtkActor>::New();
    blackDot1->SetMapper(dotMapper);

    if(black)
        blackDot1->GetProperty()->SetColor(0,0,0);
    else
        blackDot1->GetProperty()->SetColor(1,1,1);

    blackDot1->SetPosition(x , y , 0.01);
    blackDot1->SetOrientation(90,0,0);
    blackDot1->GetProperty()->SetAmbient(1.0);
    renderer->AddActor(blackDot1);
}


void FieldWidget3D::initBalls(vtkRenderer* renderer)
{
    vtkSmartPointer<vtkSphereSource> sphereSrc = vtkSmartPointer<vtkSphereSource>::New();
    sphereSrc->SetRadius(0.11);
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInput(sphereSrc->GetOutput());
    for(int i = 0; i < NROBOTS; i++)
    {
        balls[i] = vtkActor::New();
        balls[i]->SetMapper(sphereMapper);
        balls[i]->GetProperty()->SetRepresentationToSurface();
        balls[i]->GetProperty()->SetColor(robotsColorR[i],robotsColorG[i],robotsColorB[i]);
        balls[i]->SetPosition(1000,1000,1000);
        renderer->AddActor(balls[i]);
    }
}

vtkActor* FieldWidget3D::createText(QString text){
    vtkActor* actor = vtkActor::New();
    vtkSmartPointer<vtkVectorText> txt = vtkSmartPointer<vtkVectorText>::New();
    txt->SetText(text.toStdString().c_str());
    vtkSmartPointer<vtkPolyDataMapper> txtRobotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    txtRobotMapper->SetInput(txt->GetOutput());
    actor->SetMapper(txtRobotMapper);
    actor->GetProperty()->SetColor(0.0,0.0,0.0);
    actor->GetProperty()->SetAmbient(1.0);
    actor->SetOrientation(0,0,90);
    return actor;
}

vtkActor* FieldWidget3D::createObstacle(){
    // Obstacle actors
    vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetRadius(0.25);
    cylinder->SetHeight(OBSTACLE_HEIGHT);
    cylinder->SetResolution(12);
    vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    cylinderMapper->SetInput(cylinder->GetOutput());

    vtkActor* obstacleActor = vtkActor::New();
    obstacleActor->SetMapper(cylinderMapper);
    obstacleActor->GetProperty()->SetColor(0,0,0);
    //obstacleActor->GetProperty()->SetRepresentationToWireframe();
    obstacleActor->RotateX(90); // Rotate 90 degrees in XX axis

    return obstacleActor;
}

vtkActor* FieldWidget3D::createDebugPt(){
    // Setup four points
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      float zz = 0.00;

      points->InsertNextPoint( 0.05, 0.05, zz);
      points->InsertNextPoint( 0.15, 0.05, zz);
      points->InsertNextPoint( 0.15,-0.05, zz);
      points->InsertNextPoint( 0.05,-0.05, zz);
      points->InsertNextPoint( 0.05,-0.15, zz);
      points->InsertNextPoint(-0.05,-0.15, zz);
      points->InsertNextPoint(-0.05,-0.05, zz);
      points->InsertNextPoint(-0.15,-0.05, zz);
      points->InsertNextPoint(-0.15, 0.05, zz);
      points->InsertNextPoint(-0.05, 0.05, zz);
      points->InsertNextPoint(-0.05, 0.15, zz);
      points->InsertNextPoint( 0.05, 0.15, zz);

      int nPoints = 12;

      // Create the polygon
      vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
      polygon->GetPointIds()->SetNumberOfIds(nPoints);
      for(int i = 0; i < nPoints; i++)
        polygon->GetPointIds()->SetId(i, i);

      // Add the polygon to a list of polygons
      vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
      polygons->InsertNextCell(polygon);

      // Create a PolyData
      vtkSmartPointer<vtkPolyData> polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
      polygonPolyData->SetPoints(points);
      polygonPolyData->SetPolys(polygons);

      // Create a mapper and actor
      vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInput(polygonPolyData);

      vtkActor* actor = vtkActor::New();
      actor->SetMapper(mapper);
      actor->RotateZ(45);

      return actor;
}

void FieldWidget3D::updateGridView()
{
    if(!heightVisible)
    {
        if(heightActor != NULL)
            deleteGridView();

        return;
    }else if(heightActor == NULL)
    {
        initGridView();
    }

    heightActor->SetVisibility(1);
    heightActor->GetProperty()->SetOpacity(0.8);

    // GRID SIZE: 81x57

    //int sizeL = 81;
    //int sizeW = 57;
    //double resolution = 0.25;
    double xx, yy, zz;
    double minz = 2010.0;
    double maxz = -2010.0;
    int i = 0;

    //addtions
    GridView grid;
    DB_get(0,GRIDVIEW, &grid);
    if(grid.count > 0)
    {

    	for(int i=0;i<grid.count;i++)
    	{
    		xx=grid.grid[i].pos.x;
    		yy=grid.grid[i].pos.y;
    		float realVal = grid.grid[i].val;

    		if(!height3D)
    			zz = -0.01;
    		else
    			zz = realVal;
    		heightPoints->SetPoint(i, xx,yy,zz);

    		if(i==0)
    		{
    			minz = realVal;
    			maxz = realVal;
    		}else{
    			if(realVal < minz)
    				minz = realVal;
    			if(realVal > maxz)
    				maxz = realVal;
    		}
    		//cout<<"XX="<<xx<<" YY="<<yy<<" ZZ="<<zz<<" I="<<i<<endl;

    	}

    }else{
    	minz = 0.0;
    	maxz = 0.0;
    }

    if(height3D)
    {
        for(int j = 0; j < i; j++)
        {
            heightPoints->SetPoint(j, heightPoints->GetPoint(j)[0],heightPoints->GetPoint(j)[1],heightPoints->GetPoint(j)[2] - minz);
        }
    }

    heightPolyData->SetPoints(heightPoints);
    heightDelaunay->SetInput(heightPolyData);
    heightDelaunay->Update();
    heightPolyDataAfterInterp  = heightDelaunay->GetOutput();

    // Create the color map
    vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
    colorLookupTable->SetTableRange(minz, maxz);

    if(heightColor)
    {
        colorLookupTable->SetValueRange(1, 1);
        colorLookupTable->SetSaturationRange(1, 1);
        //colorLookupTable->SetHueRange(0, 1);
    }
    else
    {
        colorLookupTable->SetValueRange(0, 1);
        colorLookupTable->SetSaturationRange(0, 0);
        //colorLookupTable->SetHueRange(0, 0);
    }
    colorLookupTable->Build();

    // Generate the colors for each point based on the color map
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);

    for(int i = 0; i < heightPolyDataAfterInterp->GetNumberOfPoints(); i++)
    {
        double p[3];
        heightPolyDataAfterInterp->GetPoint(i,p);

        double dcolor[3];
        colorLookupTable->GetColor(grid.grid[i].val, dcolor);
        unsigned char color[3];
        for(unsigned int j = 0; j < 3; j++)
        {
            color[j] = static_cast<unsigned char>(255.0 * dcolor[j]);
        }
        colors->InsertNextTupleValue(color);
    }

    heightPolyDataAfterInterp->GetPointData()->SetScalars(colors);
}

void FieldWidget3D::initGridView(){

    heightPoints = vtkPoints::New(); // Create a grid of points (height/terrian map)
    heightPolyData = vtkPolyData::New();
    heightDelaunay = vtkDelaunay2D::New();

    int sizeL = 81;
    int sizeW = 57;
    float resolution = 0.25;
    for(int x = -sizeW/2; x <= sizeW/2; x++)
    {
        for(int y = -sizeL/2; y <= sizeL/2; y++)
        {
            heightPoints->InsertNextPoint(x*resolution, y*resolution, x*resolution*y*resolution);
        }
    }

    heightPolyData->SetPoints(heightPoints); // Add the grid points to a polydata object
    heightDelaunay->SetInput(heightPolyData);

    // Triangulate the grid points
    heightDelaunay->Update();
    heightPolyDataAfterInterp = heightDelaunay->GetOutput();

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(heightPolyDataAfterInterp->GetProducerPort());
    heightActor = vtkActor::New();
    heightActor->SetMapper(mapper);
    heightActor->SetVisibility(0);
    heightActor->GetProperty()->SetAmbient(1);
    heightActor->GetProperty()->SetSpecular(0);
    heightActor->GetProperty()->SetDiffuse(0);

    // Add the actor to the scene
    renderer->AddActor(heightActor);
}

void FieldWidget3D::deleteGridView(){
    if(heightActor == NULL)
        return;

    renderer->RemoveActor(heightActor);

    heightDelaunay->Delete();
    heightPolyDataAfterInterp->Delete();
    heightPolyData->Delete();
    heightPoints->Delete();

    heightActor = NULL;
    heightDelaunay = NULL;
    heightPolyDataAfterInterp = NULL;
    heightPolyData = NULL;
    heightPoints = NULL;

}

void FieldWidget3D::setTop(bool top)
{
    this->top = top;

    fprintf(stderr,"TOP\n");

    if(top)
    {
        camera->SetPosition( (camera->GetPosition()[0] < 0)?-0.001:0.001 , 0 , 25);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp((camera->GetPosition()[0] < 0)?1:-1,0,0);
    }else{
        if(camera->GetPosition()[0] < 0)
            camera->SetPosition(-_FIELD_WIDTH, 0, 22);
        else
            camera->SetPosition(_FIELD_WIDTH, 0, 22);

        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0,0,1);
    }
}

void FieldWidget3D::lock(bool lock)
{
    this->lockCam = lock;
}
