/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA AGENT
 *
 * CAMBADA AGENT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA AGENT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef cambada_loc_h
#define cambada_loc_h

#include "ConfigXML.h"
#include "FieldLUT.h"
#include "VisualPositionOptimiser.h"
#include "RobotPositionKalmanFilter.h"

#include "VisionInfo.h"

#include <deque>
#include "PositionKF.h"

using namespace cambada::geom;

namespace cambada {
namespace loc {

// ****************************************************************************
// IMPORTANTE
//
// Sistemas de coordenadas dos Tribots:
// 1. Mundo
//    - Refer�ncia: centro do campo
//    - Eixo dos XX: largura do campo
//    - Eixo dos YY: comprimento do campo
//    - YY positivo: metade do campo da equipa adversaria (cresce positivamente no sentido da baliza advers�ria)
//    - XX positivo: parte direita do campo
// 2. Robot
//    - Refer�ncia: centro do robot
//    - YY positivo: parte direita do robot
//    - XX positivo: frente do robot
//
// ==> H� aqui uma aparente contradi��o nos sistemas de coordenadas
//    do mundo e do robot, mas a an�lise do c�digo conduz a isso... <==
// ==> A imagem que e' passada para a localiza�ao esta' rodada de +90� (pixel (0,0) corresponde
//		a X negativo e Y negativo do mundo)
//		(ou os pontos sao previamente rodados, ou a camara esta montada nessa posicao)
//		Assim, quando o robot esta' virado para a baliza adversaria o "heading"
//		que resulta da localizacao e' 90�
//
//    Heading medido a partir do eixo dos XX (mundo): Heading > 0 => anti-clockwise 
//    
// Sistema de coordenadas CAMBADA:
// 1. Mundo
//   - Refer�ncia: centro do campo
//   - Eixo dos XX: largura do campo
//   - Eixo dos YY: comprimento do campo
//   - YY positivo: metade do campo da equipa adversaria (cresce positivamente no sentido da baliza advers�ria)
//   - XX positivo: parte direita do campo
// 2. Robot
//    - Refer�ncia: centro do robot
//    - YY positivo: frente do robot
//    - XX positivo: parte direita do robot 
//
// ==> No cambada a camara esta' montada em posi�ao "normal" (rodada de -90� relativamente
//		'a posi�ao dos Tribots): o pixel (0,0) corresponde a X negativo e Y positivo do mundo. 
//		Assim, quando o robot esta' virado para a baliza adversaria o "heading"
//		que resulta da localizacao e' 0�. Assim, o sistema de eixos do robot e do
//		mundo coincidem
//
//    Heading medido a partir do eixo dos YY (mundo): Heading > 0 => anti-clockwise 
// ****************************************************************************
//

#define _DEBUG		0

#define MY_HALF		-1
#define THEIR_HALF	1

using namespace cambada::util;

class CambadaLoc {
  private:
	Vec robot_pos;
	Angle robot_heading;
	double robot_pos_quality;

    FieldLUT* field_lut;                       // The spacer table 
    VisualPositionOptimiser* vis_optimiser;    // Optimization routine for visual sensor information 
  
	RobotPositionKalmanFilter kalman_filter;
	PositionKF kf;
	double ref_error;
	double latest_error;
	deque<Vec> dq_pos;
	
	int cfield_length;
	int cfield_width;
	int cside_band_width;
	int cgoal_band_width;

  public:
    CambadaLoc( ConfigXML* config );
    ~CambadaLoc();

	double UpdateRobotPosition(vector< Vec >& lines, double dx, double dy, double dw);
	double UpdateRobotPosition_KF(vector< Vec >&, double, double, double);
/*
    void UpdateRobotPosition_KF_Goals(vector< Vec >& lines, vector<Vec> & linesGoals, double odo_deltax, double odo_deltay, double odo_deltaphi);
    double UpdateRobotPosition_KF_GoalAngles( vector< Vec >& lines, 
		double odo_deltax, double odo_deltay, double odo_deltaphi,
	    GoalSensor &visOurGoal, GoalSensor &visTheirGoal);
*/
	
	double UpdateRobotPosition (vector< Vec >&);		// For Debug ONLY
	void UpdateRobotPosition_Odometry(double, double, double);	// For Debug ONLY
	double UpdateRobotPosition_Vision_and_Odometry(vector< Vec >&, double, double, double);	// For Debug ONLY

    double GetRobotPosition (Vec&, Angle&);

	double FindInitialPosition(vector< Vec >&, int fieldHalf = MY_HALF );
	double FindInitialPositionWithKnownOrientation(vector< Vec >& lines , Angle orientation );

	void SetRobotPosition( Vec, Angle );
	void GetFieldDimensions(unsigned int &width, unsigned int &length) {width = cfield_width; length = cfield_length;}

	void mirror();
	
	void MirrorRobotPosition();
	double GetError() {return ref_error; };
};

}
}

#endif
