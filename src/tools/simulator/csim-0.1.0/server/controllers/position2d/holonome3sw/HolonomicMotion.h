#ifndef HOLONOMIC_MOTION
#define HOLONOMIC_MOTION

#include <math.h>
#include <stdio.h>
#include "common.h"

#define iM_PI 3.1415
#define iM_2_PI iM_PI*2

#define ANG_MOTOR_1	iM_PI          /* Wheel 1 (0*pi) */
#define ANG_MOTOR_2 -iM_PI/3       /* Wheel 2 (2/3*pi) */
#define ANG_MOTOR_3 iM_PI/3        /* Wheel 3 (4/3*pi) */

#define	N_CONT		500         /* Encoder # Pulses per rotation */
#define	DESMUL		15          /* Motor Demultiplying factor */
#define	TIME_TICK	0.005       /* Tempo de amostragem dos motores (s) */
#define	K_HCTL		4           /* Encoder counter multiplying factor */
#define	R_RODA		0.0573 /* Wheel radius (m) */
#define	R_ROBOT		0.19   /* Robot radius (m) */
#define	WHEEL_PER (iM_2_PI * R_RODA)          /* Wheel perimeter (m) */
 
/********** Aceleration Slopes **********/
/* This acelerations parameters are the aceleration clipping */
#define V_SOFT_ACELERATION	1.4	/* Acelera��o do robot (m/s2)*///estava 0.8
#define V_SOFT_SLOPE		( ((double)MOTION_TICK)/1000.0)*(V_SOFT_ACELERATION)
#define W_SOFT_SLOPE		V_SOFT_SLOPE/R_ROBOT		/* Maximum angular velocity */

#define V_HARD_ACELERATION	4.0 /*2.2*/	/* Acelera��o do robot (m/s2)*/
//#define V_HARD_ACELERATION	2.2 /*2.2*/	/* Acelera��o do robot (m/s2)*/
#define V_HARD_SLOPE		( ( ((double)MOTION_TICK)/1000.0)*(V_HARD_ACELERATION) )
#define W_HARD_SLOPE		(V_HARD_SLOPE/R_ROBOT)		/* Maximum angular velocity */

#define SOFT 	1
#define HARD	0

#define SETPOINT_MAX	900


class HolonomicMotion
{
	double desiredVelX;	/* Desired velocity vector and direction [m/s] */
	double desiredVelY;
	double desiredVelRot;	/* Desired angular speed [rad/s] */
	double desiredSpeed;

// Variaveis relacionadas com o movimento para evitar mais overhead
	double wsp_factor;		/* Ratio Transformation vs wheel perimeter */
	double speriodo;		/* Update period in seconds */

	double dxs;				// Dist�ncia linear percorrida em x por per�odo
	double dys;				// Dist�ncia linear percorrida em y por per�odo
	
	double delta_teta;		// Variacao do angulo de orientacao do robot por periodo
	double ang_cor;			// Angulo de correccao necessario para colocar o robot na trajectoria ideal
	double f_vel_ang;		// Factor de velocidade angular das rodas em fun��o do raio
	double f_ang_corr;
	
	double vxf;				// Velocidade compensada
	double vyf;
	
	int sp1, sp2, sp3;		// setpoints calculados para serem enviados aos motores

	void TickPath(void);
	void TickPathStep(void);
	int TickMotion(void);

	int CalcSetpoint(char motor, signed int *calc_setpoint);
	char SlopeFilter(char);
	void NewSlopeFilter(char);

	//static FILE* log;
	
public:
	HolonomicMotion();
	int GetSetPoints(double vx, double vy, double vrot, int *sp1, int *sp2, int *sp3);
	
	double velX;			/* Running velocity vector and direction [m/s] */
	double velY;
	double w_rot;			/* Running angular speed [rad/s] */
};

#endif

