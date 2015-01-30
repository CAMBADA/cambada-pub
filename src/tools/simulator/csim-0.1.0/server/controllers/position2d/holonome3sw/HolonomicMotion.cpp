#include "HolonomicMotion.h"
#include <stdio.h>
#include <math.h> 
#include <stdlib.h>
#include "rtdb.h"

#include "Robot.h"
#include "WorldStateDefs.h"

//#define DEBUG

//FILE* HolonomicMotion::log = NULL;


HolonomicMotion::HolonomicMotion()
{
	wsp_factor = N_CONT * DESMUL;		    	// Transformation factor to get set_point
	wsp_factor *= TIME_TICK * K_HCTL / WHEEL_PER;	// Transformation factor to get set_point
	
	desiredVelX = desiredVelY = desiredVelRot = 0;
	velX = velY = w_rot = 0;
	speriodo = (double) MOTION_TICK/1000.0;

	dxs = dys = 0;		
	
	delta_teta = ang_cor = f_vel_ang = 0.0;
	f_ang_corr = 1.0;
	
	vxf = vyf = 0.0;

	//log = fopen("HolonomicMotionLog.txt","w"); 
}


int HolonomicMotion::GetSetPoints(double vx, double vy, double vrot, int *m1, int *m2, int *m3)
{

if(isnan(vx)) vx=0;
if(isnan(vy)) vy=0;
if(isnan(vrot)) vrot=0;

  //RobotWS mySelf;
  //DB_get(Whoami(),ROBOT_WS,&mySelf);

  //fprintf(log,"-------------------NOVO CICLO-----------------------\n");
  //fprintf(log,"\nHolonomic Motion : desiredVelX = %lf, desiredVelY = %lf,desiredVelRot = %lf\n",vx,vy,-vrot);	
  //fprintf(log,"\nHolonomic Motion : velX = %lf, velY = %lf,w_rot = %lf\n",velX,velY,w_rot);
//fprintf(log,"\nRole : %s \n",role_names[mySelf.me.role]);
//fprintf(log,"\nBehavior : %s \n", behaviour_names[mySelf.me.behaviour]);


	
	int overflowVal;
	double corrFactor=1;
	
	desiredVelX = vx;
	desiredVelY = vy;

	desiredSpeed = sqrt(desiredVelX*desiredVelX + desiredVelY*desiredVelY);
/*	
	if( vy <= 1.3 && vy >1.0  )
		desiredVelY = 1.3;
	if( vy <= 1.0 && vy>=0.8  )
		desiredVelY = 0.7;
*/	
	desiredVelRot = -vrot;

	TickPathStep();
	overflowVal = TickMotion();

	if(overflowVal != 0)
	{
		corrFactor = (double)SETPOINT_MAX / overflowVal / f_ang_corr;
		
#ifdef DEBUG	
		printf("%d\n", overflowVal);
		printf("SP1=%d, SP2=%d, SP3=%d\n", sp1, sp2, sp3);
		printf("sp1=%f, sp2=%f, sp3=%f\n", (double)sp1 * corrFactor, (double)sp2 * corrFactor, (double)sp3 * corrFactor);
		printf("VX=%.3f, VY=%.3f VW=%.3f\n", desiredVelX, desiredVelY, desiredVelRot);
#endif
	
		desiredVelX = desiredVelX * corrFactor;
		desiredVelY = desiredVelY * corrFactor;
		desiredVelRot = desiredVelRot * corrFactor;

#ifdef DEBUG	
		printf("vx=%.3f, vx=%.3f vw=%.3f\n", desiredVelX, desiredVelY, desiredVelRot);
#endif
	}

//	if(!mySelf.ball.engaged)
		NewSlopeFilter(HARD);
//	else
//		SlopeFilter(HARD);


	//put last calculated command in RTDB
	//HWcomm_Set_Last_Vel(velX,velY,-w_rot);

	TickPath(); 			// defines the path and binds the translational and rotational velocity
	overflowVal =  TickMotion();
	*m1 = sp1;
	*m2 = sp2;
	*m3 = sp3;

	
#ifdef DEBUG
	printf("SP1=%d, SP2=%d, SP3=%d #Vx=%.3f, Vy=%.3f Vw=%.3f\n", sp1, sp2, sp3, velX, velY, w_rot);
#endif
	return overflowVal;	
}

/* ********Slope_filter 
 * Esta funcao faz um clip na acelera��o maxima que o robot pode ter
 * A fun��o faz a limita��o recorendo ao vector diferen�a entre a velocidade linear 
 * pretendida e actual.
 * Assim torna-se possivel limitar instantaneamente o modulo da velocidade instantanea sem alterar 
 * a trajectoria (direccao) desejada. Se esta limita��o fosse imposta independentemente a cada roda
 * o que poderia acontecer era o robot alterar a sua trajectoria.
 */

char HolonomicMotion::SlopeFilter(char slopeType)
{	
	double temp;				
	double ls_slope, w_slope;
	double diff_velX;			/* Difference velocity vector and direction [m/s] */
	double diff_velY;
	double diff_v_norm;			/* Difference velocity norm */
	double diff_v_angle; 		/* Difference velocity angle */
	char in_acceleration = 0;
	
	if(slopeType == SOFT)
	{
		ls_slope = V_SOFT_SLOPE;
		w_slope = W_SOFT_SLOPE;
	}	
	else
	{
		ls_slope = V_HARD_SLOPE;
		w_slope = W_HARD_SLOPE;
	}

	/* Diff vector calculation */
	diff_velX = velX - desiredVelX;
	diff_velY = velY - desiredVelY;
	diff_v_norm = sqrt(diff_velX*diff_velX + diff_velY*diff_velY );	
	diff_v_angle = atan2( diff_velY, diff_velX );
	
	if ( diff_v_norm > ls_slope )
	{
		diff_v_norm -= ls_slope;
		in_acceleration = 1;
	}
	else 
		diff_v_norm = 0;

	diff_velX = diff_v_norm * cos( diff_v_angle );
	diff_velY = diff_v_norm * sin( diff_v_angle );

	velX = desiredVelX + diff_velX;
	velY = desiredVelY + diff_velY;

	/* Maintain the w_rot maximum variation per tick */
	if ( desiredVelRot > w_rot )
	{
		temp = w_rot + w_slope;
		if ( desiredVelRot > temp )
		{
			w_rot = temp;
			in_acceleration = 1;		
		}
		else
		{
			w_rot = desiredVelRot;
		}
	} 
	else
	{
		temp = w_rot - w_slope;
		if ( temp > desiredVelRot )
		{
			w_rot = temp;
			in_acceleration = 1;
		}
		else
		{
			w_rot = desiredVelRot;
		}
	}
	return in_acceleration;
}



void HolonomicMotion::TickPath(void)
{
	dxs = velX * speriodo;					/* Dist�ncia linear percorrida em x por per�odo */
	dys = velY * speriodo;					/* Dist�ncia linear percorrida em y por per�odo */
	
	delta_teta = w_rot * speriodo;			/* Variacao do angulo de orientacao do robot por periodo */
	ang_cor = delta_teta / 2;				/* Angulo de correc��o em fun��o de teta */

	/* Performs compensation */
	if ( w_rot != 0 )
		f_ang_corr = delta_teta / (2 * sin(ang_cor));	/* @MBC Factor de correc��o para o m�dulo da dist�ncia */
	else
		f_ang_corr = 1;

	f_vel_ang = R_ROBOT * w_rot;			/* @MBC Factor de velocidade angular das rodas em fun��o do raio */
}


void HolonomicMotion::TickPathStep(void)
{
	dxs = desiredVelX * speriodo;			/* Dist�ncia linear percorrida em x por per�odo */
	dys = desiredVelY * speriodo;			/* Dist�ncia linear percorrida em y por per�odo */
	
	delta_teta = desiredVelRot * speriodo;	/* Variacao do angulo de orientacao do robot por periodo */
	ang_cor = delta_teta / 2;				/* Angulo de correc��o em fun��o de teta */

	/* Performs compensation */
	if ( desiredVelRot != 0 )
		f_ang_corr = delta_teta / (2 * sin(ang_cor));	/* @MBC Factor de correc��o para o m�dulo da dist�ncia */
	else
		f_ang_corr = 1;

	f_vel_ang = R_ROBOT * desiredVelRot;	/* @MBC Factor de velocidade angular das rodas em fun��o do raio */
}



int HolonomicMotion::TickMotion(void)
{
	int temp3, overflow;
	double temp1, temp2;
	double modulo_vc = 0;				/* Modulo da distancia compensada */
	double ang_vc;						// Fase da distancia compensada

	/* Compensated velocity calculus */
	temp1 = sqrt(dxs * dxs + dys * dys);
	modulo_vc = temp1 * f_ang_corr;		/* @MBC Modulo da dist�ncia compensada */
	
	temp2 = atan2(dys, dxs);
	ang_vc = temp2 - ang_cor;			/* @MBC Angulo da dist�ncia compensada */
		
	/* Compensated velocity in cartesian coords */
	vxf = (modulo_vc * cos(ang_vc)) / speriodo;
	vyf = (modulo_vc * sin(ang_vc)) / speriodo;

	/* Cumputes motor SetPoints */
	overflow = abs( CalcSetpoint(1, &sp1) );
	temp3 = abs(CalcSetpoint(2, &sp2));
	if(temp3 > overflow)
		overflow = temp3;
	
	temp3 = abs(CalcSetpoint(3, &sp3));
	if(temp3 > overflow)
		overflow = temp3;
	
	return overflow;
}


int HolonomicMotion::CalcSetpoint(char motor, int *calc_setpoint)
{
	int overflow;

	/* @MBC Calculo dos Setpoints dos 3 motores */
	switch(motor)
	{
	case 1:
		*calc_setpoint = (signed int)(((vxf * cos( ANG_MOTOR_1 ) + vyf * sin( ANG_MOTOR_1 ))*2 + f_vel_ang) * wsp_factor);
		break;	

	case 2:
		*calc_setpoint = (signed int)(((vxf * cos( ANG_MOTOR_2 ) + vyf * sin( ANG_MOTOR_2 ))*2 + f_vel_ang) * wsp_factor);
		break;	

	case 3:
		*calc_setpoint = (signed int)(((vxf * cos( ANG_MOTOR_3 ) + vyf * sin( ANG_MOTOR_3 ))*2 + f_vel_ang) * wsp_factor);
		break;	
	}
	overflow = 0;
	
        if( (*calc_setpoint > 2*SETPOINT_MAX) || (*calc_setpoint < -SETPOINT_MAX*2) )
	{
		overflow = *calc_setpoint;
	}
        return overflow/2.0;
}

void HolonomicMotion::NewSlopeFilter(char slopeType)
{

	double temp;				
	double ls_slope, w_slope;
	double diff_velX;			/* Difference velocity vector and direction [m/s] */
	double diff_velY;
	double diff_v_norm;			/* Difference velocity norm */
	double diff_v_angle; 		/* Difference velocity angle */
	char in_acceleration = 0;
	double angle_last_vel,angle_vel;
	double last_vel;
	double diff_angles;
	
	if(slopeType == SOFT)
	{
		ls_slope = V_SOFT_SLOPE;
		w_slope = W_SOFT_SLOPE;
	}	
	else
	{
		ls_slope = V_HARD_SLOPE;
		w_slope = W_HARD_SLOPE;
	}

//fprintf(log,"\nNewSlopeFilter : desiredVelX = %lf, desiredVelY = %lf,desiredVelRot = %lf\n",desiredVelX,desiredVelY,desiredVelRot);	
//fprintf(log,"\nNewSlopeFilter : velX = %lf, velY = %lf,w_rot = %lf\n",velX,velY,w_rot);

	/* Diff vector calculation */
	diff_velX = velX - desiredVelX;
	diff_velY = velY - desiredVelY;
	diff_v_norm = sqrt(diff_velX*diff_velX + diff_velY*diff_velY );

//fprintf(log,"\nNewSlopeFilter : diff_velX = %lf, diff_velY = %lf\n",diff_velX,diff_velY);	

	angle_last_vel = atan2(velY,velX);
	if(angle_last_vel < 0)
		angle_last_vel += iM_2_PI;
	last_vel = sqrt(velX*velX + velY*velY);

	angle_vel = atan2(desiredVelY,desiredVelX);
	if(angle_vel < 0)
		angle_vel += iM_2_PI;
	diff_angles = fabs(angle_last_vel - angle_vel);

	
	if(diff_v_norm < ls_slope)
		diff_v_norm = 0;
	else
	{
		if( (diff_angles < iM_PI/2) &&  (desiredVelX !=0 || desiredVelY !=0) )
		{

			double m,r,xC,yC,discriminant,A,B,C;

			m = tan(angle_vel);
			r = ls_slope;
			xC = velX;
			yC = velY;

			A = (1 + m*m);
			B = (-2 * xC - 2 * m * yC);
			C = (xC*xC + yC*yC - r*r);

			discriminant = B*B - 4 * A * C;

			if(fabs(sin(diff_angles)*last_vel) > ls_slope || discriminant < 0 )
			{
				double m1,m2,b1,b2;
				m1 = tan(angle_vel);
				b1 = 0;
				m2 = tan(angle_vel+ iM_PI/2);
				b2 = velY - m2*velX;

				desiredVelX = (b1 - b2) / (m2 -m1);
				desiredVelY = m1*desiredVelX + b1;

				/* Diff vector calculation */
				diff_velX = velX - desiredVelX;
				diff_velY = velY - desiredVelY;
				diff_v_norm = sqrt(diff_velX*diff_velX + diff_velY*diff_velY ) - ls_slope;
			}
			else
			{
				double x1,y1,dist1,x2,y2,dist2;
		
				
//fprintf(log,"\nNewSlopeFilter : m = %lf, r = %lf, xC = %lf, yC = %lf\n",m,r,xC,yC);

				
//fprintf(log,"\nNewSlopeFilter : A = %lf, B = %lf, C=%lf\n",A,B,C);

				x1 = (-B + sqrt(B*B - 4 * A * C))/ ( 2 * A);
				y1 = m*x1;

				x2 = (-B - sqrt(B*B - 4 * A * C))/ ( 2 * A);
				y2 = m*x2;

				dist1 = sqrt( (desiredVelX-x1)*(desiredVelX-x1) + (desiredVelY-y1)*(desiredVelY-y1) );
				dist2 = sqrt( (desiredVelX-x2)*(desiredVelX-x2) + (desiredVelY-y2)*(desiredVelY-y2) );

				if(dist1 < dist2)
				{
					desiredVelX = x1;
					desiredVelY = y1;
				}
				else
				{
					desiredVelX = x2;
					desiredVelY = y2;
				}
				/* Diff vector calculation */
				diff_velX = velX - desiredVelX;
				diff_velY = velY - desiredVelY;
				diff_v_norm = 0;
			}
		}
		else
			diff_v_norm -= ls_slope;
	}

	diff_v_angle = atan2( diff_velY, diff_velX );
			
	diff_velX = diff_v_norm * cos( diff_v_angle );
	diff_velY = diff_v_norm * sin( diff_v_angle );

	velX = desiredVelX + diff_velX;
	velY = desiredVelY + diff_velY;

	double speed = sqrt(velX*velX + velY*velY);

	if(desiredSpeed)
		if(speed/desiredSpeed>1)
			desiredVelRot = speed/desiredSpeed * desiredVelRot;

	/* Maintain the w_rot maximum variation per tick */
	if ( desiredVelRot > w_rot )
	{
		temp = w_rot + w_slope;
		if ( desiredVelRot > temp )
		{
			w_rot = temp;
			in_acceleration = 1;		
		}
		else
		{
			w_rot = desiredVelRot;
		}
	} 
	else
	{
		temp = w_rot - w_slope;
		if ( temp > desiredVelRot )
		{
			w_rot = temp;
			in_acceleration = 1;
		}
		else
		{
			w_rot = desiredVelRot;
		}
	}

//fprintf(log,"\nNewSlopeFilter output : velX = %lf, velY = %lf,w_rot = %lf\n",velX,velY,w_rot);

}


#if 0
// --- Programa principal -------------------------------
void main(void)
{
	int i;
	int sp1, sp2, sp3;
	double desiredVelX;	/* Sets xx Velocity */
	double desiredVelY;	/* Sets yy Velocity */
	double desiredVelRot;	/* Sets angular velocity */

	HolonomicMotion hm;


	desiredVelX = -2.8;	/* Sets xx Velocity */
	desiredVelY = 0.0;	/* Sets yy Velocity */
	desiredVelRot = 2.5;	/* Sets angular velocity */
	
	i=0;
	while (1)
	{
		Sleep(500);
		
		i++;
		if(i == 40)
		{
			printf("A\n");
		
			desiredVelX = -1.0;	/* Sets xx Velocity */
			desiredVelY = 2.8;	/* Sets yy Velocity */
			desiredVelRot = 1.5;	/* Sets angular velocity */
		}			
		else if(i== 80)
		{
			printf("B\n");
			
			desiredVelX = 1.0;	/* Sets xx Velocity */
			desiredVelY = 2.5;	/* Sets yy Velocity */
			desiredVelRot = 2.0;	/* Sets angular velocity */
		}
		else if(i == 120)
		{
			printf("C\n");

			desiredVelX = 0.0;	/* Sets xx Velocity */
			desiredVelY = 0.0;	/* Sets yy Velocity */
			desiredVelRot = 0.0;	/* Sets angular velocity */
		}
		else if(i == 160)
			break;

		hm.GetSetpoints(desiredVelX, desiredVelY, desiredVelRot, &sp1, &sp2, &sp3);
		
	}
}
#endif


