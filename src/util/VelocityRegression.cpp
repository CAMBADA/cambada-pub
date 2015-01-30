/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA UTILITIES
 *
 * CAMBADA UTILITIES is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA UTILITIES is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "VelocityRegression.h"

using namespace cambada;
using namespace cambada::geom;

namespace cambada {
namespace util {

VelocityRegression::VelocityRegression()
{
	MAXSIZE = 9;
}


VelocityRegression::VelocityRegression( int maxSize)
{
	MAXSIZE = maxSize;
}


VelocityRegression::~VelocityRegression()
{}


void VelocityRegression::putNewValues( Vec pos, float ori, struct timeval instantIn)
{
	
	instant = instantIn;	//changes the comparison time instant to the one being currently added
	
	posBuffer.push_back( pos );
	oriBuffer.push_back( ori );
	timeBuffer.push_back( instantIn );
//fprintf(stderr,"DATA IN POSBUFFER TO INSERT X: %f, Y: %f\n",position.x, position.y);
//fprintf(stderr,"DATA BUFF MAXSIZE: %d\n",MAXSIZE);
	if( posBuffer.size() > MAXSIZE )
	{
		posBuffer.pop_front();
		oriBuffer.pop_front();
		timeBuffer.pop_front();
	}
/*	
	for(unsigned int i = 0 ; i < posBuffer.size() ; i++ )
	{		
fprintf(stderr,"DATA VALUES IN BUFFER x:%f, y:%f\n",posBuffer[i].x,posBuffer[i].y);
	}
*/
}


void VelocityRegression::clearBuffs()
{
	posBuffer.clear();
	timeBuffer.clear();
}


Vec VelocityRegression::getLinearVelocity()
{
	if (posBuffer.size()<2)
		return Vec(0,0);

	double sum_ts = 0.0, sum_ts2 = 0.0;
	Vec sum_pos = Vec(0,0);
	Vec sum_ts_pos(0,0);

	for(unsigned int i = 0 ; i < posBuffer.size() ; i++ )
	{
		double tau = -(instant.tv_sec*1000 + instant.tv_usec/1000) 
						+ ( timeBuffer[i].tv_sec*1000 + timeBuffer[i].tv_usec/1000);

		sum_pos += posBuffer[i];
		sum_ts += tau;
		sum_ts2 += tau*tau;
		sum_ts_pos += tau*posBuffer[i];
		
//fprintf(stderr,"DATA VALUES IN BUFFER x:%f, y:%f\n",posBuffer[i].x,posBuffer[i].y);
	}
//fprintf(stderr,"DATA BuffSize: %d\n",posBuffer.size());
	double det = (double)posBuffer.size() * sum_ts2 - sum_ts * sum_ts;	//DENOMINADOR DA REGRESSAO LINEAR

	if(  fabs(det) < 1e-5 )
	{
//fprintf(stderr,"DATA ERROR DET MUITO PEQUENO\n");
		return (Vec::zero_vector);
	}
	else
	{
		return (( -sum_ts*sum_pos + (double)posBuffer.size()*sum_ts_pos) / det);
	}
}


float VelocityRegression::getAngularVelocity()
{
	if (oriBuffer.size()<2)
		return 0.0;

	double sum_ts = 0.0, sum_ts2 = 0.0;
	double sum_pos = 0.0;
	double sum_ts_pos = 0.0;

	for(unsigned int i = 0 ; i < oriBuffer.size() ; i++ )
	{
		double tau = -(instant.tv_sec*1000 + instant.tv_usec/1000) 
						+ ( timeBuffer[i].tv_sec*1000 + timeBuffer[i].tv_usec/1000);

		sum_pos += oriBuffer[i];
		sum_ts += tau;
		sum_ts2 += tau*tau;
		sum_ts_pos += tau*oriBuffer[i];
		
//fprintf(stderr,"DATA VALUES IN BUFFER x:%f, y:%f\n",posBuffer[i].x,posBuffer[i].y);
	}
//fprintf(stderr,"DATA BuffSize: %d\n",posBuffer.size());
	double det = (double)oriBuffer.size() * sum_ts2 - sum_ts * sum_ts;	//DENOMINADOR DA REGRESSAO LINEAR

	if(  fabs(det) < 1e-5 )
	{
//fprintf(stderr,"DATA ERROR DET MUITO PEQUENO\n");
		return 0.0;
	}
	else
	{
		return (( -sum_ts*sum_pos + (double)oriBuffer.size()*sum_ts_pos) / det);
	}
}


Vec VelocityRegression::getPointInOrig()
{
	double sum_ts = 0.0, sum_ts2 = 0.0;
	Vec sum_pos = Vec(0,0);
	Vec sum_ts_pos(0,0);

	for(unsigned int i = 0 ; i < posBuffer.size() ; i++ )
	{
		double tau = -(instant.tv_sec*1000 + instant.tv_usec/1000) 
						+ ( timeBuffer[i].tv_sec*1000 + timeBuffer[i].tv_usec/1000);
						
		sum_pos += posBuffer[i];
		sum_ts += tau;
		sum_ts2 += tau*tau;
		sum_ts_pos += tau*posBuffer[i];	
	}

	double det = (double)posBuffer.size() * sum_ts2 - sum_ts * sum_ts;	//DENOMINADOR DA REGRESSAO LINEAR

	if(  fabs(det) < 1e-5 )
	{
		return (Vec::zero_vector);	
	}
	else
	{
		return (( sum_ts2 * sum_pos - sum_ts_pos * sum_ts ) / det);
	}
}


void VelocityRegression::printPosBuff()
{
	for(unsigned int i = 0 ; i < posBuffer.size() ; i++ )
	{		
		fprintf(stderr,"DATA VALUES IN BUFFER x:%f, y:%f\n", posBuffer[i].x, posBuffer[i].y);
	}
}


void VelocityRegression::printOriBuff()
{
	for(unsigned int i = 0 ; i < oriBuffer.size() ; i++ )
	{		
		fprintf(stderr,"DATA VALUES IN BUFFER ori:%f\n", oriBuffer[i]);
	}
}


void VelocityRegression::printTimeBuff()
{
	for(unsigned int i = 0 ; i < timeBuffer.size() ; i++ )
	{		
		fprintf(stderr,"DATA VALUES IN BUFFER time:%ld\n", timeBuffer[i].tv_sec*1000 + timeBuffer[i].tv_usec/1000);
	}
}


void VelocityRegression::resetToNSamples( unsigned int nSamples )
{
	if( posBuffer.size() > nSamples )
	{
		int toDelete = posBuffer.size() - nSamples;
		for (int count = 0; count < toDelete; count++)
		{
			posBuffer.pop_front();
			timeBuffer.pop_front();
		}
	}
	fprintf(stderr,"INTEGRATOR VELOCITY RESET TO %d ELEMENTS\n",posBuffer.size());
}


void VelocityRegression::collision(float t, Vec collisionPos, Vec newVel)
{
//fprintf(stderr,"COLLISION DETECTED\n");
	resetToNSamples(3); // reset to minimum recommended buffer size
printPosBuff();
printTimeBuff();	

	float referencial = instant.tv_sec*1000 + instant.tv_usec/1000 + t;
	unsigned int size = posBuffer.size();
	for(unsigned int i = 0 ; i < size ; i++ )
	{
//fprintf(stderr," Iteração = %d\n",i);
		double relT = - referencial + ( timeBuffer[i].tv_sec*1000 + timeBuffer[i].tv_usec/1000);
		Vec newPos(collisionPos.x + newVel.x*relT,collisionPos.y + newVel.y*relT);
		posBuffer.push_back(newPos);
		timeBuffer.push_back(timeBuffer[i]);
	}
	resetToNSamples(3);
//printPosBuff();
//printTimeBuff();	
}

void VelocityRegression::checkAngleDiscontinuity(float newAngle)
{
	if (posBuffer.size() == 0)
		return;
	if(Angle(posBuffer[posBuffer.size()-1].x/1000).in_between(Angle(0),Angle(M_PI/2)) && Angle(newAngle/1000).in_between(Angle(3*M_PI/2),Angle(2*M_PI)))
		for(unsigned int i=0;i<posBuffer.size();i++)
			posBuffer[i].x+=2*M_PI*1000;
	else if(Angle(posBuffer[posBuffer.size()-1].x/1000).in_between(Angle(3*M_PI/2),Angle(2*M_PI)) && Angle(newAngle/1000).in_between(Angle(0),Angle(M_PI/2)))
		for(unsigned int i=0;i<posBuffer.size();i++)
			posBuffer[i].x-=2*M_PI*1000;

}

}
}
