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

#include "DriveVector.h"

using namespace cambada::geom;

namespace cambada {

DriveVector::DriveVector(WorldState* w) {
	velX=velY=velA=0;
	kickPower = 0;
	grabber = GRABBER_OFF;
	smallAdjustment = false;

	this->world = w;

	// Create kicker table object and load it from file
	kickConf = new KickerConf(world, Whoami());

	ballNotVisibleTimer.restart();
}

DriveVector::DriveVector(WorldState* w, double vX, double vY, double vA, unsigned char k, GrabberType g){
	velX=vX;velY=vY;velA=vA;
	kickPower=k; grabber=g;
	smallAdjustment = false;

	this->world = w;
	// Create kicker table object and load it from file
	kickConf = new KickerConf(world, Whoami());
}

DriveVector::~DriveVector() {
	delete kickConf;
}

void DriveVector::allOff() {
	velX=velY=velA=0;
	kickPower = 0;
	grabber = GRABBER_OFF;
	smallAdjustment = false;
}

void DriveVector::motorsOff() {
	velX=velY=velA=0;
}

void DriveVector::pass( double distance )
{
	if( world->obstaclesInFront(1.0) )	// If there are no obstacles in front of me, up to a distance of X meters
		return;

	/*
	 x = [ 2.0 10.0]
	y = [ 7.0 19.0]
	p = polyfit(x, y, 1)
	xp = 2:1:10
	yp = polyval(p, xp)
	plot(xp, yp)
	*/
	const float P[] = { 1.0, 3.0 };  // m b
	const int   nP = 2;

	float sum = 0.0;
	for( int i = 0 ; i < nP; i++ )
		sum += P[i]*powf(distance,nP-i-1);

	int ff = (int)(sum);

	if( ff > 14 )
		ff = 14;

	if(ff < 4)
		ff = 4;

	kickPower	= (unsigned int)(ff + 0x80);
	grabber = GRABBER_ON;
	//myprintf("KICK: pass dist %.1f vel: %.2f power: %d\n", distance, v.vy, ff);
}

void DriveVector::kickOld( double distance ) {

	if( world->obstaclesInFront(1.0) )	// If there are no obstacles in front of me, up to a distance of X meters
		return;

	kickPower = kickConf->getPower( distance ); // Get power from the table

	// discarding the last command vel, because it is not necessarly the real velocity
	Vec myVel = world->abs2relDelta(world->me->vel);
	float vy = myVel.y;

	// correcção é uma recta com declive negativo
	float correctionFactor = 0;
	float k1 = 0.5;
	if(kickPower >= 35){
		//correctionFactor = -distance * (v.vy * k1) / (div2 - div1) + ((div2-div1) * v.vy * k1) / (div2 - div1);
		correctionFactor = vy * k1;
		correctionFactor = ((correctionFactor < 0.0)? 0.0: correctionFactor);
	}
	float adjustedDistance = distance - correctionFactor;
	kickPower	= kickConf->getPower( adjustedDistance ); // Calculate the new kickPower for the adjusted distance

	int clipping = 80;

	if( kickPower > clipping ) {
		kickPower = clipping;
	} else if( kickPower < 10 ) {
		kickPower = 10;
	}

	grabber = GRABBER_ON;

	//fprintf(stderr,"Kick to dist %.1f -> %.1d\n",distance,kickPower);
	//myprintf("KICK: dist %.1f correctionFactor %.2f distCorrect: %.1f vel: %.2f power: %d\n", distance, correctionFactor, adjustedDistance, vy, kickPower);
}

void DriveVector::kick( double distance, double height )
{
	if( world->obstaclesInFront(1.0) )	// If there are no obstacles in front of me, up to a distance of X meters
		return;

	if (height == -2014.0)
		kickPower = kickConf->getPowerThroughParab(distance);
	else
		kickPower = kickConf->getPowerThroughParab(distance, height);

	if(kickPower < 15)
		kickPower = 15;
	if(kickPower > 50)
		kickPower = 50;
}

void DriveVector::grabberControl(){
	GrabberType grabberMode = GRABBER_OFF;

	if(world->me->ball.visible)
		ballNotVisibleTimer.restart();

	if(world->me->role == rKickCalibrate) 															// Always ON when in KickCalibrate Role
	{
		grabberMode = GRABBER_ON;
	}else if( world->me->role == rStop || world->me->role == rNone 									// Off in some special conditions
			|| world->gameState == preOwnKickOff || world->gameState == preOpponentKickOff
			|| world->gameState == preOwnGoalKick || world->gameState == preOpponentGoalKick
			|| world->gameState == preOwnCornerKick || world->gameState == preOpponentCornerKick
			|| world->gameState == preOwnThrowIn || world->gameState == preOpponentThrowIn
			|| world->gameState == preOwnFreeKick || world->gameState == preOpponentFreeKick
			|| world->gameState == dropBall )
	{
		grabberMode = GRABBER_OFF;
	} else {																						// The default behaviour...
		// Grabber on when the ball is visible in front of the robot
		if( world->me->ball.visible
				&& 	fabs( world->me->ball.posRel.angleFromY().get_deg_180()) < (world->config->getParam("grabber_on_angle"))
				&&  world->me->ball.posRel.length() < (world->config->getParam("grabber_on_distance")) )
		{
			grabberMode = GRABBER_ON;
		} else {
			if(!world->me->ball.visible && ballNotVisibleTimer.elapsed() < 1000)
				grabberMode = GRABBER_ON;
			else
				grabberMode = GRABBER_OFF;
		}
	}

	grabber = grabberMode;
}

float DriveVector::getLinearVel() {
	return sqrt(velX*velX + velY*velY);
}

void DriveVector::setLinearVel(float absLinearVel) {
	Vec vel = Vec(velX, velY);
	vel = vel.setLength(absLinearVel);
	velX = vel.x;
	velY = vel.y;
}

void DriveVector::limitVel(float maxLinVel, float maxAngVel) {

	if(velA > maxAngVel)
		velA = maxAngVel;
	else if(velA < -maxAngVel)
		velA = -maxAngVel;

	if(getLinearVel() > maxLinVel)
	{
		Vec vel = Vec(velX, velY);
		vel = vel.setLength(maxLinVel);
		velX = vel.x;
		velY = vel.y;
	}

	/*if(getLinearVel() > maxLinVel || fabs(velA) > maxAngVel) {
		float ratioLin = getLinearVel()/maxLinVel;
		float ratioAng = fabs(velA)/maxAngVel;
		float factor = maxAngVel/fabs(velA);	// Correct by angular vel (by default)
		if(ratioLin > ratioAng){ 				// Correct by linear vel (if the ratio is higher)
			factor = maxLinVel/getLinearVel();
		}

		velX *= factor;
		velY *= factor;
		velA *= factor;
	}*/
}

} /* namespace cambada */
