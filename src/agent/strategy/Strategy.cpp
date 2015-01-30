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

#include "log.h"
#include "Strategy.h"
#include <string.h>
#include "geometry.h"

#include <fstream>
#include <istream>
#include <algorithm>


namespace cambada
{

Strategy::Strategy(ConfigXML* config, WorldState* world)
{
	this->world = world;
	this->config = config;
	this->field = this->world->getField();
	this->previousBall = Vec::zero_vector;
	this->lastGameState = stopRobot;
	//actualPos = world->whoami() ;
}

bool Strategy::loadFreePlay(char* fname)
{
	if (parse(fname, formationFreePlay) < 0)
		return false;
	else
	{
		for (unsigned int f = 0; f < formationFreePlay.size(); f++)
		{
			myprintf("Strat::load set world form %d to %p\n", f, world);
			formationFreePlay[f]->setWorldConfigStrategy(world, config, this);
		}
		return true;
	}
}

bool Strategy::loadSP(char* fname)
{
	if (parse(fname, formationSP) < 0)
		return false;
	else
	{
		for (unsigned int f = 0; f < formationSP.size(); f++)
		{
			myprintf("Strat::load set world form %d to %p\n", f, world);
			formationSP[f]->setWorldConfigStrategy(world, config, this);
		}
		return true;
	}
}

void Strategy::updateFreePlay()
{
//Dummy code, strategy not used in this version
}

void Strategy::updateSP(bool force)
{
//Dummy code, strategy not used in this version
}

void Strategy::exchange()
{
	double distSP[N_CAMBADAS][N_CAMBADAS]; // [pos][agente]

	for (int agent = 0; agent < N_CAMBADAS; agent++)
	{
		finfo.position[agent] = Vec::zero_vector;
		finfo.posId[agent] = -1;
		finfo.cover[agent] = false;
	}

	for (int pos = 0; pos < N_CAMBADAS; pos++)
	{
		for (int agent = 0; agent < N_CAMBADAS; agent++)
		{
			if (world->robot[agent].role == rGoalie
					|| !world->robot[agent].running)
				distSP[pos][agent] = 1001.0;
			else if (world->robot[agent].handicappedGrabber)
				distSP[pos][agent] = 500.0;
			else
				distSP[pos][agent] =
						(world->robot[agent].pos - SPosition[pos]).length();
		}
	}

	vector<int> runningFieldAgents(world->getRunningFieldRobotsIdx());

	vector<int> bestPermutation(runningFieldAgents);
	double bestGlobalDist = 1000.0;
	do
	{
		double distTmp = 0;
		for (unsigned int i = 0; i < runningFieldAgents.size(); i++)
		{
			distTmp += distSP[i][runningFieldAgents[i]]
			                     * (1.0 + ((N_CAMBADAS - i) / 10));
		}
		if (distTmp < bestGlobalDist)
		{
			bestGlobalDist = distTmp;
			bestPermutation = vector<int>(runningFieldAgents);
		}
	} while (next_permutation(runningFieldAgents.begin(),
			runningFieldAgents.end()));

	//	cout << bestGlobalDist << endl;
	for (unsigned int i = 0; i < bestPermutation.size(); i++)
	{
		finfo.position[bestPermutation[i]] = SPosition[i];
		finfo.posId[bestPermutation[i]] = i;
		//		cout << bestPermutation[i] + 1 << ":" << SPosition[i].x << ":"
		//				<< SPosition[i].y << endl;
	}
}

void Strategy::exchange(vector<Vec> positions, int gready)
{
	double distSP[N_CAMBADAS][N_CAMBADAS]; // [pos][agente]
	char freeAgent[N_CAMBADAS];

	memset(freeAgent, 1, sizeof(char) * N_CAMBADAS);

	for (int agent = 0; agent < N_CAMBADAS; agent++)
	{
		finfo.position[agent] = Vec::zero_vector;
//		finfo.posId[agent] = -1;
	}

	for (int pos = 0; pos < N_CAMBADAS; pos++)
	{
		for (int agent = 0; agent < N_CAMBADAS; agent++)
		{
			if (world->robot[agent].role == rGoalie
					|| !world->robot[agent].running)
				distSP[pos][agent] = 1001.0;
			else
				distSP[pos][agent] =
						(world->robot[agent].pos - positions[pos]).length();
		}
	}

	vector<int> runningFieldAgents(world->getRunningFieldRobotsIdx());

	for (int pos = 0; pos < gready; pos++)
	{
		int minDistAgent = -1;
		double minDist = 1000.0;

		for (int agent = 0; agent < N_CAMBADAS; agent++)
		{
			if (distSP[pos][agent] < minDist + 1.0 && freeAgent[agent])//caution add 1.0 to minimize changes
			{
				minDist = distSP[pos][agent];
				minDistAgent = agent;
			}
		}

		if (minDistAgent != -1)
		{
			freeAgent[minDistAgent] = 0;
			finfo.position[minDistAgent] = positions[pos];
//			finfo.posId[minDistAgent] = world->getNumberOfRunningFieldRobots()-1 - pos;
			finfo.cover[minDistAgent] = true;
			for (unsigned i = 0; i < runningFieldAgents.size(); ++i)
			{
				if (runningFieldAgents.at(i) == minDistAgent)
				{
					runningFieldAgents.erase(runningFieldAgents.begin() + i);
					break;
				}
			}
		}
	}
	vector<int> bestPermutation(runningFieldAgents);
	double bestGlobalDist = 1000.0;
	do
	{
		double distTmp = 0;
		for (unsigned int i = 0; i < runningFieldAgents.size(); i++)
		{
			distTmp += distSP[i + gready][runningFieldAgents[i]]
			                     * (1.0 + ((N_CAMBADAS - i) / 10));
		}
		if (distTmp < bestGlobalDist)
		{
			bestGlobalDist = distTmp;
			bestPermutation = vector<int>(runningFieldAgents);
		}
	} while (next_permutation(runningFieldAgents.begin(),
			runningFieldAgents.end()));

	//	cout << bestGlobalDist << endl;
	for (unsigned int i = 0; i < bestPermutation.size(); i++)
	{
		finfo.position[bestPermutation[i]] = positions[i + gready];
//		finfo.posId[bestPermutation[i]] = i;
		finfo.cover[bestPermutation[i]] = false;
		//		cout << bestPermutation[i] + 1 << ":" << SPosition[i].x << ":"
		//				<< SPosition[i].y << endl;
	}
}

void Strategy::exchangeGreedy()
{
	double distSP[N_CAMBADAS][N_CAMBADAS]; // [pos][agente]
	char freeAgent[N_CAMBADAS];

	memset(freeAgent, 1, sizeof(char) * N_CAMBADAS);

	for (int agent = 0; agent < N_CAMBADAS; agent++)
	{
		finfo.position[agent] = Vec::zero_vector;
		finfo.posId[agent] = -1;
	}

	for (int pos = 0; pos < N_CAMBADAS; pos++)
	{
		for (int agent = 0; agent < N_CAMBADAS; agent++)
		{
			if (world->robot[agent].role == rGoalie
					|| !world->robot[agent].running)
				distSP[pos][agent] = 1001.0;
			else
				distSP[pos][agent] =
						(world->robot[agent].pos - SPosition[pos]).length();
		}
	}

	for( int pos = 0 ; pos < N_CAMBADAS; pos++ )
	{
		int minDistAgent = -1;
		double minDist = 1000.0;

		for( int agent = 0 ; agent < N_CAMBADAS ; agent++ )
		{
			if( distSP[pos][agent] < minDist && freeAgent[agent])
			{
				minDist = distSP[pos][agent];
				minDistAgent = agent;
			}
		}

		if( minDistAgent != -1 )
		{
			freeAgent[minDistAgent] = 0;
			finfo.position[minDistAgent] = SPosition[pos];
			finfo.posId[minDistAgent] = pos;
			//cout << minDistAgent << ":" << SPosition[pos].x << ":" << SPosition[pos].y << endl;
		}
	}
}

void Strategy::distanceRestrictions()
{
	int movedRight = 0, movedLeft = 0;
	for (int pos = 0; pos < N_CAMBADAS; pos++)
	{
		Vec res = SPosition[pos];
		Vec ball = world->getBestBallAbs(false);
		float distance = 3.2; // According to the rules

		if (world->gameState == dropBall)
			distance = 1.2;

		if (world->gameState == preOpponentKickOff)
		{
			ball = Vec(0.0,0.0);
		}

		Vec line = res - ball;
		bool specialCase = false;
		if (pos == 0)
		{
			float dist2Line = 0.0;
			Vec p1(-field->penaltyAreaHalfWidth + dist2Line,
					-field->halfLength - 4.0);
			Vec p2(p1.x,
					-field->halfLength + field->penaltyAreaLength - dist2Line);
			Vec p3(-p2.x, p2.y);
			Vec p4(p3.x, -field->halfLength - 4.0);

			LineSegment penaltyL(p1, p2);
			LineSegment penaltyF(p2, p3);
			LineSegment penaltyR(p3, p4);
			LineSegment ballMe = LineSegment(ball,
					ball + line.setLength(distance));

			vector<Vec> inter1 = intersect(ballMe, penaltyL);
			vector<Vec> inter2 = intersect(ballMe, penaltyF);
			vector<Vec> inter3 = intersect(ballMe, penaltyR);

			vector<Vec> allInter;
			unsigned int i = 0;
			for (i = 0; i < inter1.size(); i++)
			{
				allInter.push_back(inter1.at(i));
			}
			for (i = 0; i < inter2.size(); i++)
			{
				allInter.push_back(inter2.at(i));
			}
			for (i = 0; i < inter3.size(); i++)
			{
				allInter.push_back(inter3.at(i));
			}

			if (allInter.size() > 0) // If any intersection is found, then chose the closest to the ball
			{
				float minDist = 2014.0f;
				float distTemp; // calculated distance between points and ball
				Vec best;
				for (i = 0; i < allInter.size(); i++)
				{
					distTemp = (allInter.at(i) - ball).length();
					if (distTemp < minDist)
					{
						minDist = distTemp;
						best = allInter.at(i);
						specialCase = true;
					}
				}

				if (specialCase)
				{
					res = best;
				}
			}
		}
		else if (field->isNearOurPenaltyArea(
				(line.length()) <= distance ?
						(ball + line.setLength(distance)) : (ball + line), 0.3))
		{
			Arc ballDistance = Arc(ball, distance, Angle::deg_angle(91),
					Angle::deg_angle(89));

			float dist2Line = -0.3;
			Vec p1(-field->penaltyAreaHalfWidth + dist2Line,
					-field->halfLength - 4.0);
			Vec p2(p1.x,
					-field->halfLength + field->penaltyAreaLength - dist2Line);
			Vec p3(-p2.x, p2.y);
			Vec p4(p3.x, -field->halfLength - 4.0);

			LineSegment penaltyL(p1, p2);
			LineSegment penaltyF(p2, p3);
			LineSegment penaltyR(p3, p4);

			vector<Vec> inter1 = intersect(penaltyL, ballDistance);
			vector<Vec> inter2 = intersect(penaltyF, ballDistance);
			vector<Vec> inter3 = intersect(penaltyR, ballDistance);

			vector<Vec> allInter;
			unsigned int i = 0;
			for (i = 0; i < inter1.size(); i++)
			{
				allInter.push_back(inter1.at(i));
			}
			for (i = 0; i < inter2.size(); i++)
			{
				allInter.push_back(inter2.at(i));
			}
			for (i = 0; i < inter3.size(); i++)
			{
				allInter.push_back(inter3.at(i));
			}

			if (allInter.size() > 0) // If any intersection is found, then chose the closest to the targetPos
			{
				float minDist = 2014.0f;
				float distTemp; // calculated distance between points and SPosition
				Vec best;
				for (i = 0; i < allInter.size(); i++)
				{
					if (field->isInside(allInter.at(i), 0.3))
					{
						distTemp = (allInter.at(i) - SPosition[pos]).length();
						if (distTemp < minDist)
						{
							minDist = distTemp;
							best = allInter.at(i);
							specialCase = true;
						}
					}
				}
				if (specialCase)
				{
					if ((best - res).x > 0.0)
					{
						if (movedRight > 0)
						{
							best = intersect(ballDistance,
									LineSegment(ball,
											((best
													+ Vec(0.3 * movedRight,
															0.3 * movedRight))
															- ball).setLength(4.0)
															+ ball)).at(0);
						}
						movedRight++;
					}
					else
					{
						if (movedLeft > 0)
						{
							best = intersect(ballDistance,
									LineSegment(ball,
											((best
													+ Vec(-0.3 * movedLeft,
															0.3 * movedLeft))
															- ball).setLength(4.0)
															+ ball)).at(0);
						}
						movedLeft++;
					}
					res = best;
				}
			}
		}
		else if (!field->isInside(ball + line.setLength(distance), -0.5))
		{
			Arc ballDistance = Arc(ball, distance, Angle::deg_angle(91),
					Angle::deg_angle(89));

			float dist2Line = 0.5;
			Vec p1(-field->halfWidth + dist2Line,
					-field->halfLength + dist2Line);
			Vec p2(p1.mirror_y());
			Vec p3(p2.mirror_x());
			Vec p4(p1.mirror_x());

			LineSegment line1(p1, p2);
			LineSegment line2(p2, p3);
			LineSegment line3(p3, p4);
			LineSegment line4(p4, p1);

			vector<Vec> inter1 = intersect(line1, ballDistance);
			vector<Vec> inter2 = intersect(line2, ballDistance);
			vector<Vec> inter3 = intersect(line3, ballDistance);
			vector<Vec> inter4 = intersect(line4, ballDistance);

			vector<Vec> allInter;
			unsigned int i = 0;
			for (i = 0; i < inter1.size(); i++)
			{
				allInter.push_back(inter1.at(i));
			}
			for (i = 0; i < inter2.size(); i++)
			{
				allInter.push_back(inter2.at(i));
			}
			for (i = 0; i < inter3.size(); i++)
			{
				allInter.push_back(inter3.at(i));
			}
			for (i = 0; i < inter4.size(); i++)
			{
				allInter.push_back(inter4.at(i));
			}

			if (allInter.size() > 0) // If any intersection is found, then chose the closest to the targetPos
			{
				float minDist = 2014.0f;
				float distTemp; // calculated distance between points and SPosition
				Vec best;
				for (i = 0; i < allInter.size(); i++)
				{
					if (!field->isNearOurPenaltyArea(allInter.at(i), 0.3))
					{
						distTemp = (allInter.at(i) - SPosition[pos]).length();
						if (distTemp < minDist)
						{
							minDist = distTemp;
							best = allInter.at(i);
							specialCase = true;
						}
					}
				}

				if (specialCase)
				{
					res = best;
				}
			}
		}

		if (!specialCase && line.length() < distance)
			res = ball + line.setLength(distance);

		SPosition[pos] = res;
	}
}

void Strategy::minDisPositions(float distToRob)
{
	Vec ball = world->getBestBallAbs(false);

	float distanceToBall = 3.2; // According to the rules

	if (world->gameState == dropBall)
		distanceToBall = 1.2;
	if (world->gameState == freePlay)
		distanceToBall = 0.0;
	// The robots should not approach too much
	for (int id = 1; id < N_CAMBADAS; id++)
	{
		for (int i = id; i > 0; i--)
		{
			Vec colleaguePos = SPosition[i - 1];
			Vec robotsVec = SPosition[id] - colleaguePos;
			if(robotsVec == Vec::zero_vector)(colleaguePos-ball).setLength(0.1);
			if (robotsVec.length() < distToRob)
			{
				if (!field->isNearOurPenaltyArea(
						colleaguePos + robotsVec.setLength(distToRob), 0.3)
						&& ((colleaguePos + robotsVec.setLength(distToRob))
								- ball).length() > distanceToBall){
					SPosition[id] = colleaguePos
							+ robotsVec.setLength(distToRob);
				}
				else if (((colleaguePos
						+ (robotsVec.mirror_x().setLength(distToRob))) - ball).length()
						> distanceToBall && !field->isNearOurPenaltyArea(colleaguePos
								+ (robotsVec.mirror_x().setLength(distToRob)),0.3)){
					SPosition[id] = colleaguePos
							+ (robotsVec.mirror_x().setLength(distToRob));
				}
				else
				{
					Vec ballVec = SPosition[id] - ball;
					ballVec.y = 0.0;
					SPosition[id] = colleaguePos
							+ (ballVec.setLength(distToRob));
					field->ourPenaltyAreaFilter(SPosition[id],0.3);
				}

			}
		}
	}
}

Strategy::~Strategy()
{
	world = NULL;
	config = NULL;
	field = NULL;
}
}
