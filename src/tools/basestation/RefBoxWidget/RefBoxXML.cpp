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

#include "RefBoxXML.h"
#include <syslog.h>
#include <assert.h>
#include <libxml++/exceptions/parse_error.h>

enum TagType { XML_ROOT=0, TAG_REFBOX_EVENT, TAG_REFEREE, TAG_GAME_INFO, TAG_TEAM_SETUP, TAG_TEAM_DATA, TAG_SETUP, XML_ERROR };


RefBoxXML::RefBoxXML( ) : SaxParser()
{
	depth = 0;
	branch = XML_ROOT;
	retValue = true;
}

RefBoxXML::~RefBoxXML()
{
}

bool RefBoxXML::parse(string msg, DB_Coach_Info* cip) throw(xmlpp::exception)
{
	depth = 0;
	branch = XML_ROOT;
	retValue = true;

	db_coach_info = *cip;

	try
	{
	//	set_substitute_entities(true);
		parse_memory(msg);
	}
//	catch(const std::exception& ex)
	catch(xmlpp::exception)
	{
		retValue = false;
	}
	
	if (retValue == true)
	{
		cip->print();
		db_coach_info.print();
		*cip = db_coach_info;
		cout << "Message OK\n";
	}
	else
	{
		cerr << "Message wrong\n";
	}

	return retValue;
}

void RefBoxXML::on_fatal_error(const std::string& text)
{
	cerr << "RefBoxXML::on_fatal_error" << endl;
}

void RefBoxXML::on_error(const std::string& text)
{
	cerr << "RefBoxXML::on_error" << endl;
	retValue = false;
	branch = XML_ERROR;
}

void RefBoxXML::on_start_document() throw (xmlpp::exception)
{
//	cout << "on_start_document()" << endl;
	depth = 0;
	branch = XML_ROOT;
	retValue = true;
}

void RefBoxXML::on_end_document() throw (xmlpp::exception)
{
//	cout << "on_end_document()" << endl;
}

void RefBoxXML::on_start_element(const string& name, const AttributeList& attributes) throw (xmlpp::exception)
{
	//cout << "tag: \"" << name << "\"" << endl;

	switch (branch)
	{
		case XML_ROOT:
		{
			if (name == "RefboxEvent")
			{
				branch = TAG_REFBOX_EVENT;
				depth++;
			}
			else
			{
				cerr << "Wrong tag: " << name << endl;
				retValue = false;
				branch = XML_ERROR;
				//throw new xmlpp::exception("Wrong tag");
			}
			break;
		}

		case XML_ERROR:
		{
			break;
		}

		case TAG_REFBOX_EVENT:
		{
			if (name == "Referee")
			{
				/** \todo process Referee attributes */
				branch = TAG_REFEREE;
				depth++;
			}
			else if (name == "GameInfo")
			{
				/** \todo process Referee attributes */
				branch = TAG_GAME_INFO;
				depth++;
			}
			else if (name == "TeamSetup")
			{
				/** \todo process Referee attributes */
				branch = TAG_TEAM_SETUP;
				depth++;
			}
			else
			{
				retValue = false;
				branch = XML_ERROR;
				cerr << "Wrong tag: " << name << " inside <RefboxEvent>" << endl;
				//throw new xmlpp::exception("Wrong tag 2");
			}
			break;
		}

		case TAG_REFEREE:
		{
			if (name == "StageChange")
			{
				/* action depends on attribute 'newStage' */
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name != "newStage")
					{
						cerr << "Invalid attribute \"" << attr->name << "\" in tag \"" << name << "\"\n";
						continue;
					}
					else
					{
						if (attr->value == "firstHalf")
						{
							db_coach_info.GameTime.start();
							db_coach_info.GamePart = 1;
							db_coach_info.Coach_Info.ourGoals=0;
							db_coach_info.Coach_Info.theirGoals=0;
						}
						else if (attr->value == "secondHalf")
						{
							db_coach_info.GameTime.start();
							db_coach_info.GamePart = 2;
						}
					}
				}
			}
			else if (name == "GameStart")
			{
				db_coach_info.Coach_Info.gameState = SIGstart;
			}
			else if (name == "GameStop")
			{
				db_coach_info.Coach_Info.gameState = SIGstop;
			}
			else if (name == "KickOff")
			{
				/* action depends on attribute 'team' */
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name != "team")
					{
						cerr << "Invalid attribute \"" << attr->name << "\" in tag \"" << name << "\"\n";
						continue;
					}
					else
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.gameState = SIGourKickOff;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirKickOff;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.gameState = SIGourKickOff;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirKickOff;
						}
					}
				}
			}
			else if (name == "FreeKick")
			{
				/* action depends on attribute 'team' */
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name != "team")
					{
						cerr << "Invalid attribute \"" << attr->name << "\" in tag \"" << name << "\"\n";
						continue;
					}
					else
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.gameState = SIGourFreeKick;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirFreeKick;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.gameState = SIGourFreeKick;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirFreeKick;
						}
					}
				}
			}
			else if (name == "GoalKick")
			{
				/* action depends on attribute 'team' */
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name != "team")
					{
						cerr << "Invalid attribute \"" << attr->name << "\" in tag \"" << name << "\"\n";
						continue;
					}
					else
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.gameState = SIGourGoalKick;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirGoalKick;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.gameState = SIGourGoalKick;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirGoalKick;
						}
					}
				}
			}
			else if (name == "ThrowIn")
			{
				/* action depends on attribute 'team' */
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name != "team")
					{
						cerr << "Invalid attribute \"" << attr->name << "\" in tag \"" << name << "\"\n";
						continue;
					}
					else
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.gameState = SIGourThrowIn;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirThrowIn;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.gameState = SIGourThrowIn;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirThrowIn;
						}
					}
				}
			}
			else if (name == "Corner")
			{
				/* action depends on attribute 'team' */
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name != "team")
					{
						cerr << "Invalid attribute \"" << attr->name << "\" in tag \"" << name << "\"\n";
						continue;
					}
					else
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.gameState = SIGourCornerKick;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirCornerKick;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.gameState = SIGourCornerKick;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirCornerKick;
						}
					}
				}
			}
			else if (name == "DroppedBall")
			{
				db_coach_info.Coach_Info.gameState = SIGdropBall;
			}
			else if (name == "GoalAwarded")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.ourGoals++;
							else
								db_coach_info.Coach_Info.theirGoals++;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.ourGoals++;
							else
								db_coach_info.Coach_Info.theirGoals++;
						}
					}
					else if (attr->name == "player")
					{
					}
					else if (attr->name == "own")
					{
					}
					else if (attr->name == "stage")
					{
					}
					else if (attr->name == "time")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "GoalRemoved")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.ourGoals--;
							else
								db_coach_info.Coach_Info.theirGoals--;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.ourGoals--;
							else
								db_coach_info.Coach_Info.theirGoals--;
						}
					}
					else if (attr->name == "player")
					{
					}
					else if (attr->name == "own")
					{
					}
					else if (attr->name == "stage")
					{
					}
					else if (attr->name == "time")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "Penalty")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
						if (attr->value == "Cyan")
						{
							if (db_coach_info.TeamColor == Cyan)
								db_coach_info.Coach_Info.gameState = SIGourPenalty;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirPenalty;
						}
						else if (attr->value == "Magenta")
						{
							if (db_coach_info.TeamColor == Magenta)
								db_coach_info.Coach_Info.gameState = SIGourPenalty;
							else
								db_coach_info.Coach_Info.gameState = SIGtheirPenalty;
						}
					}
					else if (attr->name == "goalColor")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "Substitution")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
					}
					else if (attr->name == "playerIn")
					{
					}
					else if (attr->name == "playerOut")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "Cancel")
			{
			}
			else if (name == "PlayerOut")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
					}
					else if (attr->name == "player")
					{
					}
					else if (attr->name == "reason")
					{
					}
					else if (attr->name == "minimumNrOfSecondsOfField")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "PlayerIn")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
					}
					else if (attr->name == "player")
					{
					}
					else if (attr->name == "reason")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "CardAwarded")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
					}
					else if (attr->name == "player")
					{
					}
					else if (attr->name == "color")
					{
					}
					else if (attr->name == "number")
					{
					}
					else if (attr->name == "time")
					{
					}
					else if (attr->name == "stage")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "CardRemoved")
			{
				for (AttributeList::const_iterator attr = attributes.begin(); attr != attributes.end(); attr++)
				{
					if (attr->name == "team")
					{
					}
					else if (attr->name == "player")
					{
					}
					else if (attr->name == "color")
					{
					}
					else if (attr->name == "number")
					{
					}
					else if (attr->name == "time")
					{
					}
					else if (attr->name == "stage")
					{
					}
					else
					{
					}
				}
			}
			else if (name == "Parking")
			{
				db_coach_info.Coach_Info.gameState = SIGparking;
			}
			else
			{
				cerr << "Wrong tag: " << name << " inside <RefboxEvent>" << endl;
				branch = XML_ERROR;
				retValue = false;
				//throw new xmlpp::exception("Wrong tag 3");
			}
			break;
		}

		case TAG_GAME_INFO:
		{
			/** \todo process tag <GameInfo ...> */
			break;
		}

		case TAG_TEAM_SETUP:
		{
			/** \todo process tag <TeamSetup ...> */
			break;
		}
	}

	return;
}	

void RefBoxXML::on_end_element(const string& name) throw (xmlpp::exception)
{
//	cout << "on_end_element()" << endl;
	//cout << "/tag: \"" << name << "\"" << endl;
}

