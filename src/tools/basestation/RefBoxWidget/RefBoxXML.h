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

#ifndef _CONFIGXML_H_
#define _CONFIGXML_H_

#include <libxml++/libxml++.h>
#include <iostream>
#include <map>

#include "DB_Robot_info.h"

using namespace xmlpp;
using namespace std;

class RefBoxXML : public SaxParser
{
	private:
		DB_Coach_Info db_coach_info; // a local copy 
#if 0
		map<string,PID> ctrlParam;
		map<string,float> parameter;
		map<string,int> field;
#endif
	public:
		RefBoxXML();
		~RefBoxXML();
	
		bool parse(string msg, DB_Coach_Info* cip) throw(xmlpp::exception);
	
		//map<string,PID> ctrlParam;
		//map<string,float> parameter;
		//map<string,int> field;
		
#if 0
		bool write(string fileName);
	
		void display();
#endif			
	protected:
		virtual void on_start_document() throw(xmlpp::exception);
		virtual void on_end_document() throw(xmlpp::exception);
		virtual void on_start_element(const string& name, const AttributeList& properties) throw(xmlpp::exception);
		virtual void on_end_element(const string& name) throw(xmlpp::exception);

 		virtual void on_fatal_error(const std::string& text);
 		virtual void on_error(const std::string& text);

	protected:
		int depth;
		int branch;
		bool retValue;
};

#endif //_CONFIGXML_H_

