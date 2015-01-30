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

#ifndef _CONFIGXML_H_
#define _CONFIGXML_H_

#include <iostream>
#include <map>
#include <vector>

#include "cambada.conf.hxx"
#include "PID.h"
#include "Param.h"
#include "Vec.h"


namespace cambada {
namespace util {

using namespace std;

class ConfigXML
{
	private:
        map<string,PID> ctrlParam;
        map<string,Param> parameter;
		map<string,int> field;
	
	public:
		ConfigXML();
		~ConfigXML();
	
		bool parse(string fileName="cambada.xml");
		
		
		
        PID& getCtrlParam(string name);
		float getParam(string name);
		int getField(string name);
		
        map<string,PID>::iterator getCtrlParamMapBegin();
        map<string,PID>::iterator getCtrlParamMapEnd();
		bool addCtrlParam(string name);
		bool removeCtrlParam(string name);
		bool existCtrlParam(string name);

        map<string,Param>::iterator getParamMapBegin();
        map<string,Param>::iterator getParamMapEnd();
		bool addParam(string name,Param *par);
		bool addParam(string name,float val); // Retro-compatibility
		bool removeParam(string name);
		bool existParam(string name);
		
        bool updateParam(string name,Param *par);
		bool updateParam(string name, float val); // Retro-compatibility

		map<string,int>::iterator getFieldMapBegin();
		map<string,int>::iterator getFieldMapEnd();
		bool addField(string name, int value);
		bool removeField(string name);
		bool existField(string name);
		bool updateField(string name, int value);

		bool write(string fileName);
	
		void display();

		void checkConpensators();

	protected:
		bool retValue;
};
}}
#endif //_CONFIGXML2_H_

