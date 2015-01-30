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

#include "ConfigXML.h"
#include <syslog.h>
#include <log.h>
#include <assert.h>

using namespace std;
using namespace cambada;

namespace cambada {
namespace util {

ConfigXML::ConfigXML( )
{
	retValue = true;
}

ConfigXML::~ConfigXML()
{
}

bool ConfigXML::parse(string fileName)
{
	auto_ptr<CambadaConfig> cambadaConf;
	try
	{
		cambadaConf = CambadaConfig_(fileName);
	}
	catch(const exception& ex)
//	catch(...)
	{
		cerr << "ConfigXML exception ... " << ex.what() << endl;
		return false;
	}

	for (unsigned int i = 0; i < cambadaConf->Field().size(); ++i)
	{
		field[cambadaConf->Field()[i].name()] = cambadaConf->Field()[i].value();
	}

	for (unsigned int i = 0; i < cambadaConf->CtrlParam().size(); ++i)
	{
		PID tmp;
		tmp.setP(cambadaConf->CtrlParam()[i].p());
		tmp.setI(cambadaConf->CtrlParam()[i].i());
		tmp.setD(cambadaConf->CtrlParam()[i].d());
		tmp.setMaxInt(cambadaConf->CtrlParam()[i].maxInt());
		tmp.setMaxOut(cambadaConf->CtrlParam()[i].maxOut());
		tmp.setMinOut(cambadaConf->CtrlParam()[i].minOut());

		ctrlParam[cambadaConf->CtrlParam()[i].name()] = tmp;
	}

	for (unsigned int i = 0; i < cambadaConf->Parameter().size(); ++i)
	{
		parameter[cambadaConf->Parameter()[i].name()].value = cambadaConf->Parameter()[i].value();
		
		std::string cmt = cambadaConf->Parameter()[i].comment();
		//fprintf(stderr,"Comment: %s\n",cmt.c_str());
		
		parameter[cambadaConf->Parameter()[i].name()].comment = cmt;
	}

	return retValue;
}

PID& ConfigXML::getCtrlParam(string name)
{
	if( ctrlParam.count(name) == 0 )
	{
		syslog(LOG_ERR,"ConfigXML (getCtrlParam) %s\n",name.data());
		myprintf("ConfigXML (getCtrlParam) %s\n",name.c_str());
		assert( ctrlParam.count(name) != 0 );
	}

	return ctrlParam[name];
}

float ConfigXML::getParam(string name)
{
	if( parameter.count(name) == 0 )
	{
		syslog(LOG_ERR,"ConfigXML (getParam) %s",name.data() );	
		assert( parameter.count(name) != 0 );
	}

	return parameter[name].value;
}
	
int ConfigXML::getField(string name)
{
	if( field.count(name) == 0 )
	{
		syslog(LOG_ERR,"ConfigXML (getField) %s",name.data());	
		assert( field.count(name) != 0 );
	}
	
	return field[name];
}

map<string,PID>::iterator ConfigXML::getCtrlParamMapBegin()
{
	return ctrlParam.begin();
}

map<string,PID>::iterator ConfigXML::getCtrlParamMapEnd()
{
	return ctrlParam.end();
}

map<string,Param>::iterator ConfigXML::getParamMapBegin()
{
	return parameter.begin();
}

map<string,Param>::iterator ConfigXML::getParamMapEnd()
{
	return parameter.end();
}

map<string,int>::iterator ConfigXML::getFieldMapBegin()
{
	return field.begin();
}

map<string,int>::iterator ConfigXML::getFieldMapEnd()
{
	return field.end();
}

bool ConfigXML::addCtrlParam(string name)
{
	PID newPID;

	newPID.setP(0);
	newPID.setI(0);
	newPID.setD(0);
	newPID.setMinOut(0);
	newPID.setMaxOut(0);
	newPID.setMaxInt(0);

	ctrlParam.insert(pair<string,PID>(name,newPID));

	if(ctrlParam.count(name)!=0)
		return true;
	else
		return false;
}

bool ConfigXML::removeCtrlParam(string name)
{
	if(ctrlParam.count(name)!=0)
		ctrlParam.erase(name);

	return true;
}

bool ConfigXML::addParam(string name, Param *par)
{
	parameter.insert(pair<string,Param>(name,*par));

	if(parameter.count(name)!=0)
		return true;
	else
		return false;
}

bool ConfigXML::addParam(string name, float val)
{
	parameter.insert(pair<string,Param>(name, Param(val)));

	if(parameter.count(name)!=0)
		return true;
	else
		return false;
}

bool ConfigXML::removeParam(string name)
{
	if(parameter.count(name)!=0)
		parameter.erase(name);

	return true;
}

bool ConfigXML::addField(string name, int value)
{
	field.insert(pair<string,int>(name,value));

	if(field.count(name)!=0)
		return true;
	else
		return false;
}

bool ConfigXML::removeField(string name)
{
	if(field.count(name)!=0)
		field.erase(name);

	return true;
}

bool ConfigXML::existCtrlParam(string name)
{
	if(ctrlParam.count(name)==0)
		return false;
	else
		return true;
}

bool ConfigXML::existParam(string name)
{
	if(parameter.count(name)==0)
		return false;
	else
		return true;
}

bool ConfigXML::existField(string name)
{
	if(field.count(name)==0)
		return false;
	else
		return true;
}

bool ConfigXML::updateParam(string name, Param *par)
{
	if(existParam(name))
	{
		parameter[name]=*par;
		return true;
	}
	else
		return false;
}

bool ConfigXML::updateParam(string name, float val)
{
	if(existParam(name))
	{
		Param *p = new Param(val);
		parameter[name]=*p;
		return true;
	}
	else
		return false;
}

bool ConfigXML::updateField(string name, int value)
{
	if(existField(name))
	{
		field[name]=value;
		return true;
	}
	else
		return false;
}

bool ConfigXML::write(string fileName)	
{
	cout << "{ConfigXML] : file name " << fileName <<endl;
	cout << "{ConfigXML] : ctrlParam size " << ctrlParam.size() <<endl;
	cout << "{ConfigXML] : field size " << field.size() <<endl;
	cout << "{ConfigXML] : parameter size " << parameter.size() <<endl;
	FILE* fp = fopen(fileName.c_str(),"w");
	//FILE* fp = fopen("tmp.xml","w");
	
	if( fp == NULL )
		return false;
	
	fprintf(fp,"<CambadaConfig version=\"1.0\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:noNamespaceSchemaLocation=\"cambada.conf.xsd\">\n\n\n");
	
	for( map<string,int>::iterator it = field.begin(); it != field.end() ; it++)
		fprintf(fp,"\t<Field name=\"%s\" value=\"%d\"/>\n", it->first.c_str() , it->second);
	
	fprintf(fp,"\n\n");
	
	for( map<string,Param>::iterator it = parameter.begin(); it != parameter.end() ; it++)
		fprintf(fp,"\t<Parameter name=\"%s\" value=\"%f\" comment=\"%s\"/>\n", it->first.c_str() , it->second.value, it->second.comment.c_str());
			
	fprintf(fp,"\n\n");
	
	for( map<string,PID>::iterator it = ctrlParam.begin(); it != ctrlParam.end() ; it++)
		fprintf(fp,"\t<CtrlParam name=\"%s\" p=\"%f\" i=\"%f\" d=\"%f\" maxOut=\"%f\" maxInt=\"%f\" minOut=\"%f\"/>\n",
			it->first.c_str() , it->second.getP(), it->second.getI(), it->second.getD(), it->second.getMaxOut(), 
					it->second.getMaxInt(), it->second.getMinOut() );
				
	fprintf(fp,"\n</CambadaConfig>\n");
		
	fclose(fp);
	
			
	return true;
}

void ConfigXML::display()
{
	cout << "{ConfigXML] : ctrlParam size " << ctrlParam.size() <<endl;
	cout << "{ConfigXML] : field size " << field.size() <<endl;
	cout << "{ConfigXML] : parameter size " << parameter.size() <<endl;
	
	printf("<CambadaConfig version=\"1.0\">\n\n\n");
	
	for( map<string,int>::iterator it = field.begin(); it != field.end() ; it++)
		printf("\t<Field name=\"%s\" value=\"%d\"/>\n", it->first.c_str() , it->second);
	
	printf("\n\n");
	
	for( map<string,Param>::iterator it = parameter.begin(); it != parameter.end() ; it++)
		printf("\t<Parameter name=\"%s\" value=\"%f\" comment=\"%s\"/>\n", it->first.c_str() , it->second.value, it->second.comment.c_str());
			
	printf("\n\n");
	
	for( map<string,PID>::iterator it = ctrlParam.begin(); it != ctrlParam.end() ; it++)
		printf("\t<CtrlParam name=\"%s\" p=\"%f\" i=\"%f\" d=\"%f\" maxOut=\"%f\" maxInt=\"%f\"/>\n",
			it->first.c_str() , it->second.getP(), it->second.getI(), it->second.getD(), it->second.getMaxOut(), 
					it->second.getMaxInt() );
				
	printf("\n</CambadaConfig>\n");
		
}

void ConfigXML::checkConpensators()
{
	getCtrlParam("compensateR").check();
	getCtrlParam("compensateX").check();
	getCtrlParam("compensateY").check();
	getCtrlParam("compensateTrans").check();
}

}} // namespaces
