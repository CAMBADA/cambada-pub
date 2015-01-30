
%{

#include "log.h"
#include "StrategyParser.h"
#include "Formation.h"
#include <assert.h>
#include <stdlib.h>
#include <string>
#include <map>
#include <vector>
using namespace std;
	
#define MAX_FORMATION_PLAYERS	5
#define FORMATION_NAME_SIZE	32	
#define PATH_MAX_SIZE		1024	
	
#define YY_DECL int yylex( vector<Formation *>& form)

int yydebug=1;
	
%}

%option noyywrap
%option nounput
%option stack
%option noyy_top_state

%x	INITIAL2 
%x	FORMATION 
%x	POSITION

DIGIT	[0-9]
DECIMAL	\.{DIGIT}+
FLOAT	\-*{DIGIT}+{DECIMAL}*
ID		[a-z|_|-|A-Z|0-9]+
PATH		[.|/|a-z|_|-|A-Z|0-9]+
blanks	[[:blank:]]+
%%
	char name[FORMATION_NAME_SIZE];
	char filename[PATH_MAX_SIZE];
	static int nFormationPlayers=0;
	static int numberOfFormations=0;
	static FormationSBSP *formationSBSP;

<INITIAL>[0-9]+ {
				sscanf(yytext,"%d", &numberOfFormations); 
				//cerr << " numberOfFormations : " << numberOfFormations << endl; 
				yy_push_state(INITIAL2); 
			}

<INITIAL2>{blanks}*FORMATION{blanks} { 
				nFormationPlayers = 0; 
				yy_push_state(FORMATION); 
			}

<FORMATION>{ID}{blanks}*"\n"*\{{blanks}* { 
                                sscanf(yytext,"%s",name); 
                                
                                formationSBSP = new FormationSBSP();
                                formationSBSP->name = string(name); 
                        }

<FORMATION>[1-9]+{blanks}+{FLOAT}{blanks}+{FLOAT}{blanks}+{FLOAT}{blanks}{FLOAT}{blanks}{FLOAT}{blanks}{FLOAT}{blanks}{FLOAT}{blanks}{FLOAT} {
				int player;
				float f[8];
				sscanf(yytext,"%d %f %f %f %f %f %f %f %f", &player,
						&f[0],&f[1],&f[2],&f[3],&f[4],&f[5],&f[6],&f[7]);
				assert( player > 0 || player <= MAX_FORMATION_PLAYERS ); // return -4;

				formationSBSP->pos[player-1] = Vec(f[0],f[1]);
				formationSBSP->att[player-1] = Vec(f[2],f[3]);
				formationSBSP->min[player-1] = Vec(f[4],f[5]);
				formationSBSP->max[player-1] = Vec(f[6],f[7]);
			
				nFormationPlayers++;
			}	

<FORMATION>{blanks}*\}{blanks}* { 
				//cerr << "nFormationPlayers "<<nFormationPlayers<<"|"<<MAX_FORMATION_PLAYERS<<endl;
				//formationSBSP->dump(); 
				form.push_back(formationSBSP);
				assert( nFormationPlayers <= MAX_FORMATION_PLAYERS); 
				yy_pop_state(); 
			}

<*>[#|//].* { /*	printf("COMMENT LINE : %s\n",yytext);*/ }
<*>{blanks}
<*>\n   
<*><<EOF>>	{ 
				//cerr <<form.size()<<" N FORMATIONS PROCESSED : "<<numberOfFormationsProcessed << endl;
				assert( form.size() == numberOfFormations ); 
				yy_pop_state(); 
				return ( (YYSTATE == INITIAL)?0:-3); 
			}

<*>.		{ printf("ERROR IN : '%s'\n",yytext); return -2; }
%%

int parse(char* fileName, vector<Formation *>& form)
{
	yyin = fopen(fileName,"r");
	
	if( yyin == NULL )
	{	
		return -1;
	}
	else
	{
		form.clear();
		int result = yylex(form);
		return result;
	}
}



