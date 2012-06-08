#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <iostream>
#include "globalConst.h"
#include "tHMM.h"
#include "tAgent.h"
#include "tGame.h"

#define randDouble ((double)rand()/(double)RAND_MAX)

using namespace std;

//double replacementRate=0.1;
double perSiteMutationRate = 0.005;
int update = 0;
int repeats = 1;
int maxAgent = 100;
int totalGenerations = 252;//1002;
bool make_video = false;
bool display_only = false;

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */

#include "helper.h"           /*  our own helper functions  */

#define ECHO_PORT          (2002)
#define MAX_LINE           (100000)

#define BUFLEN  512
#define NPACK   10
#define PORT    9930

int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
short int port;                  /*  port number               */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */

void setupBroadcast(void);
void doBroadcast(string data);

int main(int argc, char *argv[])
{
	vector<tAgent*>agent;
	vector<tAgent*>nextGen;
	tAgent *masterAgent;
	tGame *game;
	double maxFitness = 0.0, thresholdMaxFitness = 0.0;
    string reportString, bestString;
    FILE *LOD;
    FILE *genomeFile;
    
    agent.resize(maxAgent);
	game = new tGame;
	masterAgent = new tAgent;
    
    // time-based seed by default. can change with command-line parameter.
    srand((unsigned int)time(NULL));
    
    for (int i = 1; i < argc; ++i)
    {
        // -d [filename]: display
        if (strcmp(argv[i], "-d") == 0 && (i + 1) < argc)
        {
            ++i;
            masterAgent->loadAgent(argv[i]);
            
            display_only = make_video = true;
        }
        
        // -e [filename] [filename]: evolve
        else if (strcmp(argv[i], "-e") == 0 && (i + 2) < argc)
        {
            ++i;
            LOD = fopen(argv[i], "w");
            ++i;
            genomeFile = fopen(argv[i], "w");
        }
        
        // -s [long]: seed
        else if (strcmp(argv[i], "-s") == 0 && (i + 1) < argc)
        {
            ++i;
            srand(atoi(argv[i]));
        }
        
        // -g [long]: generations
        else if (strcmp(argv[i], "-g") == 0 && (i + 1) < argc)
        {
            ++i;
            totalGenerations = atoi(argv[i]);
        }
        
        // -v: make video
        else if (strcmp(argv[i], "-v") == 0)
        {
            make_video = true;
        }
    }
    
    if(make_video)
    {
        // start monitor first, then abeeda
        setupBroadcast();
    }
    
    if (display_only)
    {
        reportString = game->executeGame(masterAgent, NULL, true);
        reportString.append("X");
        doBroadcast(reportString);
        exit(0);
    }
    
    masterAgent = new tAgent;
    masterAgent->setupRandomAgent(5000);
    
    // seeds simulation with a specific start organism
    //masterAgent->loadAgent("startOrganism.txt");

    // save start organism to file
    masterAgent->saveGenome(genomeFile);
    //masterAgent->saveToDot((char*)"test.dot");
    
	for(int i = 0; i < agent.size(); i++)
    {
		agent[i]=new tAgent;
		agent[i]->inherit(masterAgent, 0.01, 0);
	}
    
	nextGen.resize(agent.size());
	masterAgent->nrPointingAtMe--;
	cout<<"setup complete"<<endl;
    
	while(update < totalGenerations)
    {
		for(int i = 0; i < agent.size(); i++)
        {
			agent[i]->fitness=0.0;
			agent[i]->fitnesses.clear();
		}
        
		maxFitness = 0.0;
        
		for(int i = 0; i < agent.size(); i++)
        {
			for(int j = 0; j < repeats; j++)
            {
                reportString = game->executeGame(agent[i], NULL, make_video);
 				agent[i]->fitnesses.push_back(agent[i]->fitness);
                
                if(agent[i]->fitness > maxFitness)
                {
                    bestString=reportString;
                    maxFitness=agent[i]->fitness;
                }
			}
		}
		
		maxFitness = 0.0;
		for(int i = 0; i < agent.size(); i++)
        {
			agent[i]->fitness=agent[i]->fitnesses[0];
			if(agent[i]->fitness>maxFitness)
            {
				maxFitness=agent[i]->fitness;
            }
		}
		cout << "update " << update << ": " << (double)maxFitness << endl;
        
        if(make_video)
        {
            if(maxFitness > thresholdMaxFitness)
            {
                thresholdMaxFitness = maxFitness;
                
                cout << "new threshold: " << thresholdMaxFitness << endl;
                
                doBroadcast(bestString);
            }
            else if (update == totalGenerations - 1)
            {
                bestString.append("X");
                doBroadcast(bestString);
            }
        }
        
		for(int i = 0; i < agent.size(); i++)
		{
			tAgent *d = new tAgent;
            int j = 0;
            
			do
            {
                j = rand() % (int)agent.size();
            } while((j == i) || (randDouble > (agent[j]->fitness / maxFitness)));
            
			d->inherit(agent[j], perSiteMutationRate, update);
			nextGen[i] = d;
		}
        
		for(int i = 0; i < agent.size(); i++)
        {
			agent[i]->retire();
			agent[i]->nrPointingAtMe--;
			if(agent[i]->nrPointingAtMe == 0)
            {
				delete agent[i];
            }
			agent[i] = nextGen[i];
		}
        
		agent = nextGen;
		update++;
	}
	
    // save the genome file of the lmrca
	agent[0]->ancestor->ancestor->saveGenome(genomeFile);
    
    // save LOD
    vector<tAgent*> saveLOD;
    
    tAgent* curAncestor = agent[0]->ancestor->ancestor;
    
    while (curAncestor != NULL)
    {
        saveLOD.insert(saveLOD.begin(), curAncestor);
        
        curAncestor = curAncestor->ancestor;
    }
    
    for (vector<tAgent*>::iterator it = saveLOD.begin(); it != saveLOD.end(); ++it)
    {
        game->executeGame(*it, LOD, make_video);
    }
    
    fclose(LOD);
    fclose(genomeFile);
    
    return 0;
}

void setupBroadcast(void)
{
    port = ECHO_PORT;
	if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
    {
		fprintf(stderr, "ECHOSERV: Error creating listening socket.\n");
    }
	/*  Set all bytes in socket address structure to
	 zero, and fill in the relevant data members   */
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(port);
	/*  Bind our socket addresss to the 
	 listening socket, and call listen()  */
    if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 )
    {
		fprintf(stderr, "ECHOSERV: Error calling bind()\n");
    }
    if ( listen(list_s, LISTENQ) < 0 )
    {
		fprintf(stderr, "ECHOSERV: Error calling listen()\n");
    }
}

void doBroadcast(string data)
{
    if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 )
    {
        fprintf(stderr, "ECHOSERV: Error calling accept()\n");
    }
    Writeline(conn_s, data.c_str(), data.length());
    
    if ( close(conn_s) < 0 )
    {
        fprintf(stderr, "ECHOSERV: Error calling close()\n");
    }
}
