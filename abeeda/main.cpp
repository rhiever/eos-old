#include <stdio.h>
#include <stdlib.h>
#include <sstream>
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

using namespace std;

//double replacementRate = 0.1;
double  perSiteMutationRate = 0.005;
int     populationSize = 100;
int     totalGenerations = 252;
bool    make_video = false;
int     make_video_frequency = 25;
bool    display_only = false;
bool    track_best_brains = false;
int     track_best_brains_frequency = 25;

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
	vector<tAgent*> swarmAgents, SANextGen, predatorAgents, PANextGen;
	tAgent *swarmAgent, *predatorAgent, *bestSwarmAgent, *bestPredatorAgent;
	tGame *game;
	double swarmMaxFitness = 0.0, predatorMaxFitness = 0.0;
    FILE *LOD, *swarmGenomeFile, *predatorGenomeFile;
    string swarmGenomeFileName = "", predatorGenomeFileName = "";
    
    // initial object setup
    swarmAgents.resize(populationSize);
    predatorAgents.resize(populationSize);
	game = new tGame;
	swarmAgent = new tAgent;
    predatorAgent = new tAgent;
    
    // time-based seed by default. can change with command-line parameter.
    srand((unsigned int)time(NULL));
    
    for (int i = 1; i < argc; ++i)
    {
        // -d [filename] [filename]: display
        if (strcmp(argv[i], "-d") == 0 && (i + 1) < argc)
        {
            ++i;
            swarmAgent->loadAgent(argv[i]);
            
            ++i;
            predatorAgent->loadAgent(argv[i]);
            
            display_only = make_video = true;
        }
        
        // -e [filename] [filename] [filename]: evolve
        else if (strcmp(argv[i], "-e") == 0 && (i + 2) < argc)
        {
            ++i;
            LOD = fopen(argv[i], "w");
            ++i;
            stringstream sgfn;
            sgfn << argv[i];
            swarmGenomeFileName = sgfn.str();
            ++i;
            stringstream pgfn;
            pgfn << argv[i];
            predatorGenomeFileName = pgfn.str();
        }
        
        // -s [int]: seed
        else if (strcmp(argv[i], "-s") == 0 && (i + 1) < argc)
        {
            ++i;
            srand(atoi(argv[i]));
        }
        
        // -g [int]: generations
        else if (strcmp(argv[i], "-g") == 0 && (i + 1) < argc)
        {
            ++i;
            totalGenerations = atoi(argv[i]);
            
            if (totalGenerations < 3)
            {
                cout << "Minimum number of generations permitted is 3." << endl;
                exit(0);
            }
        }
        
        // -t [int]: track best brains
        else if (strcmp(argv[i], "-t") == 0 && (i + 1) < argc)
        {
            track_best_brains = true;
            ++i;
            track_best_brains_frequency = atoi(argv[i]);
            
            if (track_best_brains_frequency < 1)
            {
                cout << "Minimum brain tracking frequency is 1." << endl;
                exit(0);
            }
        }
        
        // -v [int]: make video
        else if (strcmp(argv[i], "-v") == 0 && (i + 1) < argc)
        {
            make_video = true;
            ++i;
            make_video_frequency = atoi(argv[i]);
            
            if (make_video_frequency < 1)
            {
                cout << "Minimum video making frequency is 1." << endl;
                exit(0);
            }
        }
    }
    
    if(make_video)
    {
        // start monitor first, then abeeda
        setupBroadcast();
    }
    
    if (display_only)
    {
        string reportString = game->executeGame(swarmAgent, predatorAgent, NULL, true);
        reportString.append("X");
        doBroadcast(reportString);
        exit(0);
    }
    
    // seed the agents
    delete swarmAgent;
    swarmAgent = new tAgent;
    swarmAgent->setupRandomAgent(5000);
    //swarmAgent->loadAgent("startSwarm.genome");
    
    delete predatorAgent;
    predatorAgent = new tAgent;
    //predatorAgent->setupRandomAgent(5000);
    predatorAgent->loadAgent("startPredator.genome");
    
    swarmAgents[0] = swarmAgent;
    predatorAgents[0] = predatorAgent;
    
    // make mutated copies of the start genome to fill up the initial population
	for(int i = 1; i < populationSize; ++i)
    {
		swarmAgents[i] = new tAgent;
		swarmAgents[i]->inherit(swarmAgent, 0.01, 0);
        
        predatorAgents[i] = new tAgent;
		predatorAgents[i]->inherit(predatorAgent, 0.01, 0);
    }
    
	SANextGen.resize(populationSize);
    PANextGen.resize(populationSize);
    
	swarmAgent->nrPointingAtMe--;
    predatorAgent->nrPointingAtMe--;
    
	cout << "setup complete" << endl;
    
    // main loop
	for (int update = 1; update <= totalGenerations; ++update)
    {
        // reset fitnesses
		for(int i = 0; i < populationSize; ++i)
        {
			swarmAgents[i]->fitness = 0.0;
			//swarmAgents[i]->fitnesses.clear();
            
            predatorAgents[i]->fitness = 0.0;
			//predatorAgents[i]->fitnesses.clear();
		}
        
        // determine fitness of population
		swarmMaxFitness = 0.0;
        predatorMaxFitness = 0.0;
        double swarmAvgFitness = 0.0;
        double predatorAvgFitness = 0.0;
        
		for(int i = 0; i < populationSize; ++i)
        {
            game->executeGame(swarmAgents[i], predatorAgents[i], NULL, false);
            
            // store the swarm agent's corresponding predator agent
            swarmAgents[i]->predator = new tAgent;
            swarmAgents[i]->predator->inherit(predatorAgents[i], 0.0, predatorAgents[i]->born);
            
            swarmAvgFitness += swarmAgents[i]->fitness;
            predatorAvgFitness += predatorAgents[i]->fitness;
            
            //swarmAgents[i]->fitnesses.push_back(swarmAgents[i]->fitness);
            //predatorAgents[i]->fitnesses.push_back(predatorAgents[i]->fitness);
            
            if(swarmAgents[i]->fitness > swarmMaxFitness)
            {
                swarmMaxFitness = swarmAgents[i]->fitness;
                bestSwarmAgent = swarmAgents[i];
            }
            
            if(predatorAgents[i]->fitness > predatorMaxFitness)
            {
                predatorMaxFitness = predatorAgents[i]->fitness;
                bestPredatorAgent = predatorAgents[i];
            }
		}
        
        swarmAvgFitness /= (double)populationSize;
        predatorAvgFitness /= (double)populationSize;
		
		cout << "generation " << update << ": swarm [" << (int)swarmAvgFitness << " : " << (int)swarmMaxFitness << "] :: predator [" << (int)predatorAvgFitness << " : " << (int)predatorMaxFitness << "]" << endl;
        
        // display video of simulation
        if (make_video)
        {
            bool finalGeneration = update == totalGenerations;
            
            if (update % make_video_frequency == 0 || finalGeneration)
            {
                string reportString = "", bestString = "";
                double bestFitness = 0.0;
                
                // repeatedly face the two brains against each other to account for stochasticity
                for (int i = 0; i < 100; ++i)
                {
                    reportString = game->executeGame(bestSwarmAgent, bestPredatorAgent, NULL, make_video);
                    
                    // interested in seeing the best swarms
                    if (bestSwarmAgent->fitness > bestFitness)
                    {
                        bestFitness = bestSwarmAgent->fitness;
                        bestString = reportString;
                    }
                }
                
                if (finalGeneration)
                {
                    bestString.append("X");
                }
                
                doBroadcast(bestString);
            }
        }
        
        // construct swarm agent population for the next generation
		for(int i = 0; i < populationSize; ++i)
		{
			tAgent *d = new tAgent;
            int j = 0;
            
			do
            {
                j = rand() % populationSize;
            } while((j == i) || (randDouble > (swarmAgents[j]->fitness / swarmMaxFitness)));
            
			d->inherit(swarmAgents[j], perSiteMutationRate, update);
			SANextGen[i] = d;
		}
        
        // construct predator agent population for the next generation
        for(int i = 0; i < populationSize; ++i)
		{
			tAgent *d = new tAgent;
            int j = 0;
            
			do
            {
                j = rand() % populationSize;
            } while((j == i) || (randDouble > (predatorAgents[j]->fitness / predatorMaxFitness)));
            
			d->inherit(predatorAgents[j], perSiteMutationRate, update);
			PANextGen[i] = d;
		}
        
        // shuffle the populations so there is a small chance of the same predator/prey combo in the next generation
        random_shuffle(SANextGen.begin(), SANextGen.end());
        random_shuffle(PANextGen.begin(), PANextGen.end());
        
        // retire and replace the swarm agents from the previous generation
		for(int i = 0; i < populationSize; ++i)
        {
			swarmAgents[i]->retire();
			swarmAgents[i]->nrPointingAtMe--;
			if(swarmAgents[i]->nrPointingAtMe == 0)
            {
				delete swarmAgents[i];
            }
			swarmAgents[i] = SANextGen[i];
		}
        
		swarmAgents = SANextGen;
        
        // retire and replace the predator agents from the previous generation
		for(int i = 0; i < populationSize; ++i)
        {
			predatorAgents[i]->retire();
			predatorAgents[i]->nrPointingAtMe--;
			if(predatorAgents[i]->nrPointingAtMe == 0)
            {
				delete predatorAgents[i];
            }
			predatorAgents[i] = PANextGen[i];
		}
        
		predatorAgents = PANextGen;
        
        if (track_best_brains && update > 2 && update % 25 == 0)
        {
            std::stringstream sss, pss;
            
            sss << "swarm" << update << ".genome";
            pss << "predator" << update << ".genome";
            
            swarmGenomeFile = fopen(sss.str().c_str(), "w");
            predatorGenomeFile = fopen(pss.str().c_str(), "w");
            
            swarmAgents[0]->ancestor->ancestor->saveGenome(swarmGenomeFile);
            predatorAgents[0]->ancestor->ancestor->saveGenome(predatorGenomeFile);
            
            fclose(swarmGenomeFile);
            fclose(predatorGenomeFile);
        }
	}
	
    // save the genome file of the lmrca
    swarmGenomeFile = fopen(swarmGenomeFileName.c_str(), "w");
    predatorGenomeFile = fopen(predatorGenomeFileName.c_str(), "w");
    
	swarmAgents[0]->ancestor->ancestor->saveGenome(swarmGenomeFile);
    predatorAgents[0]->ancestor->ancestor->saveGenome(predatorGenomeFile);
    
    fclose(swarmGenomeFile);
    fclose(predatorGenomeFile);
    
    // save LOD
    /*vector<tAgent*> saveLOD;
    
    tAgent* curAncestor = swarmAgents[0]->ancestor->ancestor;
    
    while (curAncestor != NULL)
    {
        saveLOD.insert(saveLOD.begin(), curAncestor);
        
        curAncestor = curAncestor->ancestor;
    }
    
    for (vector<tAgent*>::iterator it = saveLOD.begin(); it != saveLOD.end(); ++it)
    {
        if ((*it)->predator == NULL)
        {
            cout << "NULL predator at " << (*it)->born << endl;
        }
        else
        {
            game->executeGame(*it, (*it)->predator, LOD, make_video);
        }
    }*/
    
    fclose(LOD);
    
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
