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

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */

#include "helper.h"           /*  our own helper functions  */

#define ECHO_PORT          (2002)
#define MAX_LINE           (100000)

#define BUFLEN              512
#define NPACK               10
#define PORT                9930

int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
short int port;                  /*  port number               */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */

void setupBroadcast(void);
void doBroadcast(string data);

using namespace std;

//double replacementRate = 0.1;
double  perSiteMutationRate         = 0.005;
int     populationSize              = 100;
int     totalGenerations            = 252;
bool    make_interval_video         = false;
int     make_video_frequency        = 25;
bool    make_LOD_video              = false;
bool    track_best_brains           = false;
int     track_best_brains_frequency = 25;

int main(int argc, char *argv[])
{
	vector<tAgent*> swarmAgents, SANextGen, predatorAgents, PANextGen;
	tAgent *swarmAgent = NULL, *predatorAgent = NULL, *bestSwarmAgent = NULL, *bestPredatorAgent = NULL;
	tGame *game = NULL;
	double swarmMaxFitness = 0.0, predatorMaxFitness = 0.0;
    string LODFileName = "", swarmGenomeFileName = "", predatorGenomeFileName = "", inputGenomeFileName = "";
    
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
        // -d [in file name] [in file name]: display
        if (strcmp(argv[i], "-d") == 0 && (i + 2) < argc)
        {
            ++i;
            swarmAgent->loadAgent(argv[i]);
            
            ++i;
            predatorAgent->loadAgent(argv[i]);

            setupBroadcast();
            
            string reportString = game->executeGame(swarmAgent, predatorAgent, NULL, true);
            reportString.append("X");
            doBroadcast(reportString);
            exit(0);
        }
        
        // -e [out file name] [out file name] [out file name]: evolve
        else if (strcmp(argv[i], "-e") == 0 && (i + 3) < argc)
        {
            ++i;
            stringstream lodfn;
            lodfn << argv[i];
            LODFileName = lodfn.str();
            
            ++i;
            stringstream sgfn;
            sgfn << argv[i];
            swarmGenomeFileName = sgfn.str();
            
            ++i;
            stringstream pgfn;
            pgfn << argv[i];
            predatorGenomeFileName = pgfn.str();
        }
        
        // -s [int]: set seed
        else if (strcmp(argv[i], "-s") == 0 && (i + 1) < argc)
        {
            ++i;
            srand(atoi(argv[i]));
        }
        
        // -g [int]: set generations
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
        
        // -v [int]: make video of best brains at an interval
        else if (strcmp(argv[i], "-v") == 0 && (i + 1) < argc)
        {
            make_interval_video = true;
            ++i;
            make_video_frequency = atoi(argv[i]);
            
            if (make_video_frequency < 1)
            {
                cout << "Minimum video creation frequency is 1." << endl;
                exit(0);
            }
        }
        
        // -lv: make video of LOD of best agent brain at the end
        else if (strcmp(argv[i], "-lv") == 0)
        {
            make_LOD_video = true;
        }
        
        // -lt [in file name] [out file name]: create logic table for given genome
        else if (strcmp(argv[i], "-lt") == 0 && (i + 2) < argc)
        {
            ++i;
            swarmAgent->loadAgent(argv[i]);
            ++i;
            swarmAgent->saveLogicTable(argv[i]);
            exit(0);
        }
        
        // -dfs [in file name] [out file name]: create dot image file for given swarm genome
        else if (strcmp(argv[i], "-dfs") == 0 && (i + 2) < argc)
        {
            ++i;
            swarmAgent->loadAgent(argv[i]);
            ++i;
            swarmAgent->saveToDot(argv[i], false);
            exit(0);
        }
        
        // -dfp [in file name] [out file name]: create dot image file for given predator genome
        else if (strcmp(argv[i], "-dfp") == 0 && (i + 2) < argc)
        {
            ++i;
            predatorAgent->loadAgent(argv[i]);
            ++i;
            predatorAgent->saveToDot(argv[i], true);
            exit(0);
        }
    }
    
    if (make_interval_video || make_LOD_video)
    {
        // start monitor first, then abeeda
        setupBroadcast();
    }
    
    // seed the agents
    delete swarmAgent;
    swarmAgent = new tAgent;
    swarmAgent->setupRandomAgent(5000);
    //swarmAgent->loadAgent("startSwarm.genome");
    
    delete predatorAgent;
    predatorAgent = new tAgent;
    //predatorAgent->setupRandomAgent(5000);
    predatorAgent->loadAgent((char *)"startPredator.genome");
    
    // make mutated copies of the start genome to fill up the initial population
	for(int i = 0; i < populationSize; ++i)
    {
		swarmAgents[i] = new tAgent;
		swarmAgents[i]->inherit(swarmAgent, 0.01, 1);
        
        predatorAgents[i] = new tAgent;
		predatorAgents[i]->inherit(predatorAgent, 0.01, 1);
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
            //game->executeGame(swarmAgents[i], predatorAgents[i], NULL, false);
            swarmAgents[i]->fitness = randDouble;
            predatorAgents[i]->fitness = randDouble;
            
            // store the swarm agent's corresponding predator agent
            swarmAgents[i]->predator = new tAgent;
            swarmAgents[i]->predator->inherit(predatorAgents[i], 0.0, predatorAgents[i]->born);
            predatorAgents[i]->nrPointingAtMe--;
            predatorAgents[i]->nrOfOffspring--;
            
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
        if (make_interval_video)
        {
            bool finalGeneration = update == totalGenerations;
            
            if (update % make_video_frequency == 0 || finalGeneration)
            {
                string reportString = "", bestString = "";
                double bestFitness = 0.0;
                
                // repeatedly face the two brains against each other to account for stochasticity
                for (int i = 0; i < 100; ++i)
                {
                    reportString = game->executeGame(bestSwarmAgent, bestPredatorAgent, NULL, true);
                    
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
        
        
		for(int i = 0; i < populationSize; ++i)
		{
            // construct swarm agent population for the next generation
			tAgent *d = new tAgent;
            int j = 0;
            
			do
            {
                j = rand() % populationSize;
            } while((j == i) || (randDouble > (swarmAgents[j]->fitness / swarmMaxFitness)));
            
			d->inherit(swarmAgents[j], perSiteMutationRate, update);
			SANextGen[i] = d;
            
            // construct predator agent population for the next generation
            d = new tAgent;
            j = 0;
            
			do
            {
                j = rand() % populationSize;
            } while((j == i) || (randDouble > (predatorAgents[j]->fitness / predatorMaxFitness)));
            
			d->inherit(predatorAgents[j], perSiteMutationRate, update);
			PANextGen[i] = d;
		}
        
        // shuffle the populations so there is a minimal chance of the same predator/prey combo in the next generation
        random_shuffle(SANextGen.begin(), SANextGen.end());
        random_shuffle(PANextGen.begin(), PANextGen.end());
        
		for(int i = 0; i < populationSize; ++i)
        {
            // retire and replace the swarm agents from the previous generation
			swarmAgents[i]->retire();
			swarmAgents[i]->nrPointingAtMe--;
			if(swarmAgents[i]->nrPointingAtMe == 0)
            {
				delete swarmAgents[i];
            }
			swarmAgents[i] = SANextGen[i];
            
            // retire and replace the predator agents from the previous generation
            predatorAgents[i]->retire();
			predatorAgents[i]->nrPointingAtMe--;
			if(predatorAgents[i]->nrPointingAtMe == 0)
            {
				delete predatorAgents[i];
            }
			predatorAgents[i] = PANextGen[i];
		}
        
		swarmAgents = SANextGen;
		predatorAgents = PANextGen;
        
        if (track_best_brains && update % track_best_brains_frequency == 0)
        {
            stringstream sss, pss;
            
            sss << "swarm" << update << ".genome";
            pss << "predator" << update << ".genome";
            
            swarmAgents[0]->ancestor->ancestor->saveGenome(sss.str().c_str());
            predatorAgents[0]->ancestor->ancestor->saveGenome(pss.str().c_str());
        }
	}
	
    // save the genome file of the lmrca
	swarmAgents[0]->ancestor->ancestor->saveGenome(swarmGenomeFileName.c_str());
    predatorAgents[0]->ancestor->ancestor->saveGenome(predatorGenomeFileName.c_str());
    
    // save video and quantitative stats on the best swarm agent's LOD
    vector<tAgent*> saveLOD;
    
    cout << "building ancestor list" << endl;
    
    // use 2 ancestors down from current population because that ancestor is highly likely to have high fitness
    tAgent* curAncestor = swarmAgents[0]->ancestor->ancestor;
    
    while (curAncestor != NULL)
    {
        // don't add the base ancestor
        if (curAncestor->ancestor != NULL)
        {
            saveLOD.insert(saveLOD.begin(), curAncestor);
        }
        
        curAncestor = curAncestor->ancestor;
    }
    
    FILE *LOD = fopen(LODFileName.c_str(), "w");

    fprintf(LOD, "generation,prey_fitness,predator_fitness,num_alive_end,avg_bb_size,var_bb_size,avg_shortest_dist,swarm_density_count,prey_neurons_connected_prey_retina,prey_neurons_connected_predator_retina,predator_neurons_connected_prey_retina,mutual_info\n");
    
    cout << "analyzing ancestor list" << endl;
    
    for (vector<tAgent*>::iterator it = saveLOD.begin(); it != saveLOD.end(); ++it)
    {
        if ((*it)->predator == NULL)
        {
            cout << "NULL predator at " << (*it)->born << endl;
        }
        else
        {
            // collect quantitative stats
            game->executeGame(*it, (*it)->predator, LOD, false);
            
            // make video
            if (make_LOD_video)
            {
                string bestString = "", reportString = "";
                double bestFitness = 0.0;
                
                for (int i = 0; i < 100; ++i)
                {
                    reportString = game->executeGame(*it, (*it)->predator, NULL, true);
                    
                    if ((*it)->fitness > bestFitness)
                    {
                        bestFitness = (*it)->fitness;
                        bestString = reportString;
                    }
                }
                
                if ( (it + 1) == saveLOD.end() )
                {
                    bestString.append("X");
                }
                
                doBroadcast(bestString);
            }
        }
    }
    
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
