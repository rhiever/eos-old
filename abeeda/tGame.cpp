/*
 *  tGame.cpp
 *  HMMBrain
 *
 *  Created by Arend on 9/23/10.
 *  Copyright 2010 __predYCompanyName__. All rights reserved.
 *
 */

#include "tGame.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>

#define cPI 3.14159265

// simulation-specific constants
#define preyVisionRange 100.0 * 100.0
#define predatorVisionRange 200.0 * 200.0
#define preySensors 12
#define predatorSensors 12
#define totalStepsInSimulation 2000
#define gridX 256.0
#define gridY 256.0
#define killDist 4.0 * 4.0
#define killChance 0.25
#define boundaryDist 250.0

// precalculated cos and sin lookup tables for the game
double cosLookup[360];
double sinLookup[360];

tGame::tGame()
{
    // fill lookup tables
    for (int i = 0; i < 360; ++i)
    {
        cosLookup[i] = cos((double)i * (cPI / 180.0));
        sinLookup[i] = sin((double)i * (cPI / 180.0));
    }
}

tGame::~tGame()
{
}

// runs the simulation for the given agent(s)
string tGame::executeGame(tAgent* swarmAgent, tAgent* predatorAgent, FILE *data_file, bool report)
{
    // LOD data variables
    double swarmFitness = 0.0;
    double predatorFitness = 0.0;
    
    vector<double> bbSizes;
    vector<double> shortestDists;
    vector<double> swarmDensityCount20;
    vector<double> swarmDensityCount30;
    vector<double> swarmDensityCount40;
    vector<int> predatorAngle, preyAngle;
    vector<double> distsToCentroid[swarmSize];
    
    // swarm agent x, y, angles
    double preyX[swarmSize], preyY[swarmSize], preyA[swarmSize];
    // swarm alive status
    bool preyDead[swarmSize];
    
    // lookup table for distances between predator and swarm agents
    double predDists[swarmSize];
    
    // lookup table for distances between swarm agents and other swarm agents
    double preyDists[swarmSize][swarmSize];
    
    // counter of how many swarm agents are still alive
    int numAlive = swarmSize;
    
    // predator X, Y, and angle
    double predX = (double)((double)rand()/(double)RAND_MAX * gridX * 2.0) - gridX;
    double predY = (double)((double)rand()/(double)RAND_MAX * gridY * 2.0) - gridY;
    double predA = (int)((double)rand()/(double)RAND_MAX * 360.0);

    // string containing the information to create a video of the simulation
    string reportString = "";
    
    // set up brain for clone swarm
    swarmAgent->setupMegaPhenotype(swarmSize);
    swarmAgent->fitness = 0.0;
    
    // set up predator brain
    predatorAgent->setupPhenotype();
    predatorAgent->fitness = 0.0;
    
    for(int k = 0; k < swarmSize; ++k)
    {
        preyX[k] = (double)((double)rand() / (double)RAND_MAX * gridX * 2.0) - gridX;
        preyY[k] = (double)((double)rand() / (double)RAND_MAX * gridY * 2.0) - gridY;
        preyA[k] = (int)((double)rand() / (double)RAND_MAX * 360.0);
        preyDead[k] = false;
    }
    
    // initialize predator and prey lookup tables
    recalcPredAndPreyDistTable(preyX, preyY, preyDead, predX, predY, predDists, preyDists);
    
    /*       BEGINNING OF SIMULATION LOOP       */
    
    for(int step = 0; step < totalStepsInSimulation; ++step)
    {
        
        /*       CREATE THE REPORT STRING FOR THE VIDEO       */
        if(report)
        {
            // report X, Y, angle of predator
            char text[1000];
            sprintf(text,"%f,%f,%f,%d,%d,%d=", predX, predY, predA, 255, 0, 0);
            reportString.append(text);
            
            // compute center of swarm
            double cX = 0.0, cY = 0.0;
            calcSwarmCenter(preyX,preyY, preyDead, cX, cY);
            
            // report X, Y of center of swarm
            char text2[1000];
            sprintf(text2,"%f,%f,%f,%d,%d,%d=", cX, cY, 0.0, 124, 252, 0);
            reportString.append(text2);
            
            // report X, Y, angle of all prey
            for(int i = 0; i <swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    char text[1000];
                    
                    double dist = predDists[i];
                    double angle = calcAngle(predX, predY, predA, preyX[i], preyY[i], dist);
                    
                    // here we have to map the angle into the sensor, btw: angle in degrees
                    if(fabs(angle) < 45 && dist < predatorVisionRange) // predator has a 90 degree vision field in front of it
                    {
                        sprintf(text,"%f,%f,%f,%d,%d,%d=", preyX[i], preyY[i], preyA[i], 30, 144, 255);
                    }
                    else
                    {
                        sprintf(text,"%f,%f,%f,%d,%d,%d=", preyX[i], preyY[i], preyA[i], 255, 255, 255);
                    }
                    
                    reportString.append(text);
                }
            }
            reportString.append("N");
            
        }
        /*       END OF REPORT STRING CREATION       */
        
        
        /*       SAVE DATA FOR THE LOD FILE       */
        if(data_file != NULL)
        {
            // calculate bounding box size for this update
            // lu = Left Uppermost point
            // rb = Right Bottommost point
            double luX = DBL_MAX, luY = DBL_MAX;
            double rbX = -DBL_MAX, rbY = -DBL_MAX;
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    if (preyX[i] < luX)
                    {
                        luX = preyX[i];
                    }
                    
                    if (preyX[i] > rbX)
                    {
                        rbX = preyX[i];
                    }
                    
                    if (preyY[i] < luY)
                    {
                        luY = preyY[i];
                    }
                    
                    if (preyY[i] > rbY)
                    {
                        rbY = preyY[i];
                    }
                }
            }
            
            // area = L x W
            //                L = dist (rbX, rbY) to (rbX, luY); W = dist (luX, luY) to (rbX, luY)
            bbSizes.push_back( calcDistanceSquared(rbX, rbY, rbX, luY) * calcDistanceSquared(luX, luY, rbX, luY) );
            
            // calculate mean of shortest distance to other swarm agents
            double aliveCount = 0.0;
            double meanShortestDist = 0.0;
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    aliveCount += 1.0;
                    
                    // find closest agent to agent i
                    double shortestDist = DBL_MAX;
                    
                    for(int j = 0; j < swarmSize; ++j)
                    {
                        if (!preyDead[j] && i != j)
                        {
                            if (preyDists[i][j] < shortestDist)
                            {
                                shortestDist = preyDists[i][j];
                            }
                        }
                    }
                    
                    // sum the shortest distance for agent i
                    meanShortestDist += shortestDist;
                }
            }
            
            // average the shortest distances
            meanShortestDist /= aliveCount;
            
            // store mean shortest dist for this update
            shortestDists.push_back(meanShortestDist);
            
            // calculate swarm density counts: avg. agents within 20, 30, and 40 units of each other
            double avgWithin20 = 0.0, avgWithin30 = 0.0, avgWithin40 = 0.0;
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    for(int j = 0; j < swarmSize; ++j)
                    {
                        if (!preyDead[j] && i != j)
                        {
                            double dist = preyDists[i][j];
                            
                            if (dist <= 40.0 * 40.0)
                            {
                                avgWithin40 += 1.0;
                                avgWithin30 += 1.0;
                                avgWithin20 += 1.0;
                            }
                            else if (dist <= 30.0 * 30.0)
                            {
                                avgWithin30 += 1.0;
                                avgWithin20 += 1.0;
                            }
                            else if (dist <= 20.0 * 20.0)
                            {
                                avgWithin20 += 1.0;
                            }
                        }
                    }
                }
            }
            
            avgWithin20 /= aliveCount;
            avgWithin30 /= aliveCount;
            avgWithin40 /= aliveCount;
            
            swarmDensityCount20.push_back(avgWithin20);
            swarmDensityCount30.push_back(avgWithin30);
            swarmDensityCount40.push_back(avgWithin40);
            
            // log predator and prey angles
            for(int i = 0; i < swarmSize; ++i)
            {
                if(!preyDead[i])
                {
                    predatorAngle.push_back((int)(predA/36.0));
                    preyAngle.push_back((int)(preyA[i]/36.0));
                }
            }

            // log distances to swarm centroid
            double cX = 0.0, cY = 0.0;
            calcSwarmCenter(preyX,preyY, preyDead, cX, cY);
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    distsToCentroid[i].push_back(calcDistanceSquared(preyX[i], preyY[i], cX, cY));
                }
            }
            
        }
        /*       END OF DATA GATHERING       */
        
        
        /*       UPDATE PREDATOR       */
        
        // clear the predator sensors
        for(int i = 0; i < predatorSensors; ++i)
        {
            predatorAgent->states[i] = 0;
        }
        
        // update the predator sensors
        for(int i = 0; i < swarmSize; ++i)
        {
            if (!preyDead[i])
            {
                double d = predDists[i];
                
                // don't bother if an agent is too far
                if(d < predatorVisionRange)
                {
                    double angle = calcAngle(predX, predY, predA, preyX[i], preyY[i], d);
                    
                    // here we have to map the angle into the sensor, btw: angle in degrees
                    if(fabs(angle) < 45) // predator has a 90 degree vision field in front of it
                    {
                        predatorAgent->states[(int)(angle / (90.0 / ((double)predatorSensors / 2.0)) + ((double)predatorSensors / 2.0))] = 1;
                    }
                }
            }
        }
        
        // activate the predator agent's brain
        predatorAgent->updateStates();
        
        //                                      node 31                                              node 30
        int action = ((predatorAgent->states[(maxNodes - 1)] & 1) << 1) + (predatorAgent->states[(maxNodes - 2)] & 1);
        
        switch(action)
        {
            // do nothing
            case 0:
                break;
                
            // turn 8 degrees right
            case 1:
                predA += 8.0;
                
                while(predA > 360.0)
                {
                    predA -= 360.0;
                }
                
                predX += cosLookup[(int)predA];
                predY += sinLookup[(int)predA];
                
                predatorFitness += 1;
                
                break;
                
            // turn 8 degrees left
            case 2:
                predA -= 8.0;
                
                while(predA < 0.0)
                {
                    predA += 360.0;
                }
                
                predX += cosLookup[(int)predA];
                predY += sinLookup[(int)predA];
                
                predatorFitness += 1;
                
                break;
                
            // move straight ahead
            case 3:
                predX += cosLookup[(int)predA] * 2.0;
                predY += sinLookup[(int)predA] * 2.0;
                
                predatorFitness += 2;
                
                break;
                
            default:
                break;
        }
        
        // keep position within simulation boundary
        applyBoundary(predX);
        applyBoundary(predY);
        
        // determine if the predator made a kill
        if (numAlive > 2)
        {
            bool killed = false;
            
            for(int i = 0; i < swarmSize && !killed; ++i)
            {
                if (!preyDead[i])
                {
                    double d = predDists[i];
                    
                    if ((d < killDist) && (randDouble > killChance))
                    {
                        preyDead[i] = killed = true;
                        --numAlive;
                    }
                }
            }
        }
        
        // recalculate the predator distances lookup table since the predator has moved
        recalcPredDistTable(preyX, preyY, preyDead, predX, predY, predDists, preyDists);
        
        /*       END OF PREDATOR UPDATE       */
        
        
        /*       UPDATE SWARM       */
        for(int i = 0; i < swarmSize; ++i)
        {
            if (!preyDead[i])
            {
                //clear the sensors of agent i
                for(int j = 0; j < preySensors * 2; ++j)
                {
                    swarmAgent->states[j + (i * maxNodes)] = 0;
                }
                
                //iterate for agent i over all agents j
                // indicate the presence of other visible agents in agent i's retina
                for(int j = 0; j < swarmSize; ++j)
                {
                    //ignore i==j because an agent can't see itself
                    if(i != j && !preyDead[j])
                    {
                        double d = preyDists[i][j];
			
                        //don't bother if an agent is too far
                        if(d < preyVisionRange)
                        {
                            double angle = calcAngle(preyX[i], preyY[i], preyA[i], preyX[j], preyY[j], d);
                            
                            //here we have to map the angle into the sensor, btw: angle in degrees
                            if(fabs(angle) < 90) // you have a 180 degree vision field infront of you
                            {
                                swarmAgent->states[(int)(angle / (90.0 / ((double)preySensors / 2.0)) + ((double)preySensors / 2.0)) + (i * maxNodes)] = 1;
                            }
                        }
                    }
                }
                
                // indicate the presence of the predator in agent i's retina
                double d = predDists[i];
                
                if (d < preyVisionRange)
                {
                    double angle = calcAngle(preyX[i], preyY[i], preyA[i], predX, predY, d);

                    //here we have to map the angle into the sensor, btw: angle in degree
                    // you have a 180 degree vision field infront of you
                    if(fabs(angle) < 90)
                    {
                        swarmAgent->states[preySensors + (int)(angle / (90.0 / ((double)preySensors / 2.0)) + ((double)preySensors / 2.0)) + (i * maxNodes)] = 1;
                    }
                }
            }
        }
        
        // activate the swarm agent's brains
        swarmAgent->updateStates();
        
        // activate each swarm agent's brain, determine its action for this update, and update its position and angle
        for(int i = 0; i < swarmSize; ++i)
        {
            if (!preyDead[i])
            {
                //                                  node 31                                                         node 30
                int action = ((swarmAgent->states[(maxNodes - 1) + (i * maxNodes)] & 1) << 1) + (swarmAgent->states[(maxNodes - 2) + (i * maxNodes)] & 1);
                
                switch(action)
                {
                    // do nothing
                    case 0:
                        break;
                        
                    // turn 8 degrees right
                    case 1:
                        preyA[i] += 8.0;
                        
                        while(preyA[i] > 360.0)
                        {
                            preyA[i] -= 360.0;
                        }
                        
                        preyX[i] += cosLookup[(int)preyA[i]];
                        preyY[i] += sinLookup[(int)preyA[i]];
                        
                        swarmFitness += 1.0 / (double)swarmSize;
                        
                        break;
                        
                    // turn 8 degrees left
                    case 2:
                        preyA[i] -= 8.0;
                        while(preyA[i] < 0.0)
                        {
                            preyA[i] += 360.0;
                        }
                        
                        preyX[i] += cosLookup[(int)preyA[i]];
                        preyY[i] += sinLookup[(int)preyA[i]];
                        
                        swarmFitness += 1.0 / (double)swarmSize;
                        
                        break;
                        
                    // move straight ahead
                    case 3:
                        preyX[i] += cosLookup[(int)preyA[i]];
                        preyY[i] += sinLookup[(int)preyA[i]];
                        
                        swarmFitness += 2.0 / (double)swarmSize;
                        
                        break;
                        
                    default:
                        break;
                }

                // keep position within boundary
                applyBoundary(preyX[i]);
                applyBoundary(preyY[i]);
            }
        }
        
        // recalculate both the predator and prey distances lookup tables since the entire swarm has moved
        recalcPredAndPreyDistTable(preyX, preyY, preyDead, predX, predY, predDists, preyDists);
        
        /*       END OF SWARM UPDATE       */

        
        /*       DETERMINE FITNESSES FOR THIS UPDATE       */
        
        predatorFitness += swarmSize - numAlive;
        
        vector<double> nearbyCounts;
        
        for (int i = 0; i < swarmSize; ++i)
        {
            if (!preyDead[i])
            {
                int nearbyCount = 1;
                
                for (int j = 0; j < swarmSize; ++j)
                {
                    if (!preyDead[j] && i != j && preyDists[i][j] < 50.0 * 50.0)
                    {
                        ++nearbyCount;
                    }
                }
                
                nearbyCounts.push_back((double)nearbyCount);
            }
        }
        
        swarmFitness += numAlive * (average(nearbyCounts) / (double)numAlive);
        
        /*       END OF FITNESS CALCULATIONS       */
        
    }
    /*       END OF SIMULATION LOOP       */
    
    // compute overall fitness
    swarmAgent->fitness = swarmFitness;
    predatorAgent->fitness = predatorFitness;
    
    if(swarmAgent->fitness < 0.0)
    {
        swarmAgent->fitness = 0.0;
    }
    
    if(predatorAgent->fitness < 0.0)
    {
        predatorAgent->fitness = 0.0;
    }
    
    // output to data file, if provided
    if (data_file != NULL)
    {
        // find final bounding box coordinates
        double luX = DBL_MAX, luY = DBL_MAX;
        double rbX = -DBL_MAX, rbY = -DBL_MAX;
        
        for(int i = 0; i < swarmSize; ++i)
        {
            if (!preyDead[i])
            {
                if (preyX[i] < luX)
                {
                    luX = preyX[i];
                }
                
                if (preyX[i] > rbX)
                {
                    rbX = preyX[i];
                }
                
                if (preyY[i] < luY)
                {
                    luY = preyY[i];
                }
                
                if (preyY[i] > rbY)
                {
                    rbY = preyY[i];
                }
            }
        }

        // compute avg variance in each swarm agent's distance to the swarm centroid
        double avgVarianceDistToCentroid = 0.0;
        vector<double> varsDistToCentroid;
        
        for (int i = 0; i < swarmSize; ++i)
        {
            varsDistToCentroid.push_back(variance(distsToCentroid[i]));
        }
        
        avgVarianceDistToCentroid = average(varsDistToCentroid);
            
        fprintf(data_file, "%d %f %f %d %f %f %f %f %f %f %f %f %f %f %i %i %f %f\n",
                swarmAgent->born,               // update born
                swarmAgent->fitness,            // swarm fitness
                predatorAgent->fitness,         // predator fitness
                numAlive,                       // # alive at end
                luX, luY,                       // (x1, y1) of bounding box at end
                rbX, rbY,                       // (x2, y2) of bounding box at end
                average(bbSizes),               // average bounding box size
                variance(bbSizes),              // variance in bounding box size
                average(shortestDists),         // average of avg. shortest distance to other swarm agent
                average(swarmDensityCount20),   // average # of agents within 20 units of each other
                average(swarmDensityCount30),   // average # of agents within 30 units of each other
                average(swarmDensityCount40),   // average # of agents within 40 units of each other
                neuronsConnectedToPreyRetina(swarmAgent), // #neurons connected to prey part of retina
                neuronsConnectedToPredatorRetina(swarmAgent), // #neurons connected to predator part of retina
                mutualInformation(predatorAngle, preyAngle), // mutual Information between prey flight angle and predator flight angle
                avgVarianceDistToCentroid       // avg variance in each swarm agent's distance to the swarm centroid
                );
    }
    
    return reportString;
}


// calculates the distance^2 between two points
double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
{
    double diffX = fromX - toX;
    double diffY = fromY - toY;
    
    return ( diffX * diffX ) + ( diffY * diffY );
}

// calculates the angle between two agents
double tGame::calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY, double dist)
{
    double d = 0.0;
    double Ux, Uy, Vx, Vy;
    
    if (dist == 0.0)
    {
        d = calcDistanceSquared(fromX, fromY, toX, toY);
    }
    else
    {
        d = dist;
    }
    
    //ann kathete divided by hypothenuse
    Ux = (toX - fromX) / d;
    
    //gegenkathete divided by hypothenuse
    Uy = (toY - fromY) / d;
    
    //I forgot what the line below does...
    Vx = cosLookup[(int)fromAngle];
    Vy = sinLookup[(int)fromAngle];
    
    //anyway the following line computes the angle between predY own and the object I am looking at
    return atan2(((Ux * Vy) - (Uy * Vx)), ((Ux * Vx) + (Uy * Vy))) * 180.0 / cPI;
}

// calculates the center of the swarm and stores it in (cX, cY)
void tGame::calcSwarmCenter(double preyX[], double preyY[], bool preyDead[], double& preyCenterX, double& preyCenterY)
{
    int aliveCount = 0;
    preyCenterX = 0.0;
    preyCenterY = 0.0;
    
    for(int i = 0; i < swarmSize; ++i)
    {
        if (!preyDead[i])
        {
            preyCenterX += preyX[i];
            preyCenterY += preyY[i];
            ++aliveCount;
        }
    }
    
    preyCenterX /= (double)aliveCount;
    preyCenterY /= (double)aliveCount;
}

// recalculates only the predator distance lookup table
void tGame::recalcPredDistTable(double preyX[], double preyY[], bool preyDead[],
                                double predX, double predY,
                                double predDists[swarmSize], double preyDists[swarmSize][swarmSize])
{
    for (int i = 0; i < swarmSize; ++i)
    {
        if (!preyDead[i])
        {
            predDists[i] = calcDistanceSquared(predX, predY, preyX[i], preyX[i]);
        }
    }
}

// recalculates the predator and prey distance lookup tables
void tGame::recalcPredAndPreyDistTable(double preyX[], double preyY[], bool preyDead[],
                                       double predX, double predY,
                                       double predDists[swarmSize], double preyDists[swarmSize][swarmSize])
{
    for (int i = 0; i < swarmSize; ++i)
    {
        if (!preyDead[i])
        {
            predDists[i] = calcDistanceSquared(predX, predY, preyX[i], preyX[i]);
            preyDists[i][i] = 0.0;
            
            for (int j = i + 1; j < swarmSize; ++j)
            {
                preyDists[i][j] = calcDistanceSquared(preyX[i], preyY[i], preyX[j], preyY[j]);
                preyDists[j][i] = preyDists[i][j];
            }
        }
    }
}

// maintains a position within a preset boundary
void tGame::applyBoundary(double& positionVal)
{
    double val = positionVal;

    if (fabs(val) > boundaryDist)
    {
        if (val < 0)
        {
            val = -1.0 * boundaryDist;
        }
        else
        {
            val = boundaryDist;
        }
    }
    
    positionVal = val;
}

// sums a vector of values
double tGame::sum(vector<double> values)
{
    double sum = 0.0;
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sum += *i;
    }
    
    return sum;
}

// averages a vector of values
double tGame::average(vector<double> values)
{
    return sum(values) / (double)values.size();
}

// computes the variance of a vector of values
double tGame::variance(vector<double> values)
{
    double sumSqDist = 0.0;
    double mean = average(values);
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sumSqDist += pow( *i - mean, 2.0 );
    }
    
    return sumSqDist /= (double)values.size();
}

double tGame::mutualInformation(vector<int> A,vector<int>B)
{
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]);
		nrB.insert(B[i]);
		pX[A[i]]=0.0;
		pY[B[i]]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]][B[i]]+=c;
		pX[A[i]]+=c;
		pY[B[i]]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pX[*aI]*pY[*bI]));
	return I;
	
}

double tGame::entropy(vector<int> list){
	map<int, double> p;
	map<int,double>::iterator pI;
	int i;
	double H=0.0;
	double c=1.0/(double)list.size();
	for(i=0;i<list.size();i++)
		p[list[i]]+=c;
	for (pI=p.begin();pI!=p.end();pI++) {
			H+=p[pI->first]*log2(p[pI->first]);	
	}
	return -1.0*H;
}

double tGame::ei(vector<int> A,vector<int> B,int theMask){
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]&theMask);
		nrB.insert(B[i]&theMask);
		pX[A[i]&theMask]=0.0;
		pY[B[i]&theMask]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]&theMask][B[i]&theMask]+=c;
		pX[A[i]&theMask]+=c;
		pY[B[i]&theMask]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pY[*bI]));
	return -I;
}

double tGame::computeAtomicPhi(vector<int>A,int states){
	int i;
	double P,EIsystem;
	vector<int> T0,T1;
	T0=A;
	T1=A;
	T0.erase(T0.begin()+T0.size()-1);
	T1.erase(T1.begin());
	EIsystem=ei(T0,T1,(1<<states)-1);
	P=0.0;
	for(i=0;i<states;i++){
		double EIP=ei(T0,T1,1<<i);
//		cout<<EIP<<endl;
		P+=EIP;
	}
//	cout<<-EIsystem+P<<" "<<EIsystem<<" "<<P<<" "<<T0.size()<<" "<<T1.size()<<endl;
	return -EIsystem+P;
}

double tGame::computeR(vector<vector<int> > table,int howFarBack){
	double Iwh,Iws,Ish,Hh,Hs,Hw,Hhws,delta,R;
	int i;
	for(i=0;i<howFarBack;i++){
		table[0].erase(table[0].begin());
		table[1].erase(table[1].begin());
		table[2].erase(table[2].begin()+(table[2].size()-1));
	}
	table[4].clear();
	for(i=0;i<table[0].size();i++){
		table[4].push_back((table[0][i]<<14)+(table[1][i]<<10)+table[2][i]);
	}
	Iwh=mutualInformation(table[0],table[2]);
    Iws=mutualInformation(table[0],table[1]);
    Ish=mutualInformation(table[1],table[2]);
    Hh=entropy(table[2]);
    Hs=entropy(table[1]);
    Hw=entropy(table[0]);
    Hhws=entropy(table[4]);
    delta=Hhws+Iwh+Iws+Ish-Hh-Hs-Hw;
    R=Iwh-delta;
  	return R;
}

double tGame::computeOldR(vector<vector<int> > table){
	double Ia,Ib;
	Ia=mutualInformation(table[0], table[2]);
	Ib=mutualInformation(table[1], table[2]);
	return Ib-Ia;
}

double tGame::predictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return mutualInformation(S, I);
}

double tGame::nonPredictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return entropy(I)-mutualInformation(S, I);
}

double tGame::predictNextInput(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	S.erase(S.begin());
	I.erase(I.begin()+I.size()-1);
	return mutualInformation(S, I);
}

void tGame::loadExperiment(char *filename){
    theExperiment.loadExperiment(filename);
}

int tGame::neuronsConnectedToPreyRetina(tAgent *agent){
    tAgent *A=new tAgent;
    int i,j,c=0;
    A->genome=agent->genome;
    A->setupPhenotype();
    for(i=0;i<A->hmmus.size();i++)
        for(j=0;j<A->hmmus[i]->ins.size();j++)
        if(A->hmmus[i]->ins[j]<preySensors)
            c++;
    delete A;
    return c;
}

int tGame::neuronsConnectedToPredatorRetina(tAgent* agent){
    tAgent *A=new tAgent;
    int i,j,c=0;
    A->genome=agent->genome;
    A->setupPhenotype();
    for(i=0;i<A->hmmus.size();i++)
        for(j=0;j<A->hmmus[i]->ins.size();j++)
            if((A->hmmus[i]->ins[j]<(preySensors*2))&&(A->hmmus[i]->ins[j]>=preySensors))
                c++;
    delete A;
    return c;
    
}

//** tOctuplet implementation
void tOctuplet::loadOctuplet(FILE *f){
    int i,IN;
    data.clear();
    data.resize(8);
    for(i=0;i<8;i++){
        fscanf(f,"  %i",&IN);
        data[i]=IN;
    }
}

//** tEperiment class implementations
void tExperiment::loadExperiment(char *filename){
    FILE *f=fopen(filename,"r+t");
    int i,j,k;
    fscanf(f,"%i:",&j);
    dropSequences.resize(j);
    for(i=0;i<dropSequences.size();i++)
        dropSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    sizeSequences.resize(j);
    for(i=0;i<sizeSequences.size();i++)
        sizeSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    selfSequences.resize(j);
    for(i=0;i<selfSequences.size();i++)
        selfSequences[i].loadOctuplet(f);
    shouldHit.resize(drops());
    for(i=0;i<shouldHit.size();i++){
        shouldHit[i].resize(sizes());
        for(j=0;j<shouldHit[i].size();j++){
            shouldHit[i][j].resize(selves());
            for(k=0;k<shouldHit[i][j].size();k++){
                int l;
                fscanf(f,"%i\n",&l);
                if(l==1)
                    shouldHit[i][j][k]=true;
                else
                    shouldHit[i][j][k]=false;
            }
        }
    }
    fclose(f);
}

void tExperiment::showExperimentProtokoll(void){
    int i,j,k;
    printf("drop directions: %i\n",drops());
    for(i=0;i<drops();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",dropSequences[i].data[j]);
        printf("\n");
    }
    printf("drop sizes: %i\n",sizes());
    for(i=0;i<sizes();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",sizeSequences[i].data[j]);
        printf("\n");
    }
    printf("self sizes: %i\n",selves());
    for(i=0;i<selves();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",selfSequences[i].data[j]);
        printf("\n");
    }
    printf("should hit\n%i means true\nD  B   S   catch\n",(int)true);
    for(i=0;i<shouldHit.size();i++)
        for(j=0;j<shouldHit[i].size();j++)
            for(k=0;k<shouldHit[i][j].size();k++)
                printf("%i  %i  %i  %i\n",i,j,k,(int)shouldHit[i][j][k]);
}

int tExperiment::drops(void){
    return (int) dropSequences.size();
}

int tExperiment::sizes(void){
    return (int) sizeSequences.size();
}

int tExperiment::selves(void){
    return (int) selfSequences.size();
    
}
