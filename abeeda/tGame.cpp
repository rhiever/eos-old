/*
 *  tGame.cpp
 *  HMMBrain
 *
 *  Created by Arend on 9/23/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "tGame.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>

#define cEmpty 0
#define cAgent 1
#define cFood 2
#define cWall 3
#define cDanceUp 4
#define cDanceRight 5
#define cDanceDown 6
#define cDanceLeft 7

#define visionRange 100.0
#define sensors 12
#define cPI 3.14159265

tGame::tGame()
{
}

tGame::~tGame()
{
}


string tGame::executeGame(tAgent* agent, FILE *data_file, bool report, float p)
{
    // LOD data variables
    float fitnessFromSwarming = 0.0;
    float fitnessFromPredation = 0.0;
    vector<double> bbSizes;
    vector<double> shortestDists;
    vector<double> sumSqrtDists;
    vector<double> swarmDensityCount20;
    vector<double> swarmDensityCount30;
    vector<double> swarmDensityCount40;
    vector<int> predatorAngle,preyAngle;
    vector<double> distsToCentroid[hiveSize];
    bool swarm_fitness = true;
    bool predator_fitness = true;
    
    // swarm agent x, y, angles, and alive status
    float x[hiveSize], y[hiveSize], a[hiveSize];
    bool dead[hiveSize];
    float delay=0.0;
    // predator initial X, Y, and angle
    float mX = (float)((float)rand()/(float)RAND_MAX*gridX);
    float mY = (float)((float)rand()/(float)RAND_MAX*gridY);
    float mA = (float)((float)rand()/(float)RAND_MAX*360.0);
    
    float sD = 0.0, pD = 0.0;
    int i = 0, j = 0, k = 0;
    int action = 0;
    string reportString = "";
    agent->setupMegaPhenotype(hiveSize);
    agent->fitness = 0.0;
    
    for(k = 0; k < hiveSize; k++)
    {
        x[k] = (float)((float)rand() / (float)RAND_MAX * gridX);
        y[k] = (float)((float)rand() / (float)RAND_MAX * gridY);
        a[k] = (float)((float)rand() / (float)RAND_MAX * 360.0);
        dead[k] = false;
    }
    
    int targetPrey = rand() % hiveSize;
    
    for(k = 0; k < totalStepsInMaze; k++)
    {
        if(predator_fitness)
        {
            // Predator randomly sometimes chooses new prey
            if(( (float)( (float)rand() / (float)RAND_MAX ) < selectNewPreyChance ) || (dead[targetPrey]) )
            {
                int newPrey = selectClosestPrey(x, y, a, dead, mX, mY, mA);
                
                if (newPrey != -1)
                {
                    if (targetPrey != newPrey && (float)rand() / (float)RAND_MAX < 0.15)
                    {
                        delay = 60.0;
                    }
                    
                    targetPrey = newPrey;
                }
            }
            
            // Update predator movement angle
            double d = calcDistance(mX, mY, x[targetPrey], y[targetPrey]);
            
            double Ux,Uy,Vx,Vy;
            double angle;
            //ann kathete divided by hypothenuse
            Ux=(x[targetPrey]-mX)/d;
            //gegenkathete divided by hypothenuse
            Uy=(y[targetPrey]-mY)/d;
            //I forgot what the line below does...
            Vx=cos(mA*(cPI/180.0));
            Vy=sin(mA*(cPI/180.0));
            //anyway the following line computes the angle between my own and the object I am looking at
            angle = atan2(((Ux*Vy)-(Uy*Vx)), ((Ux*Vx)+(Uy*Vy)))*180.0/cPI;
            
            //here we have to map the angle into the sensor, btw: angle in degree
            double speed = 2.5;
            double tA = 4.0;
            
            if (d > 80.0)
            {
                tA = 16.0;
            }
            else if (d > 10.0)
            {
                tA = 8.0;
            }
            
            if(angle>tA)
                angle=tA;
            if(angle<tA)
                angle=-tA;
            if(delay<0.0)
                mA-=angle;

            delay-=1.0;

            // Update predator position
            mX += cos(mA * (cPI / 180.0)) * speed;
            mY += sin(mA * (cPI / 180.0)) * speed;

            // keep position within boundary
            mX = applyBoundary(mX);
            mY = applyBoundary(mY);

            //change the 191 to a higher number to make the predator less effective
            //or reduce the 5.0
            if(predator_fitness && (d < killDist) && ((rand()&255)>killChance))
            {
                j = hiveSize;
                for(i = 0; i < hiveSize; i++)
                {
                    if(dead[i])
                    {
                        j--;
                    }
                }
                if(j > 2)
                {
                    dead[targetPrey]=true;
                    
                    do
                    {
                        targetPrey = rand() % hiveSize;
                    } while (dead[targetPrey]);
                }
                
                // increase the delay to make the predator fly away for longer after it successfully killed a target
                delay = 40.0;
            }
        }
        
        // create the report string for the video
        if(report)
        {            
            if (predator_fitness)
            {
                // report X, Y, angle of predator as first entry
                char text[1000];
                sprintf(text,"%f,%f,%f,%d,%d,%d=", mX, mY, mA, 255, 0, 0);
                reportString.append(text);
            }
            
            // compute center of swarm
            double cX = 0, cY = 0;
            unsigned int aliveCount = 0;
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    cX += x[i];
                    cY += y[i];
                    aliveCount++;
                }
            }
            
            cX /= (double)aliveCount;
            cY /= (double)aliveCount;
            
            // report X, Y of center of swarm as second entry
            char text2[1000];
            sprintf(text2,"%f,%f,%f,%d,%d,%d=", cX, cY, 0.0, 124, 252, 0);
            reportString.append(text2);
            
            // report X, Y, angle of all prey
            for(i=0;i<hiveSize;i++)
            {
                if (!dead[i])
                {
                    char text[1000];
                    
                    if(i == targetPrey && predator_fitness)
                    {
                        sprintf(text,"%f,%f,%f,%d,%d,%d=", x[i], y[i], a[i], 128, 128, 255);
                    }
                    else
                    {
                        sprintf(text,"%f,%f,%f,%d,%d,%d=", x[i], y[i], a[i], 255, 255, 255);
                    }
                    reportString.append(text);
                }
            }
            reportString.append("N");
        }
        
        // save data for the LOD file, if provided
        if(data_file != NULL)
        {
            // calculate bounding box size for this update
            // lu = Left Uppermost point
            // rb = Right Bottommost point
            double luX = DBL_MAX, luY = DBL_MAX;
            double rbX = -DBL_MAX, rbY = -DBL_MAX;
            
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    if (x[i] < luX)
                    {
                        luX = x[i];
                    }
                    
                    if (x[i] > rbX)
                    {
                        rbX = x[i];
                    }
                    
                    if (y[i] < luY)
                    {
                        luY = y[i];
                    }
                    
                    if (y[i] > rbY)
                    {
                        rbY = y[i];
                    }
                }
            }
            
            // area = L x W
            //                L = dist (rbX, rbY) to (rbX, luY); W = dist (luX, luY) to (rbX, luY)
            bbSizes.push_back( calcDistance(rbX, rbY, rbX, luY) * calcDistance(luX, luY, rbX, luY) );
            
            
            // calculate mean of shortest distance to other swarm agents
            double aliveCount = 0.0;
            double meanShortestDist = 0.0;
            
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    aliveCount += 1.0;
                    
                    // find closest agent to agent i
                    double shortestDist = DBL_MAX;
                    
                    for(j = 0; j < hiveSize; j++)
                    {
                        if (!dead[j] && i != j)
                        {
                            double dist = calcDistance(x[i], y[i], x[j], y[j]);
                            if (dist < shortestDist)
                            {
                                shortestDist = dist;
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
            
            
            // calculate sum of sqrt distances between every alive agent
            double sumSqrtDist = 0.0;
            
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    for(j = i; j < hiveSize; j++)
                    {
                        if (!dead[j] && i != j)
                        {
                            // sum the sqrt of the dist between every alive agent
                            sumSqrtDist += sqrt( calcDistance(x[i], y[i], x[j], y[j]) );
                        }
                    }
                }
            }
            
            // average dists over # agents alive
            sumSqrtDist /= aliveCount;
            
            // store sum of sqrt distances
            sumSqrtDists.push_back(sumSqrtDist);
            
            
            // calculate swarm density counts: avg. agents within 20, 30, and 40 units of each other
            double avgWithin20 = 0.0, avgWithin30 = 0.0, avgWithin40 = 0.0;
            
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    for(j = 0; j < hiveSize; j++)
                    {
                        if (!dead[j] && i != j)
                        {
                            double dist = calcDistance(x[i], y[i], x[j], y[j]);
                            
                            if (dist <= 40.0)
                            {
                                avgWithin40 += 1.0;
                                avgWithin30 += 1.0;
                                avgWithin20 += 1.0;
                            }
                            else if (dist <= 30.0)
                            {
                                avgWithin30 += 1.0;
                                avgWithin20 += 1.0;
                            }
                            else if (dist <= 20.0)
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
            
            //log predator and prey angles
            for(i = 0; i < hiveSize; i++)
            {
                if(!dead[i])
                {
                    predatorAngle.push_back((int)(mA/36.0));
                    preyAngle.push_back((int)(a[i]/36.0));
                }
            }

            // log distances to swarm centroid
            double cX = 0, cY = 0;
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    cX += x[i];
                    cY += y[i];
                }
            }
            
            cX /= aliveCount;
            cY /= aliveCount;
            
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    distsToCentroid[i].push_back(calcDistance(x[i], y[i], cX, cY));
                }
            }
            
            /*unsigned int aliveCount = 0;
            for(i = 0; i < hiveSize; i++)
            {
                if (!dead[i])
                {
                    aliveCount++;
                }
            }
            
            fprintf(data_file, "%d %f %f %d %d ", k, fitnessFromSwarming, fitnessFromPredation, aliveCount, hiveSize);
            
            // report X, Y, alive status of all prey
            for(i = 0; i < hiveSize; i++)
            {
                fprintf(data_file, "%f %f %d ", x[i], y[i], dead[i]);
            }
            
            if(predator_fitness)
            {
                fprintf(data_file, "%f %f ", mX, mY);
            }
            
            fprintf(data_file, "\n");*/
        }
        
        for(i = 0; i < hiveSize; i++)
        {
            if (!dead[i])
            {
                //clear the sensors of agent i
                for(j = 0; j < sensors * 2; j++)
                {
                    agent->states[j+(i*maxNodes)]=0;
                }
                
                //iterate for agent i over all agents j
                for(j = 0; j < hiveSize; j++)
                {
                    //ignore i==j because an agent can't see itself
                    if(i != j)
                    {
                        double d = calcDistance(x[i], y[i], x[j], y[j]);
			
                        //don't bother is an agent is too far, so we compute the distance and call it d
                        if(d < visionRange)
                        {
                            double Ux,Uy,Vx,Vy;
                            double angle;
                            //ann kathete divided by hypothenuse
                            Ux=(x[j]-x[i])/d;
                            //gegenkathete divided by hypothenuse
                            Uy=(y[j]-y[i])/d;
                            //I forgot what the line below does...
                            Vx=cos(a[i]*(cPI/180.0));
                            Vy=sin(a[i]*(cPI/180.0));
                            //anyway the following line computes the angle between my own and the object I am looking at
                            angle = atan2(((Ux*Vy)-(Uy*Vx)), ((Ux*Vx)+(Uy*Vy)))*180.0/cPI;
                            //here we have to map the angle into the sensor, btw: angle in degree
                            if(fabs(angle) < 90) // you have a 180 degree vision field infront of you
                            {
                                agent->states[(int)(angle/90.0+((double)sensors/2.0))+(i*maxNodes)]=1;
                            }
                        }
                    }
                }
                
                double d = calcDistance(x[i], y[i], mX, mY);
                //fitness function that rewards closeness and punishes too closeness
                /*if(predator_fitness && (d < killDist) && ((rand()&255)>killChance) && i != targetPrey)
                {
                    j=hiveSize;
                    for(int z=0;z<hiveSize;z++)
                        if(dead[z]) j--;
                    if(j>2)
                    {
                        dead[i]=true;
                    }
                    
                    //increase the delay to make the predator fly away for longer afte she successfully killed a target
                    delay=40.0;
                }*/
                
                if (d < visionRange)
                {
                    double Ux,Uy,Vx,Vy;
                    double angle;
                    //ann kathete divided by hypothenuse
                    Ux=(mX-x[i])/d;
                    //gegenkathete divided by hypothenuse
                    Uy=(mY-y[i])/d;
                    //I forgot what the line below does...
                    Vx=cos(a[i]*(cPI/180.0));
                    Vy=sin(a[i]*(cPI/180.0));
                    //anyway the following line computes the angle between my own and the object I am looking at
                    angle = atan2(((Ux*Vy)-(Uy*Vx)), ((Ux*Vx)+(Uy*Vy)))*180.0/cPI;
                    //here we have to map the angle into the sensor, btw: angle in degree
                    // you have a 180 degree vision field infront of you
                    if(fabs(angle)<90)
                    {
                        agent->states[sensors+((int)(angle/90.0+((double)sensors/2.0))+(i*maxNodes))]=1;
                    }
                }
            }
        }
        agent->updateStates();
        
        for(i=0;i<hiveSize;i++)
        {
            if (!dead[i])
            {
                action=((agent->states[(maxNodes-1)+(i*maxNodes)]&1)<<1)+(agent->states[(maxNodes-2)+(i*maxNodes)]&1);
                switch(action)
                {
                    case 0: 
                        break;
                    // go 2x faster
                    case 3:
                        x[i]+=cos(a[i]*(cPI/180.0));
                        y[i]+=sin(a[i]*(cPI/180.0));
                        break;
                    // turn 8 degrees right
                    case 1:
                        a[i]+=8.0;
                        while(a[i]>360.0) a[i]-=360.0;
                        break;
                    // turn 8 degrees left
                    case 2:
                        a[i]-=8.0;
                        while(a[i]<0.0) a[i]+=360.0;
                        break;
                }

                // update prey position
                x[i] += cos(a[i]*(cPI/180.0));
                y[i] += sin(a[i]*(cPI/180.0));

                // keep position within boundary
                x[i] = applyBoundary(x[i]);
                y[i] = applyBoundary(y[i]);
            }
        }

        //fitness function that leads to a moving circle
        sD = 0.0;
        pD = 0.0;
        
        // compute center of swarm
        double cX = 0, cY = 0;
        unsigned int aliveCount = 0;
        for(i = 0; i < hiveSize; i++)
        {
            if (!dead[i])
            {
                cX += x[i];
                cY += y[i];
                aliveCount++;
            }
        }
        
        cX /= (double)aliveCount;
        cY /= (double)aliveCount;
        
        for(i = 0; i < hiveSize; i++)
        {
            if (!dead[i])
            {
                // sum prey's distance from center of swarm
                float distToCenter = calcDistance(x[i], y[i], cX, cY);
                
                if (distToCenter > 20.0)
                {
                    sD += 1.0 / distToCenter;
                }
                else if (distToCenter > 10.0)
                {
                    sD += 1.0;
                }
                
                if(predator_fitness)
                {
                    // sum prey's distance from predator
                    pD += calcDistance(x[i], y[i], mX, mY);
                }
            }
        }
        
        // closer to center of swarm = better
        fitnessFromSwarming += sD;
        
        fitnessFromPredation += pD;
    }
    
    if (swarm_fitness && predator_fitness)
    {
        agent->fitness = ( pow(fitnessFromSwarming, 1.0 - p) ) * ( pow(fitnessFromPredation, p) );
    }
    else if (predator_fitness)
    {
        agent->fitness = fitnessFromPredation;
    }
    else if (swarm_fitness)
    {
        agent->fitness = fitnessFromSwarming;
    }
    
    if(agent->fitness < 0.0)
        agent->fitness = 0.0;
    
    // output to data file, if provided
    if (data_file != NULL)
    {
        // count # agents alive at end of simulation
        int aliveCount = 0;

        for(i = 0; i < hiveSize; i++)
        {
            if (!dead[i])
            {
                aliveCount += 1;
            }
        }
        
        // find final bounding box coordinates
        double luX = DBL_MAX, luY = DBL_MAX;
        double rbX = -DBL_MAX, rbY = -DBL_MAX;
        
        for(i = 0; i < hiveSize; i++)
        {
            if (!dead[i])
            {
                if (x[i] < luX)
                {
                    luX = x[i];
                }
                
                if (x[i] > rbX)
                {
                    rbX = x[i];
                }
                
                if (y[i] < luY)
                {
                    luY = y[i];
                }
                
                if (y[i] > rbY)
                {
                    rbY = y[i];
                }
            }
        }

	// compute avg variance in each swarm agent's distance to the swarm centroid
	double avgVarianceDistToCentroid = 0.0;
	vector<double> varsDistToCentroid;
    
	for (i = 0; i < hiveSize; i++)
	  {
	    varsDistToCentroid.push_back(variance(distsToCentroid[i]));
	  }
    
	avgVarianceDistToCentroid = average(varsDistToCentroid);
        
        fprintf(data_file, "%d %f %f %f %d %f %f %f %f %f %f %f %f %f %f %f %i %i %f %f\n",
                agent->born,                    // update born
                fitnessFromSwarming,            // W_s
                fitnessFromPredation,           // W_p
                agent->fitness,                 // W
                aliveCount,                     // # alive at end
                luX, luY,                       // (x1, y1) of bounding box at end
                rbX, rbY,                       // (x2, y2) of bounding box at end
                average(bbSizes),               // average bounding box size
                variance(bbSizes),              // variance in bounding box size
                average(shortestDists),         // average of avg. shortest distance to other swarm agent
                sum(sumSqrtDists),              // sum of sqrt of dist from every agent to every other agent over all updates
                average(swarmDensityCount20),   // average # of agents within 20 units of each other
                average(swarmDensityCount30),   // average # of agents within 30 units of each other
                average(swarmDensityCount40),  // average # of agents within 40 units of each other
                neuronsConnectedToPreyRetina(agent), // #neurons connected to prey part of retina
                neuronsConnectedToPredatorRetina(agent), // #neurons connected to predator part of retina
                mutualInformation(predatorAngle, preyAngle), // mutual Information between prey flight angle and predator flight angle
		avgVarianceDistToCentroid       // avg variance in each swarm agent's distance to the swarm centroid
                );
    }
    
    return reportString;
}

int tGame::selectClosestPrey(float x[], float y[], float a[], bool dead[], float mX, float mY, float mA)
{
    //int targetPrey = -1;
    //float targetPreyDist = FLT_MAX;
    vector<int> possibleTargets;
    
    for (int i = 0; i < hiveSize; i++)
    {
        if (!dead[i])
        {
          double curPreyDist = calcDistance(mX, mY, x[i], y[i]);
          
          double Ux,Uy,Vx,Vy;
          double angle;
          //ann kathete divided by hypothenuse
          Ux=(x[i]-mX)/curPreyDist;
          //gegenkathete divided by hypothenuse
          Uy=(y[i]-mY)/curPreyDist;
          //I forgot what the line below does...
          Vx=cos(mA*(cPI/180.0));
          Vy=sin(mA*(cPI/180.0));
          //anyway the following line computes the angle between my own and the object I am looking at
          angle = atan2(((Ux*Vy)-(Uy*Vx)), ((Ux*Vx)+(Uy*Vy)))*180.0/cPI;
              
          if ( fabs(angle) < 20 && curPreyDist > 0 && ( (float)rand() / (float)RAND_MAX ) < (1.0 / curPreyDist) )
          {
              possibleTargets.push_back(i);
              //targetPrey = i;
              //targetPreyDist = curPreyDist;
          }
        }
    }
    
    if (possibleTargets.size() > 0)
    {
        /*int newTarget = -1;
        float sum = 0.0;
        
        for (int i = 0; i < possibleTargets.size(); i++)
        {
            sum += (1.0 / calcDistance(mX, mY, possibleTargets[i], possibleTargets[i]));
        }
        
        float sumChoice = ((float)rand() / (float)RAND_MAX) * sum;
        
        for(int i = 0; sumChoice > 0; i++)
        {
            sumChoice -= (1.0 / calcDistance(mX, mY, possibleTargets[i], possibleTargets[i]));
            
            if (sumChoice <= 0)
                newTarget = i;
        }
        
        return newTarget;*/
        return possibleTargets[rand() % possibleTargets.size()];
    }
    else
        return -1;
}

// calculates the distance between two points
double tGame::calcDistance(float fromX, float fromY, float toX, float toY)
{
    float diffX = fromX - toX;
    float diffY = fromY - toY;
    
    return sqrt( ( diffX * diffX ) + ( diffY * diffY ) );
}

// maintains a position within the specified boundary
double tGame::applyBoundary(float positionVal)
{
    float val = positionVal;

    if (fabs(val) > maxDistFromCenter)
    {
        if (val < 0)
        {
            val = -1.0 * maxDistFromCenter;
        }
        else
        {
            val = maxDistFromCenter;
        }
    }
    
    return val;
}

// sums a vector of values
double tGame::sum(vector<double> values)
{
    double sum = 0.0;
    
    for (int i = 0; i < values.size(); i++)
    {
        sum += values[i];
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
    
    for (int i = 0; i < values.size(); i++)
    {
        sumSqDist += pow( values[i] - mean, 2.0 );
    }
    
    return sumSqDist /= (double)values.size();
}

double tGame::mutualInformation(vector<int> A,vector<int>B)
{
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i,j;
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
	int i,j;
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
        if(A->hmmus[i]->ins[j]<sensors)
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
            if((A->hmmus[i]->ins[j]<(sensors*2))&&(A->hmmus[i]->ins[j]>=sensors))
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




