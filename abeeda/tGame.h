/*
 *  tGame.h
 *  HMMBrain
 *
 *  Created by Arend on 9/23/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
 
#ifndef _tGame_h_included_
#define _tGame_h_included_

#include "globalConst.h"
#include "tAgent.h"
#include <vector>
#include <map>
#include <set>
#include <string>

using namespace std;

#define xDim 64
#define yDim 64
#define startMazes 1
#define cPI 3.14159265

class tOctuplet{
public:
    vector<int> data;
    void loadOctuplet(FILE *f);
};

class tExperiment{
public:
    vector<tOctuplet> dropSequences,sizeSequences,selfSequences;
    vector<vector<vector<bool> > > shouldHit;
    void loadExperiment(char *filename);
    void showExperimentProtokoll(void);
    int drops(void);
    int sizes(void);
    int selves(void);
};

class tGame{
public:
    tExperiment theExperiment;
    void loadExperiment(char *filename);
    string executeGame(tAgent* agent, FILE *data_file, bool report, float p);
    tGame();
    ~tGame();
    int selectClosestPrey(float x[], float y[], float a[], bool dead[], float mX, float mY, float mA);
    double calcDistance(float fromX, float fromY, float toX, float toY);
    double applyBoundary(float positionVal);
    double sum(vector<double> values);
    double average(vector<double> values);
    double variance(vector<double> values);
    double mutualInformation(vector<int> A,vector<int>B);
    double ei(vector<int> A,vector<int> B,int theMask);
    double computeAtomicPhi(vector<int>A,int states);
    double predictiveI(vector<int>A);
    double nonPredictiveI(vector<int>A);
    double predictNextInput(vector<int>A);
    double computeR(vector<vector<int> > table,int howFarBack);
    double computeOldR(vector<vector<int> > table);
    double entropy(vector<int> list);
    int neuronsConnectedToPreyRetina(tAgent *agent);
    int neuronsConnectedToPredatorRetina(tAgent* agent);

};
#endif
