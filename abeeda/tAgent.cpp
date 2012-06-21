/*
 *  tAgent.cpp
 *  HMMBrain
 *
 *  Created by Arend on 9/16/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include "tAgent.h"

tAgent::tAgent(){
	nrPointingAtMe=1;
	ancestor = NULL;
    predator = NULL;
	for(int i=0;i<maxNodes;i++)
    {
		states[i]=0;
		newStates[i]=0;
	}
	bestSteps=-1;
	ID=masterID;
	masterID++;
	saved=false;
	hmmus.clear();
	nrOfOffspring=0;
	retired=false;
	food=0;
    totalSteps=0;
#ifdef useANN
	ANN=new tANN;
#endif
}

tAgent::~tAgent(){
	for(int i=0;i<hmmus.size();i++)
    {
		delete hmmus[i];
    }
    
    if (predator != NULL)
    {
        //delete predator;
    }
    
	if(ancestor!=NULL)
    {
		ancestor->nrPointingAtMe--;
		if(ancestor->nrPointingAtMe==0)
        {
			delete ancestor;
        }
	}
#ifdef useANN
	delete ANN;
#endif
}

void tAgent::setupRandomAgent(int nucleotides){
	int i;
	genome.resize(nucleotides);
	for(i=0;i<nucleotides;i++)
		genome[i]=127;//rand()&255;
	ampUpStartCodons();
//	setupPhenotype();
#ifdef useANN
	ANN->setup();
#endif
}
void tAgent::loadAgent(char* filename){
	FILE *f=fopen(filename,"r+t");
	int i;
	genome.clear();
	while(!(feof(f))){
		fscanf(f,"%i	",&i);
		genome.push_back((unsigned char)(i&255));
	}
	//setupPhenotype();
}
void tAgent::loadAgentWithTrailer(char* filename){
#ifdef useANN
	ANN=new tANN;
	ANN->load(filename);
#else
	FILE *f=fopen(filename,"r+t");
	int i;
	genome.clear();
	fscanf(f,"%i	",&i);
	while(!(feof(f))){
		fscanf(f,"%i	",&i);
		genome.push_back((unsigned char)(i&255));
	}
	//setupPhenotype();
#endif
}


void tAgent::ampUpStartCodons(void){
	int i,j;
	for(i=0;i<genome.size();i++)
		genome[i]=rand()&255;
	for(i=0;i<4;i++)
	{
		j=rand()%((int)genome.size()-100);
		genome[j]=42;
		genome[j+1]=(255-42);
		for(int k=2;k<20;k++)
			genome[j+k]=rand()&255;
	}
}

void tAgent::inherit(tAgent *from,double mutationRate,int theTime){
	int nucleotides=(int)from->genome.size();
	int i,s,o,w;
	//double localMutationRate=4.0/from->genome.size();
	vector<unsigned char> buffer;
	born=theTime;
	ancestor=from;
	from->nrPointingAtMe++;
	from->nrOfOffspring++;
	genome.clear();
	genome.resize(from->genome.size());
	for(i=0;i<nucleotides;i++)
		if(((double)rand()/(double)RAND_MAX)<mutationRate)
			genome[i]=rand()&255;
		else
			genome[i]=from->genome[i];
	if((((double)rand()/(double)RAND_MAX)<0.05)&&(genome.size()<20000)){
		//duplication
		w=15+rand()&511;
		s=rand()%((int)genome.size()-w);
		o=rand()%(int)genome.size();
		buffer.clear();
		buffer.insert(buffer.begin(),genome.begin()+s,genome.begin()+s+w);
		genome.insert(genome.begin()+o,buffer.begin(),buffer.end());
	}
	if((((double)rand()/(double)RAND_MAX)<0.02)&&(genome.size()>1000)){
		//deletion
		w=15+rand()&511;
		s=rand()%((int)genome.size()-w);
		genome.erase(genome.begin()+s,genome.begin()+s+w);
	}
	//setupPhenotype();
	fitness=0.0;
#ifdef useANN
	ANN->inherit(ancestor->ANN,mutationRate);
#endif
}
void tAgent::setupPhenotype(void){
	int i;
	tHMMU *hmmu;
	if(hmmus.size()!=0)
		for(i=0;i<hmmus.size();i++)
			delete hmmus[i];
	hmmus.clear();
	for(i=0;i<genome.size();i++){
		if((genome[i]==42)&&(genome[(i+1)%genome.size()]==(255-42))){
			hmmu=new tHMMU;
			//hmmu->setupQuick(genome,i);
			hmmu->setup(genome,i);
			hmmus.push_back(hmmu);
		}
        /*
		if((genome[i]==43)&&(genome[(i+1)%genome.size()]==(255-43))){
			hmmu=new tHMMU;
			//hmmu->setup(genome,i);
			hmmu->setupQuick(genome,i);
			hmmus.push_back(hmmu);
		}
         */
	}
}
void tAgent::setupMegaPhenotype(int howMany){
	int i,j;
	tHMMU *hmmu;
	if(hmmus.size()!=0)
    {
		for(vector<tHMMU*>::iterator it = hmmus.begin(), end = hmmus.end(); it != end; ++it)
        {
			delete *it;
        }
    }
	hmmus.clear();
	for(i=0;i<genome.size();i++){
		if((genome[i]==42)&&(genome[(i+1)%genome.size()]==(255-42)))
            for(j=0;j<howMany;j++)
            {
                hmmu=new tHMMU;
                hmmu->setup(genome, i);
                //hmmu->setupQuick(genome,i);
                for(int k=0;k<4;k++){
                    hmmu->ins[k]+=(j*maxNodes);
                    hmmu->outs[k]+=(j*maxNodes);
                }
                hmmus.push_back(hmmu);
            }
        /*
         if((genome[i]==43)&&(genome[(i+1)%genome.size()]==(255-43))){
         hmmu=new tHMMU;
         //hmmu->setup(genome,i);
         hmmu->setupQuick(genome,i);
         hmmus.push_back(hmmu);
         }
         */
	}
    
}


void tAgent::retire(void){
	retired=true;
}

unsigned char * tAgent::getStatesPointer(void){
	return states;
}

void tAgent::resetBrain(void){
	for(int i=0;i<maxNodes*swarmSize;i++)
		states[i]=0;
#ifdef useANN
	ANN->resetBrain();
#endif
}

void tAgent::updateStates(void)
{
	for(vector<tHMMU*>::iterator it = hmmus.begin(), end = hmmus.end(); it != end; ++it)
    {
		(*it)->update(&states[0],&newStates[0]);
    }
    
	for(int i=0;i<maxNodes*swarmSize;i++)
    {
		states[i]=newStates[i];
		newStates[i]=0;
	}
	++totalSteps;
}

void tAgent::showBrain(void){
	for(int i=0;i<maxNodes;i++)
		cout<<(int)states[i];
	cout<<endl;
}

void tAgent::initialize(int x, int y, int d){
	//int i,j;
	//unsigned char dummy;
	xPos=x;
	yPos=y;
	direction=d;
	steps=0;
	/*
	if((rand()&1)==0){
		scramble[1]=2;
		scramble[2]=1;
	}
	*/
}

tAgent* tAgent::findLMRCA(void){
	tAgent *r,*d;
	if(ancestor==NULL)
		return NULL;
	else{
		r=ancestor;
		d=NULL;
		while(r->ancestor!=NULL){
			if(r->ancestor->nrPointingAtMe!=1)
				d=r;
			r=r->ancestor;
		}
		return d;
	}
}

void tAgent::saveFromLMRCAtoNULL(FILE *statsFile,FILE *genomeFile){
	if(ancestor!=NULL)
		ancestor->saveFromLMRCAtoNULL(statsFile,genomeFile);
	if(!saved){ 
		fprintf(statsFile,"%i	%i	%i	%f	%i	%f	%i	%i\n",ID,born,(int)genome.size(),fitness,bestSteps,(float)totalSteps/(float)nrOfOffspring,correct,incorrect);
		fprintf(genomeFile,"%i	",ID);
		for(int i=0;i<genome.size();i++)
			fprintf(genomeFile,"	%i",genome[i]);
		fprintf(genomeFile,"\n");
		saved=true;
	}
	if((saved)&&(retired)) genome.clear();
}

/*
void tAgent::saveLOD(FILE *statsFile,FILE *genomeFile){
	if(ancestor!=NULL)
		ancestor->saveLOD(statsFile,genomeFile);
#ifdef useANN
	fprintf(genomeFile,"%i	",ID);
	fprintf(statsFile,"%i	%i	%i	%f	%i	%f	%i	%i\n",ID,born,(int)genome.size(),fitness,bestSteps,(float)totalSteps/(float)nrOfOffspring,correct,incorrect);
	ANN->saveLOD(genomeFile);
#else	
	fprintf(statsFile,"%i	%i	%i	%f	%i	%f	%i	%i\n",ID,born,(int)genome.size(),fitness,bestSteps,(float)totalSteps/(float)nrOfOffspring,correct,incorrect);
	fprintf(genomeFile,"%i	",ID);
	for(int i=0;i<genome.size();i++)
		fprintf(genomeFile,"	%i",genome[i]);
	fprintf(genomeFile,"\n");
#endif
	
}*/

void tAgent::showPhenotype(void){
	for(int i=0;i<hmmus.size();i++)
		hmmus[i]->show();
	cout<<"------"<<endl;
}

void tAgent::saveToDot(char *filename, bool predator)
{
	FILE *f=fopen(filename,"w+t");
	int i,j,k,node;
	fprintf(f,"digraph brain {\n");
	fprintf(f,"	ranksep=2.0;\n");
    
    // determine which nodes to print (no connections = do not print)
    bool print_node[32];
    
    for(i = 0; i < 32; i++)
    {
        print_node[i] = false;
    }
    
    for(i=0;i<hmmus.size();i++)
    {
        for(j=0;j<hmmus[i]->ins.size();j++)
        {
            print_node[hmmus[i]->ins[j]] = true;
        }
        
        for(k=0;k<hmmus[i]->outs.size();k++)
        {
            print_node[hmmus[i]->outs[k]] = true;
        }
    }
    
    // swarm agent input layer
	for(node=0;node<12;node++)
    {
        if(print_node[node])
        {
            fprintf(f,"	%i [shape=invtriangle,style=filled,color=cornflowerblue];\n",node);
        }
    }
    
    // for prey brains
    if (!predator)
    {
        // predator input layer
        for(node=12;node<24;node++)
        {
            if(print_node[node])
            {
                fprintf(f,"	%i [shape=invtriangle,style=filled,color=red];\n",node);
            }
        }
        
        // hidden states
        for(node=24;node<30;node++)
        {
            if(print_node[node])
            {
                fprintf(f,"	%i [shape=circle,color=black];\n",node);
            }
        }
    }
    
    // for predator brains (no predator-detecting retina, more hidden states)
    else
    {
        // hidden states
        for(node=12;node<30;node++)
        {
            if(print_node[node])
            {
                fprintf(f,"	%i [shape=circle,color=black];\n",node);
            }
        }
    }
    
    // outputs
	for(node=30;node<32;node++)
    {
		fprintf(f,"	%i [shape=circle,style=filled,color=green];\n",node);
    }
    
    // connections
	for(i=0;i<hmmus.size();i++)
    {
		for(j=0;j<hmmus[i]->ins.size();j++)
        {
			for(k=0;k<hmmus[i]->outs.size();k++)
            {
				fprintf(f,"	%i	->	%i;\n",hmmus[i]->ins[j],hmmus[i]->outs[k]);
            }
		}
	}
    
    // which nodes go on the same level
    
    // inputs
	fprintf(f,"	{ rank=same; ");
    
    for(node = 0; node < 24; node++)
    {
        if(print_node[node])
        {
            fprintf(f, "%d; ", node);
        }
    }
    
    fprintf(f, "}\n");
    
    // hidden states
	fprintf(f,"	{ rank=same; ");
    
    for(node = 24; node < 30; node++)
    {
        if(print_node[node])
        {
            fprintf(f, "%d; ", node);
        }
    }
    
    fprintf(f, "}\n");
    
    // outputs
	fprintf(f,"	{ rank=same; 30; 31; }\n");
    
	fprintf(f,"}\n");
	fclose(f);
}

void tAgent::saveToDotFullLayout(char *filename){
	FILE *f=fopen(filename,"w+t");
	int i,j,k;
	fprintf(f,"digraph brain {\n");
	fprintf(f,"	ranksep=2.0;\n");
	for(i=0;i<hmmus.size();i++){
		fprintf(f,"MM_%i [shape=box]\n",i);
		for(j=0;j<hmmus[i]->ins.size();j++)
			fprintf(f,"	t0_%i -> MM_%i\n",hmmus[i]->ins[j],i);
		for(k=0;k<hmmus[i]->outs.size();k++)
			fprintf(f,"	MM_%i -> t1_%i\n",i,hmmus[i]->outs[k]);
		
	}
	fprintf(f,"}\n");
}

void tAgent::setupDots(int x, int y,double spacing){
	double xo,yo;
	int i,j,k;
	xo=(double)(x-1)*spacing;
	xo=-(xo/2.0);
	yo=(double)(y-1)*spacing;
	yo=-(yo/2.0);
	dots.resize(x*y);
	k=0;
	for(i=0;i<x;i++)
		for(j=0;j<y;j++){
//			dots[k].xPos=(double)(rand()%(int)(spacing*x))+xo;
//			dots[k].yPos=(double)(rand()%(int)(spacing*y))+yo;
			dots[k].xPos=xo+((double)i*spacing);
			dots[k].yPos=yo+((double)j*spacing);
//			cout<<dots[k].xPos<<" "<<dots[k].yPos<<endl;
			k++;
		}
}

void tAgent::saveLogicTable(char *filename)
{
    FILE *f=fopen(filename, "w");
	int i,j;
	//fprintf(f,"s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,p1,p2,p6,p8,,o1,o2\n");
    fprintf(f,"s5,s8,s23,s24,,o1,o2\n");
    
	//for(i=0;i<65536;i++)
    for(i = 0; i < 16; i++)
    {
        map<vector<int>, int> outputCounts;
        const int NUM_REPEATS = 1001;
        
        for (int repeat = 1; repeat < NUM_REPEATS; repeat++)
        {
            for(j = 0; j < 30; j++)
            {
                if (j == 5)
                {
                    if(repeat == 1)
                    {
                        fprintf(f,"%i,",(i>>0)&1);
                    }
                    
                    states[j] = (i>>0)&1;
                }
                else if (j == 6)
                {
                    if (repeat == 1)
                    {
                        fprintf(f,"%i,",(i>>1)&1);
                    }
                    
                    states[j] = (i>>1)&1;
                }
                else if (j == 14)
                {
                    if (repeat == 1)
                    {
                        fprintf(f,"%i,",(i>>2)&1);
                    }
                    
                    states[j] = (i>>2)&1;
                }
                else if (j == 18)
                {
                    if (repeat == 1)
                    {
                        fprintf(f,"%i,",(i>>3)&1);
                    }
                    
                    states[j] = (i>>3)&1;
                }
                else
                {
                    states[j] = 0;
                }
                
                /*if (j < 14)
                 {
                 fprintf(f,"%i,",(i>>j)&1);
                 states[j]=(i>>j)&1;
                 }
                 else if(j == 17)
                 {
                 fprintf(f,"%i,",(i>>14)&1);
                 states[14]=(i>>14)&1;
                 }
                 else if(j == 19)
                 {
                 fprintf(f,"%i,",(i>>15)&1);
                 states[15]=(i>>15)&1;
                 }*/
            }
            
            updateStates();
            
            vector<int> output;
            // order: 30 31
            output.push_back(states[30]);
            output.push_back(states[31]);
            
            if (outputCounts.count(output) > 0)
            {
                outputCounts[output]++;
            }
            else
            {
                outputCounts[output] = 1;
            }
            
            // all repeats completed; determine most common output
            if (repeat == (NUM_REPEATS - 1))
            {
                map<vector<int>, int>::iterator it;
                map<vector<int>, int>::iterator mostCommonOutput = outputCounts.begin();
                
                for (it = outputCounts.begin(); it != outputCounts.end(); ++it)
                {
                    if (it->second > mostCommonOutput->second)
                    {
                        mostCommonOutput = it;
                    }
                }
                
                fprintf(f, ",%i,%i\n", mostCommonOutput->first[0], mostCommonOutput->first[1]);
            }
        }
	}
    
    fclose(f);
}

void tAgent::saveGenome(const char *filename)
{
    FILE *f=fopen(filename, "w");
    
	for (int i = 0, end = (int)genome.size(); i < end; ++i)
    {
		fprintf(f, "%i	", genome[i]);
    }
    
	fprintf(f, "\n");
    
    fclose(f);
}
