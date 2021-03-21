/**
*	--- THESIS WORK ---
*	--- PROJECT N ---
* 
* 	Avoid Obstacle State cpp - FINAL ODDYSSEY
* 
* 	Note: ...
* 
* 	@author Nandax
* 	@date 371007, 371019, 371020, 371024-25, 371027
* 	@version 115aofo019
*/
//#include "avoid_obstacle.h"
#include "obst_avoidance_VISION.h"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cmath>
#include <cstdlib>

#include "SignalBeam/coordinate_conversion.h"
#include "SignalBeam/the_black_nest.h"

using std::cout;
using std::ifstream;

namespace ProjectN {

/**
 * @class ObstAvoidanceV
 * @brief Specific class for obstacle avoidance, inherit State and GlobalCondition class
 */
ObstAvoidanceV::ObstAvoidanceV(string stateName):State(stateName),GlobalCondition()
{
	this->stateName = stateName;
	
	/* Fill the sense degrees */
	int numOfAngles;
	if(senseCoverage == 360.0)
		numOfAngles = this->senseCoverage / this->stepSenseDegree;
	else
		numOfAngles = (this->senseCoverage / this->stepSenseDegree) + 1;
		
	float an_angle = -(this->senseCoverage / 2);
	
	for(int ad = 0; ad < numOfAngles; ad++){
		this->senseDegrees.push_back(an_angle);
		an_angle += this->stepSenseDegree;
	}
}

/* Calculate distances in some angles */
vector< vector<int> > ObstAvoidanceV::calc_sense_positions(GlobalCondition cond)
{
	//printf("Flag Sense 1\n");
	int numOfAngles = (int)(this->senseDegrees.size());
	//cout << "NumOfAngle = " << numOfAngles << "\n";
	vector< vector<int> > allSensePositions;	//sensePositions in test_sense deg.cpp
	
	//printf("Flag Sense 2\n");
	
	/* Init classes */
	CoordinateConversion cc;
	vector< vector<float> > transMat = cc.trans_mat_2D((float)cond.getPoseX(), (float)cond.getPoseY(), cond.getPoseTheta());
	TheBlackNest nNest;
	vector< vector<int> > nestMap = nNest.readNest(); //This map index is (y, x)!!!
	
	//printf("Nest map size = (%d, %d)\n", (int)nestMap[0].size(), (int)nestMap.size());
	//printf("Flag Sense 3\n");
	
	/* Scan all angles */
	for(int ad = 0; ad < numOfAngles; ad++){
	
		float sense_angle_rad = cc.degreeToRad(this->senseDegrees[ad]);
		
		//printf("Flag Sense 4\n");
		
		/* Init vectors */
		const int matColRowSize = 3;
		vector<float> senseRoboFrame(matColRowSize, 0.0);
		vector<int> senseSekaiFrame(matColRowSize-1, 0.0);
		
		/* Init sense step variables */
		float xStep = 0.0;
		float yStep = 0.0;
		float senseLength = 0.0;
		bool continueSense = true;
		
		//printf("Flag Sense 5\n");
		
		/* Sense run */
		while(continueSense){
			
			
			senseLength = sqrt(pow(xStep, 2) + pow(yStep, 2));
			
			senseRoboFrame[0] = xStep;
			senseRoboFrame[1] = yStep;
			senseRoboFrame[2] = 1.0;
			
			//printf("Flag Sense 6\n");
			
			/* Transform from robo frame to world frame */
			for(int row = 0; row < (matColRowSize-1); row++){
			
				float temp = 0.0;
				for(int col = 0; col < matColRowSize; col++){
					
					temp += transMat[row][col] * senseRoboFrame[col];
				}
				
				//Rounding -- not direct flooring like when cast directly to int
				/*
				if((int)(temp-0.5) == (int)(temp))
					temp += 1.0;*/
				temp += 0.5;
				
				//Keep sense inside nest
				if(temp < 0.0)
					temp = 0.0;
				else if((row == 0) && ((int)temp >= (int)nestMap[0].size()))	//(int)temp >= 160
					temp = (int)nestMap[0].size() - 1;	//159.0;	
				else if((row == 1) && ((int)temp >= (int)nestMap.size()))	//(int)temp >= 140
					temp = (int)nestMap.size() - 1; //139.0;
				
				senseSekaiFrame[row] = temp;
			}

			//printf("Flag Sense 7\n");
			//printf("Sense position in world frame = (%d, %d)\n", senseSekaiFrame[0], senseSekaiFrame[1]);
			if((nestMap[senseSekaiFrame[1]][senseSekaiFrame[0]]) || (senseLength > this->senseRange)){
				
				//printf("Flag Sense 7.1\n");
				continueSense = false;
				//printf("Flag Sense 7.2\n");
				allSensePositions.push_back(senseSekaiFrame);
				//printf("Flag Sense 7.3\n");
				
				//printf("Sense position in angle %.2f degree is: (%d, %d)\n", this->senseDegrees[ad], senseSekaiFrame[0], senseSekaiFrame[1]);
				
				//if(nestMap[senseSekaiFrame[1]][senseSekaiFrame[0]])
				//	printf("Hit obstacle\n");
			}
			else{
				
				if(senseLength == 0.0){
					
					xStep += cos(sense_angle_rad) * 4.0;
					yStep += sin(sense_angle_rad) * 4.0;
					//printf("First sense.\n");
				}
				else{
					xStep += cos(sense_angle_rad);	// * this->stepLength;
					yStep += sin(sense_angle_rad);	// * this->stepLength;
					//printf("Continue sense...\n");
				}
			}
			//printf("Flag Sense 8\n");		
		}
		senseRoboFrame.clear();
		senseRoboFrame.shrink_to_fit();
		senseSekaiFrame.clear();
		senseSekaiFrame.shrink_to_fit();
		//printf("Flag Sense 9\n");
	}
	transMat.clear();
	transMat.shrink_to_fit();
	
	//printf("Flag Sense 10\n");
		
	return allSensePositions;
}


/** Calculate control signal and control PWMs **/
void ObstAvoidanceV::calc_ctrl_signal(DD_Robot &dd, GlobalCondition cond)
{
	//printf("Flag ObstAvV 1\n");
	//cond.updateStatus();	//Update Naml pose
	//cond.updatePose();	//Update Naml pose
	//printf("Flag ObstAvV 2\n");
	/* Compute sense ranges positions (world frame) */
	vector< vector<int> > sensePositions = this->calc_sense_positions(cond);
	//printf("Flag ObstAvV 3\n");
	
	int numOfAngles = (int)(this->senseDegrees.size());
	//printf("Flag ObstAvV 4\n");
	
	/* Compute sense vectors */
	vector< vector<float> > senseVectors(numOfAngles, vector<float>(2, 0.0));
	for(int ad = 0; ad < numOfAngles; ad++){
		
		//Condition to use sense gain
		//if((this->senseDegrees[ad] > -(this->frontSenseDegreeThreshold)) && (this->senseDegrees[ad] < (this->frontSenseDegreeThreshold))){
		float tempThres = this->frontSenseDegreeThreshold;	//this->fSDT;
		if((this->senseDegrees[ad] > -tempThres) && (this->senseDegrees[ad] < tempThres)){
			senseVectors[ad][0] = ((float)sensePositions[ad][0] - (float)cond.getPoseX());
			senseVectors[ad][1] = ((float)sensePositions[ad][1] - (float)cond.getPoseY());
		}
		else{
			senseVectors[ad][0] = ((float)sensePositions[ad][0] - (float)cond.getPoseX()) * senseGain;
			senseVectors[ad][1] = ((float)sensePositions[ad][1] - (float)cond.getPoseY()) * senseGain;
		}
	}
	sensePositions.clear();
	sensePositions.shrink_to_fit();
	//printf("Flag ObstAvV 5\n");
	
	/* Compute heading angle in radian */
	float tempXDirection = 0.0;
	float tempYDirection = 0.0;
	for(int ad = 0; ad < numOfAngles; ad++){
		
		tempXDirection += senseVectors[ad][0];
		tempYDirection += senseVectors[ad][1];
	}
	float theta_ao = atan2f(tempYDirection, tempXDirection);
	//printf("Flag ObstAvV 6\n");
	
	CoordinateConversion cc;
	/*
	float theta_ao = cc.radToDegree(atan2f(tempYDirection, tempXDirection));
	if(theta_ao < 0.0)
		theta_ao += 360.0;
	//printf("The avoidance vector direction is to angle %f degree\n", theta_ao);
	
	//printf("Flag ObstAvV 7\n");
	
	
	this->err = theta_ao - cond.getPoseTheta();
	this->err = cc.radToDegree(atan2f(sin(cc.degreeToRad(this->err)), cos(cc.degreeToRad(this->err))));
	//if(abs(this->err) < 4.0)
	//	this->err = 0.0;
	printf("Err1 = %f\n", this->err);
	/* Compute heading error */
	//float theta_ao2 = atan2f(tempYDirection, tempXDirection);
	this->err = theta_ao - cc.degreeToRad(cond.getPoseTheta());
	this->err = cc.radToDegree(atan2f(sin(this->err), cos(this->err)));
	//printf("Err2 = %f\n", this->err);
		
	//printf("Flag ObstAvV 8\n");
	
	//Make some move (avoidance) if the robot move perpendicular to obstacles
	float frontSenseLength = sqrt(pow(senseVectors[(numOfAngles/2)+1][0], 2) + pow(senseVectors[(numOfAngles/2)+1][1], 2));
	srand (time(NULL));
	int randVal = (rand() % 100) + 1;
	//if(this->err == 0.0 && frontSenseLength < (this->senseRange - this->frontSenseRangeTolerance)){
	if((abs(this->err) < this->errPerpendThres) && (frontSenseLength < (this->senseRange - this->frontSenseRangeOffset))){
		
		if(randVal > 50)
			this->err = 90.0;
		else
			this->err = -90.0;
	}
	senseVectors.clear();
	senseVectors.shrink_to_fit();
	
	printf("Heading error is %f degree\n\n", this->err);
	
	//printf("Flag ObstAvV 9\n");
	
	/* Control:
	 * 
	 * - Right motor more speed in positive error
	 * - Left motor more speed in negative error 
	 **/
	float ctrlSignal = this->KpV * this->err;
	float dutyPercentR = 0.0;
	float dutyPercentL = 0.0;
	
	if(ctrlSignal > 50.0){
		
		dd.setReverseLeft(true);
		dd.setReverseRight(false);
		
		//dd.setDutyRight(100.0);
		dutyPercentR = 100.0;
		//dd.setDutyLeft(ctrlSignal - this->basePWM);
		dutyPercentL = ctrlSignal - this->basePWM;
	}
	else if(ctrlSignal < -50.0){	
		
		dd.setReverseRight(true);
		dd.setReverseLeft(false);
		
		//dd.setDutyLeft(100.0);
		dutyPercentL = 100.0;
		//dd.setDutyRight(-ctrlSignal - this->basePWM);
		dutyPercentR = -ctrlSignal - this->basePWM;
	}
	else{
		//dd.setDutyRight(this->basePWM + ctrlSignal);
		dutyPercentR = this->basePWM + ctrlSignal;
		//dd.setDutyLeft(this->basePWM - ctrlSignal);
		dutyPercentL = this->basePWM - ctrlSignal;
		
		dd.setReverseRight(false);
		dd.setReverseLeft(false);
	}
	
	dutyPercentR = dutyPercentR * (frontSenseLength / this->senseRange);
	dutyPercentL = dutyPercentL * (frontSenseLength / this->senseRange);
	
	dd.setDutyRight(dutyPercentR);
	dd.setDutyLeft(dutyPercentL);
}

float ObstAvoidanceV::getHeadingError()
{
	return this->err;
}

/** Run method in this state  **/
void ObstAvoidanceV::run(DD_Robot &dd, GlobalCondition cond)
{
	cond.updatePose();
	this->calc_ctrl_signal(dd, cond);
}

ObstAvoidanceV::~ObstAvoidanceV(){}

} /* namespace ProjectN */
