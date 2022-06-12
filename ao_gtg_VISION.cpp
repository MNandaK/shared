/**
*	--- THESIS WORK ---
*	--- PROJECT N ---
* 
* 	Ao-Gtg State VISION cpp - FINAL ODDYSSEY
* 
* 	Note: ...
* 
* 	@author Nandax
* 	@date 371028, 371029
* 	@version 107agfo009
*/
#include "ao_gtg_VISION.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>

#include "SignalBeam/coordinate_conversion.h"
#include "SignalBeam/the_black_nest.h"

using std::cout;
using std::ifstream;

namespace ProjectN {

/**
 * @class AoGtgVISION
 * @brief Specific class for go to goal while avoiding obstacles, inherit State and GlobalCondition class
 */
AoGtgVISION::AoGtgVISION(string stateName):State(stateName),GlobalCondition()
{
	this->stateName = stateName;
	
	/* Fill the sense degrees */
	int numOfAngles;
	if(senseCoverage == 360.0)
		numOfAngles = this->senseCoverage / this->stepSenseDegree;
	else
		numOfAngles = (this->senseCoverage / this->stepSenseDegree) + 1;
	
	
	float an_angle = -(this->senseCoverage / 2);	//Init angle
	
	for(int ad = 0; ad < numOfAngles; ad++){
		
		this->senseDegrees.push_back(an_angle);
		an_angle += this->stepSenseDegree;
	}
}

void AoGtgVISION::calc_sense_positions(GlobalCondition cond)
{
	int numOfAngles = (int)(this->senseDegrees.size());
	
	/* Init classes */
	CoordinateConversion cc;
	vector< vector<float> > transMat = cc.trans_mat_2D((float)cond.getPoseX(), (float)cond.getPoseY(), cond.getPoseTheta());
	TheBlackNest nNest;
	vector< vector<int> > nestMap = nNest.readNest(); //This map index is (y, x)!!!
	
	this->obstacleDetected = false;
	
	/* Scan all angles */
	for(int ad = 0; ad < numOfAngles; ad++){
	
		float sense_angle_rad = cc.degreeToRad(this->senseDegrees[ad]);
		
		/* Init vectors */
		const int matColRowSize = 3;
		vector<float> senseRoboFrame(matColRowSize, 0.0);
		vector<int> senseSekaiFrame(matColRowSize-1, 0.0);
		
		/* Init sense step variables */
		float xStep = 0.0;
		float yStep = 0.0;
		float senseLength = 0.0;
		bool continueSense = true;
		
		/* Sense run */
		while(continueSense){
			
			
			senseLength = sqrt(pow(xStep, 2) + pow(yStep, 2));
			
			senseRoboFrame[0] = xStep;
			senseRoboFrame[1] = yStep;
			senseRoboFrame[2] = 1.0;
			
			/* Transform from robo frame to world frame */
			for(int row = 0; row < (matColRowSize-1); row++){
			
				float temp = 0.0;
				for(int col = 0; col < matColRowSize; col++){
					
					temp += transMat[row][col] * senseRoboFrame[col];
				}
				
				//Rounding -- not direct flooring like when cast directly to int
				/*if((int)(temp-0.5) == (int)(temp))
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

			//printf("Sense position in world frame = (%d, %d)\n", senseSekaiFrame[0], senseSekaiFrame[1]);
			if((nestMap[senseSekaiFrame[1]][senseSekaiFrame[0]]) || (senseLength > this->senseRange)){
				
				continueSense = false;
				this->sense_positions.push_back(senseSekaiFrame);
				
				if((ad == numOfAngles/2) && (senseLength < this->frontSenseRangeTolerance)) //Front sense
					this->obstacleDetected = true;
				else if(senseLength < this->senseThresSwitch) //Other senses
					this->obstacleDetected = true;
			}
			else{
				
				if(senseLength == 0.0){
					
					xStep += cos(sense_angle_rad) * 4.0;
					yStep += sin(sense_angle_rad) * 4.0;
					//printf("First sense.\n");
				}
				else{
					xStep += cos(sense_angle_rad);
					yStep += sin(sense_angle_rad);
					//printf("Continue sense...\n");
				}
			}
		}
		senseRoboFrame.clear();
		senseRoboFrame.shrink_to_fit();
		senseSekaiFrame.clear();
		senseSekaiFrame.shrink_to_fit();
	}
	transMat.clear();
	transMat.shrink_to_fit();
	
}

float AoGtgVISION::calc_err_ao(GlobalCondition cond)
{
	int numOfAngles = (int)(this->senseDegrees.size());
	
	/* Compute sense vectors */
	vector< vector<float> > senseVectors(numOfAngles, vector<float>(2, 0.0));
	for(int ad = 0; ad < numOfAngles; ad++){
		
		//Condition to use sense gain
		float tempThres = this->frontSenseDegreeThreshold;
		if((this->senseDegrees[ad] > -tempThres) && (this->senseDegrees[ad] < tempThres)){
			senseVectors[ad][0] = ((float)this->sense_positions[ad][0] - (float)cond.getPoseX());
			senseVectors[ad][1] = ((float)this->sense_positions[ad][1] - (float)cond.getPoseY());
		}
		else{
			senseVectors[ad][0] = ((float)this->sense_positions[ad][0] - (float)cond.getPoseX()) * senseGain;
			senseVectors[ad][1] = ((float)this->sense_positions[ad][1] - (float)cond.getPoseY()) * senseGain;
		}
	}
	
	/* Compute heading angle in radian */
	float tempXDirection = 0.0;
	float tempYDirection = 0.0;
	for(int ad = 0; ad < numOfAngles; ad++){
		
		tempXDirection += senseVectors[ad][0];
		tempYDirection += senseVectors[ad][1];
	}
	float theta_ao = atan2f(tempYDirection, tempXDirection);
	
	CoordinateConversion cc;
	/* Compute heading error */
	float err_ao = theta_ao - cc.degreeToRad(cond.getPoseTheta());
	err_ao = cc.radToDegree(atan2f(sin(err_ao), cos(err_ao)));
	
	//Make some move (avoidance) if the robot move perpendicular to obstacles
	this->front_sense_length = sqrt(pow(senseVectors[(numOfAngles/2)][0], 2) + pow(senseVectors[(numOfAngles/2)][1], 2));//sqrt(pow(senseVectors[(numOfAngles/2)+1][0], 2) + pow(senseVectors[(numOfAngles/2)+1][1], 2));
	srand (time(NULL));
	int randVal = (rand() % 100) + 1;
	
	if((abs(err_ao) < this->errPerpendThres) && (this->front_sense_length < this->frontSenseRangeTolerance)){
		
		if(randVal > 50)
			err_ao = 90.0;
		else
			err_ao = -90.0;
	}
	senseVectors.clear();
	senseVectors.shrink_to_fit();
	
	//cout << "Naml (x, y, theta) = (" << cond.getPoseX() << ", " << cond.getPoseY() << ", " << cond.getPoseTheta() << ");\n";
	printf("Error avoid obstacle = %f degree\n", err_ao);
	
	return err_ao;
}

float AoGtgVISION::calc_err_tg(GlobalCondition cond)
{
	vector<int> goal_coordinate = cond.getGoalCoordinate();
	float theta_tg = atan2f(goal_coordinate[1]-cond.getPoseY(), goal_coordinate[0]-cond.getPoseX());
	goal_coordinate.clear();
	goal_coordinate.shrink_to_fit();
	
	CoordinateConversion cc;
	float err_gtg = theta_tg - cc.degreeToRad(cond.getPoseTheta());
	err_gtg = cc.radToDegree(atan2f(sin(err_gtg), cos(err_gtg)));
	
	printf("Error to goal = %f degree\n", err_gtg);
	
	return err_gtg;
}

/** Calculate control signal and control PWMs **/
void AoGtgVISION::calc_ctrl_signal(DD_Robot &dd, GlobalCondition cond)
{
	float KpV;
	if(this->obstacleDetected){	//Do obstacle avoidance
		
		printf("Do obstacle avoidance!\n");
		this->err = this->calc_err_ao(cond);
		KpV = this->KpAoV;
	}
	else{						//Do go-to-goal
		
		printf("Lets move to the goal.. :D\n");
		this->err = this->calc_err_tg(cond);
		KpV = this->KpGtgV;
	}
	float ctrlSignal = KpV * this->err;
	float dutyPercentR = 0.0;
	float dutyPercentL = 0.0;
	
	if(ctrlSignal > 50.0){
		
		dd.setReverseRight(false);
		dd.setReverseLeft(true);
		
		dutyPercentR = 100.0;
		dutyPercentL = ctrlSignal - this->basePWM;
	}
	else if(ctrlSignal < -50.0){
		
		dd.setReverseRight(true);
		dd.setReverseLeft(false);
		
		dutyPercentR = -ctrlSignal - this->basePWM;
		dutyPercentL = 100.0;
	}
	else{
		
		dutyPercentR = this->basePWM + ctrlSignal;
		dutyPercentL = this->basePWM - ctrlSignal;
		
		dd.setReverseRight(false);
		dd.setReverseLeft(false);
	}
	
	if(this->obstacleDetected){
		
		dutyPercentR = dutyPercentR * (this->front_sense_length / (float)this->senseRange);
		dutyPercentL = dutyPercentL * (this->front_sense_length / (float)this->senseRange);
	}
	
	dd.setDutyRight(dutyPercentR);
	dd.setDutyLeft(dutyPercentL);
}

/** Calculate Naml-Goal distance **/
void AoGtgVISION::calc_dist_tg(GlobalCondition cond)
{
	vector<int> goal_coordinate = cond.getGoalCoordinate();
	
	this->distance_to_goal = sqrt((pow((float)(goal_coordinate[0]-cond.getPoseX()), 2) + pow((float)(goal_coordinate[1]-cond.getPoseY()), 2)));
	
	goal_coordinate.clear();
	goal_coordinate.shrink_to_fit();
}
float AoGtgVISION::getDistanceToGoal()
{
	return this->distance_to_goal;
}

float AoGtgVISION::getHeadingError()
{
	return this->err;
}

/** Run method in this state  **/
void AoGtgVISION::run(DD_Robot &dd, GlobalCondition cond)
{
	cond.updatePose();
	this->calc_sense_positions(cond);
	
	this->calc_ctrl_signal(dd, cond);
	
	this->sense_positions.clear();
	this->sense_positions.shrink_to_fit();
	
	cond.updatePose();
	this->calc_dist_tg(cond);
}

	
AoGtgVISION::~AoGtgVISION(){}

} /* namespace ProjectN */
