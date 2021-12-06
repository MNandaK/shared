//objectTrackingTutorial.cpp

//Written by  Kyle Hounslow 2013

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.

/**
*	--- THESIS WORK ---
*	--- PROJECT N ---
* 
* 	Arena overhead camera program - HAWK-EYE
* 
* 	Note: 1. The code is modified from objectTrackingTutorial.cpp 
*		  	 by Kyle Hounslow 2013
* 		  2. Save pose in grid
* 		  3. Load goal coordidate in grid
* 		  
* 
* 	@modified_by Nandax
* 	@date OV_Ph0-10
* 	@version 0107aoc109
*/

#define PI 3.14159265

//Native C
#include <cstdio>
#include <cmath>

//C++
//#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <algorithm>

//UNIX
#include <unistd.h>
#include <sys/timeb.h>

//OpenCV
#include "opencv2/highgui/highgui.hpp"
#include <cv.h>

//Created
#include "AColour.h"
#include "coordinate_conversion_double.h"


using namespace cv;
using std::vector;
//using std::cout;
using std::ofstream;
using std::ifstream;
using std::setprecision;

using std::istream_iterator;
using std::count;
using std::reverse;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 255;
int S_MIN = 0;
int S_MAX = 255;
int V_MIN = 0;
int V_MAX = 255;

//default capture width and height
/*NandaXconst*/ int FRAME_WIDTH; 	//NandaX= 640;
/*NandaXconst*/ int FRAME_HEIGHT; 	//NandaX= 480;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;

//minimum and maximum object area
const int MIN_OBJECT_AREA = 3*3;//5*5;	//NandaX= 40*40;
/*NandaXconst*/ int MAX_OBJECT_AREA;	//NandaX= FRAME_HEIGHT*FRAME_WIDTH/1.5;

//names that will appear at the top of each window
const string windowName = "HawkEye View!";//"Original Image";
//const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
//const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

//NandaX: In calibration mode, trackbar will appear
bool calibrationMode = false;

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

string intToString(int number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}

//NandaX: double to string
string floatToString(double number)
{
	std::stringstream ss;
	ss << number;
	return ss.str();
}


void createTrackbars()
{	
	//create window for trackbars
	namedWindow(trackbarWindowName,0);
	
	//create memory to store trackbar name on window
	char TrackbarName[50];
	
	sprintf( TrackbarName, "H_MIN=%d", H_MIN);
	sprintf( TrackbarName, "H_MAX=%d", H_MAX);
	sprintf( TrackbarName, "S_MIN=%d", S_MIN);
	sprintf( TrackbarName, "S_MAX=%d", S_MAX);
	sprintf( TrackbarName, "V_MIN=%d", V_MIN);
	sprintf( TrackbarName, "V_MAX=%d", V_MAX);
	
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

//void drawObject(int x,int y,Mat &frame){
/* NandaX: Used to draw start point */
void drawObject(AColour theColor, int clrIdx, Mat &frame)
{
	int x = theColor.getXCoord();
	int y = theColor.getYCoord();
	
	/*
	cv::circle(frame,cv::Point(x,y),10,cv::Scalar(0,0,255));
	cv::putText(frame,intToString(x)+ " , " + intToString(y),cv::Point(x,y+20),1,1,Scalar(0,255,0));
	*/
	
	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame,Point(x,y),20,Scalar(255,0,200),2);
	
	
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,215,255),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,215,255),2);
    //line(frame,Point(x,y),Point(x,y-25),Scalar(255,141,0),2);
    //else line(frame,Point(x,y),Point(x,0),Scalar(255,141,0),2);
    
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,215,255),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,215,255),2);
    //line(frame,Point(x,y),Point(x,y+25),Scalar(255,141,0),2);
    //else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(255,141,0),2);
    
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,215,255),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,215,255),2);
    //line(frame,Point(x,y),Point(x-25,y),Scalar(255,141,0),2);
    //else line(frame,Point(x,y),Point(0,y),Scalar(255,141,0),2);
    
    
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,215,255),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,215,255),2);
    //line(frame,Point(x,y),Point(x+25,y),Scalar(255,141,0),2);
    //else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(255,141,0),2);

	vector<double> tempPx(2, 0.0);
	tempPx[0] = x;
	tempPx[1] = y;
	
	vector<int> tempGrid(2, 0.0);
	tempGrid = pxToGridMASTER(tempPx, FRAME_HEIGHT);
	
	if(clrIdx == 1){
		putText(frame, intToString(tempGrid[0])+","+intToString(tempGrid[1]), Point(x-70,y+30), 1, 1, Scalar(255,0,200), 2);
		putText(frame, theColor.getName(), Point(x-32,y-30), 1, 1, Scalar(255,0,200), 2);
	}
	else{
		putText(frame, intToString(tempGrid[0])+","+intToString(tempGrid[1]), Point(x,y+30), 1, 1, Scalar(255,0,200), 2);
		putText(frame, theColor.getName(), Point(x,y-30), 1, 1, Scalar(255,0,200), 2);
	}
	tempGrid.clear();
	tempGrid.shrink_to_fit();
}

void morphOps(Mat &thresh)
{

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);

}

//void trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed){
void trackFilteredObject(AColour &theColor, int clrIdx, Mat threshold, Mat &cameraFeed)
{
	//int x,y;
	//NandaXFruit apple;
	//AColour color1;

	Mat temp;
	threshold.copyTo(temp);
	
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	
	//use moments method to find our filtered object
	double refArea = 0;
	
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				//if(area>MIN_OBJECT_AREA){
				if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
					theColor.setXCoord(moment.m10/area);
					theColor.setYCoord(moment.m01/area);
					
					/*x = moment.m10/area;
					y = moment.m01/area;*/

					objectFound = true;
					refArea = area;
				}
				else{
					objectFound = false;
				}
			}
			
			//let user know you found an object
			if(objectFound == true){
				
				//draw object location on screen
				//drawObject(x,y,cameraFeed);
				if(calibrationMode)
					drawObject(theColor, clrIdx, cameraFeed);
			}
		}
		else{
			putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
		}
	}
}


/** NandaX
 *  This function calculate Naml Pose (x, y, theta) in cartesian px
 ***/
vector<double> estimatePoseCPx(AColour color1, AColour color2)
{
	vector<double> temp(3, 0.0);
	double x1 = (double)(color1.getXCoord());
	double y1 = pixelYToCartesianPx((double)(color1.getYCoord()), FRAME_HEIGHT);
	double x2 = (double)(color2.getXCoord());
	double y2 = pixelYToCartesianPx((double)(color2.getYCoord()), FRAME_HEIGHT);
	
	//Estimate x
	if(x2 < x1)
		temp[0] = x2 + ((x1-x2)/2);
	else
		temp[0] = x1 + ((x2-x1)/2);
	
	//Estimate y
	if(y2 < y1)
		temp[1] = y2 + ((y1-y2)/2);
	else
		temp[1] = y1 + ((y2-y1)/2);
	
	//Calculate theta
	temp[2] = atan2((y1-y2), (x1-x2)) * 180 / PI;
	
	if(temp[2] < 0.0f)
		temp[2] = temp[2] + 360.0f;
		
	
	return temp;
}


/** NandaX
 *  This function convert Naml pose from cartesian px to grid
 ***/
vector<double> estimatePoseGrid(vector<double> poseCPx, int FRAME_HEIGHT)
{
	vector<double> temp(3, 0.0);
	vector<double> temp2(2, 0.0);
	
	vector<int> temp3(2, 0.0);
	
	temp2[0] = poseCPx[0]; //x
	temp2[1] = poseCPx[1]; //y
	
	temp2 = cartesianPxToPixel(temp2, FRAME_HEIGHT);
	
	temp3 = pxToGridMASTER(temp2, FRAME_HEIGHT);
	temp2.clear();
	temp2.shrink_to_fit();
	
	temp[0] = temp3[0];
	temp[1] = temp3[1];
	temp[2] = poseCPx[2];
	temp3.clear();
	temp3.shrink_to_fit();
	
	return temp;
}

/**
 * Calculate heading error.
 *
double calculateErrorToGoal(vector<int> goal_coordinate, vector<double> namlPose)
{
	//printf("Sebellomm\n");
	//printf("Goal: (%d, %d) \n",  goal_coordinate[0], goal_coordinate[1]);
	//printf("Pose: (%f, %f, %f)\n", namlPose[0], namlPose[1], namlPose[2]);
	
	double theta_tg = atan2(((double)(goal_coordinate[1]))-namlPose[1], ((double)(goal_coordinate[0]))-namlPose[0]);
	
	//printf("Abis theta-to-goal: %f\n", radToDegree(theta_tg));
	
	double err_tg = theta_tg - degreeToRad(namlPose[2]);
	
	//printf("Abis err_tg\n");
	
	err_tg = radToDegree(atan2(sin(err_tg), cos(err_tg)));
	
	return err_tg;
}*/

/** NandaX
 *  Read goal coordinate from goal.csv file.
 ***/
vector<int> readGoalCoordinate()
{
	vector<int> goal_coordinate;
	ifstream fileCsv;
	char ch[32];
	
	// Get the data
	//fileCsv.open("../NamlQueen/data/goal.csv", ifstream::in);
	fileCsv.open("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/goal.csv");
	
	if (fileCsv.is_open()){
		
		fileCsv.getline(ch, 32);
		
		char* ch_val = strtok(ch, ",");
		while(ch_val){
			goal_coordinate.push_back(atoi(ch_val));
			ch_val = strtok (NULL, ",");
		}
		
		fileCsv.close();
		
		printf("Read goal..\n");
	}
	else
		printf("Unable to open goal file.. :(\n");
	
	
	printf("Goal coordinate = (%d,%d)\n\n", goal_coordinate[0], goal_coordinate[1]);
	
	return goal_coordinate;
}

/** NandaX
 *  Read map from naml_nest.csv file.
 ***/
vector< vector<int> > readBlackNest()
{
	vector< vector<int> > black_nest;
	ifstream fileCsv;
	char ch[1024];
	unsigned int line_count = 0;

	fileCsv.open("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/naml_nest.csv", ifstream::in);
	
	if (fileCsv.is_open()){
		
		/**
		 * From: http://stackoverflow.com/questions/3482064/counting-the-number-of-lines-in-a-text-file
		 * by: Jerry Coffin
		 */
		/** START **/
		// new lines will be skipped unless we stop it from happening:    
		fileCsv.unsetf(std::ios_base::skipws);

		// count the newlines with an algorithm specialized for counting:
		line_count = count(istream_iterator<char>(fileCsv), istream_iterator<char>(), '\n');
		/** END **/
		
		// Add 1 more line (the last line) which is whithout '\n' character
		line_count = line_count + 1;
		
		//printf("Number of Lines: %d\n", line_count);
		
		fileCsv.close();
	}
	else
		printf("Unable to open nest map file, the 1st.. :(\n");
	
	// Get the data
	fileCsv.open("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/naml_nest.csv", ifstream::in);
	
	if (fileCsv.is_open()){

		int line = 1;
		
		while (line <= line_count){
			
			vector<int> temp_val;
			string str_line;
			
			fileCsv.getline(ch, 1024);
			
			char* ch_val = strtok(ch, ",");
			while(ch_val){
				temp_val.push_back(atoi(ch_val));
				ch_val = strtok(NULL, ",");
			}
			
			black_nest.push_back(temp_val);
			//black_nest.push(temp_val);
			temp_val.clear();
			temp_val.shrink_to_fit();
			
			line++;
		}
		fileCsv.close();
		
		reverse(black_nest.begin(), black_nest.end());
	}
	else
		printf("Unable to open nest map file, the 2nd.. :(\n");
	
	return black_nest;
}


/** NandaX
 *  This function draw Naml Pose in frame
 ***/
void drawObject2(vector<double> poseGrid, int loop, Mat &frame)
{
	//double x = pose[0];
	//double y = cartesianPxYToPixel(pose[1]);
	vector<double> temp(2, 0.0);
	temp[0] = poseGrid[0];
	temp[1] = poseGrid[1];
	
	vector<int> temp2(2, 0.0);
	temp2[0] = (int)temp[0];
	temp2[1] = (int)temp[1];
	
	temp = gridToPxMASTER(temp2, FRAME_HEIGHT);
	temp2.clear();
	temp2.shrink_to_fit();
	
	double x = temp[0];
	double y = temp[1];
	double theta = poseGrid[2];
	temp.clear();
	temp.shrink_to_fit();
	
	circle(frame,Point(x,y),13,Scalar(0,255,0),2);
	
	int addX = (int)(25*cos(degreeToRad(theta)));
	int addY = -(int)(25*sin(degreeToRad(theta)));
	line(frame,Point(x,y),Point(x+addX,y+addY),Scalar(32,32,178),3);
	
    /*
    if(y-25>0)
    line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    
    if(y+25<FRAME_HEIGHT)
    line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    
    if(x-25>0)
    line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);
	*/
	//putText(frame, floatToString(x)+","+floatToString(pixelYToCartesianPx(y))+","+floatToString(theta), Point(x,y+30), 1, 1, Scalar(0,255,0), 2);
	//putText(frame, "|"+floatToString(x)+"|", Point(x+30,y-17), 1, 1, Scalar(0,255,0), 2);
	//putText(frame, "|"+floatToString(pixelYToCartesianPx(y))+"|", Point(x+30,y), 1, 1, Scalar(0,255,0), 2);
	
	/*Nguacoooo....
	int xPixel = (int)pose[0];
	int yPixel = (int)cartesianPxYToPixel(pose[1], FRAME_HEIGHT);
	*/
	
	putText(frame, "|"+floatToString(poseGrid[0])+"|", Point(x+30,y-17), 1, 1, Scalar(0,255,0), 2);
	putText(frame, "|"+floatToString(poseGrid[1])+"|", Point(x+30,y), 1, 1, Scalar(0,255,0), 2);
	putText(frame, "|"+floatToString(theta)+"|", Point(x+30,y+17), 1, 1, Scalar(0,255,0), 2);
	
	putText(frame, "Naml", Point(x-57,y-17), 1, 1, Scalar(0,255,0), 2);
	
	//putText(frame,"LOOP: "+intToString(loop)+"; POSE: "+floatToString(poseGrid[0])+"|"+floatToString(poseGrid[1])+"|"+floatToString(theta),Point(0,50),1,2,Scalar(255,0,255),2);
}

/** NandaX
 *  This function draw Naml Trail in frame
 ***/
void drawNamlTrail(vector< vector<double> > posesGrid, Mat &frame)
{
	int numberOfTrail = (int)posesGrid.size();
	
	for(int t = 0; t < numberOfTrail; t++){
		
		vector<double> temp(2, 0.0);
		temp[0] = posesGrid[t][0];
		temp[1] = posesGrid[t][1];
		
		vector<int> temp2(2, 0.0);
		temp2[0] = (int)temp[0];
		temp2[1] = (int)temp[1];
		
		temp = gridToPxMASTER(temp2, FRAME_HEIGHT);
		temp2.clear();
		temp2.shrink_to_fit();
		
		double x = temp[0];
		double y = temp[1];
		//double theta = posesGrid[t][2];
		temp.clear();
		temp.shrink_to_fit();
		
		circle(frame,Point(x,y),10,Scalar(255,141,0),2);
		
	}
}

/** NandaX
 *  Convert goal to pixel coordinate then draw goal position.
 ***/
void drawGoalCoordinate(vector<int> goal_coordinate, Mat &frame)
{	
	vector<double> tempPx = gridToPxMASTER(goal_coordinate, FRAME_HEIGHT);
	double x = tempPx[0];
	double y = tempPx[1];
	tempPx.clear();
	tempPx.shrink_to_fit();
	
	//circle(frame, Point(x, y), 29, Scalar(255,0,0), 3);
	
	/*
	if(y-25>0)
    //line(frame,Point(x,y),Point(x-10,y-10),Scalar(0,0,255),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,0,255),2);
    
    if(y+25<FRAME_HEIGHT)
    //line(frame,Point(x,y),Point(x+10,y+10),Scalar(0,0,255),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,0,255),2);
    */
    
    if(x-25>0)
    line(frame,Point(x,y),Point(x-29,y),Scalar(255,0,0),2);
    //line(frame,Point(x,y),Point(x-10,y+10),Scalar(0,0,255),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(255,0,0),2);
    
    if(x+25<FRAME_WIDTH)
    line(frame,Point(x,y),Point(x+29,y),Scalar(255,0,0),2);
    //line(frame,Point(x,y),Point(x+10,y-10),Scalar(0,0,255),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(255,0,0),2);
    
	putText(frame, " FINISH ", Point(x-37,y-13), 1, 1, Scalar(255,0,0), 2);
}


/** NandaX
 *  Draw obstacle points.
 ***/
void drawBlackNest(vector< vector<int> > black_nest_map, Mat &frame)
{	
	int nestHeight = (int)black_nest_map.size();
	int nestWidth = (int)black_nest_map[0].size();
	
	//printf("Nest (width, height) = (%d, %d)\n", nestWidth, nestHeight);
	
	for(int h = 0; h < nestHeight; h++){
		for(int w = 0; w < nestWidth; w++){
		
			if(black_nest_map[h][w]){
			
				vector<int> tempGrid(2, 0);
				tempGrid[0] = w;
				tempGrid[1] = h;
				
				vector<double> pxCoord = gridToPxMASTER(tempGrid, FRAME_HEIGHT);
				
				
				circle(frame,Point((int)pxCoord[0],(int)pxCoord[1]),1,Scalar(0,0,255),2);
			}
		}
	}
}


/**
 * Save goal in csv file.
 **/
void write_goal(vector<int> goal)
{
	//ofstream goal_file("../NamlQueen/data/goal.csv");
	ofstream goal_file("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/goal.csv");
    if (goal_file.is_open()){
		
		goal_file << setprecision(16) << goal[0] << "," << setprecision(16) << goal[1];
		
		goal_file.close();
		
		printf("Save goal to goal.csv!\n");
    }
    else
    	printf("Cannot save goal bro..\n");
}


/** @author NandaX
 *  @date 371024
 *  @param vector<double>
 *  @brief Write pose grid (x, y, theta) naml_pose.csv file
 **/
 void write_pose(vector<double> pose)
{
	//ofstream etg_file("../NamlQueen/data/error_tg.csv");
	ofstream pose_file("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/naml_pose.csv");
    if (pose_file.is_open()){
		
		pose_file << (int)pose[0] << "\n" << (int)pose[1] << "\n" << setprecision(16) << pose[2];
		
		pose_file.close();
    }
    else
    	printf("Cannot write pose single or married, Bro..\n");
}


/** @author NandaX
 *  @date 370916
 *  @param int
 *  @brief Write pose x coordinate in pose_x.csv file
 **
void write_pose_X(int pose_x)
{
	//ofstream pose_x_file("../NamlQueen/data/pose_x.csv");
	ofstream pose_x_file("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/pose_x.csv");
    if (pose_x_file.is_open()){
		
		pose_x_file << pose_x;
		
		pose_x_file.close();
		
		//printf("Update pose to pose_x.csv!\n");
    }
    else
    	printf("Cannot write pose X bro..\n");
}*/
/** @author NandaX
 *  @date 370916
 *  @param int
 *  @brief Write pose y coordinate in pose_y.csv file
 **
void write_pose_Y(int pose_y)
{
	//ofstream pose_y_file("../NamlQueen/data/pose_y.csv");
	ofstream pose_y_file("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/pose_y.csv");
    if (pose_y_file.is_open()){
		
		pose_y_file << pose_y;
		
		pose_y_file.close();
		
		//printf("Update pose to pose_y.csv!\n");
    }
    else
    	printf("Cannot write pose Y bro..\n");
}*/
/** @author NandaX
 *  @date 370916
 *  @param double
 *  @brief Write pose theta (orientation) in pose_theta.csv file
 **
void write_pose_theta(double pose_theta)
{
	//ofstream pose_th_file("../NamlQueen/data/pose_theta.csv");
	ofstream pose_th_file("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/pose_theta.csv");
    if (pose_th_file.is_open()){
		
		pose_th_file  << setprecision(16) << pose_theta;
		
		pose_th_file.close();
		
		//printf("Update pose to pose_theta.csv!\n");
    }
    else
    	printf("Cannot write pose theta bro..\n");
}*/


/** @author NandaX
 *  @date 370916
 *  @param double
 *  @brief Write heading error (degree) in error_tg.csv file
 **
void write_heading_err(double err_tg)
{
	//ofstream etg_file("../NamlQueen/data/error_tg.csv");
	ofstream etg_file("/home/kingofgames-mk2/Coding/Thesis/BeagleBoneBlackCoding/NAML_BBB_eMMC/usr/Nandax/FINAL_ODYSSEY/data/error_tg.csv");
    if (etg_file.is_open()){
		
		etg_file << setprecision(16) << err_tg;
		
		etg_file.close();
		
		//printf("Update error to error_tg.csv!\n");
    }
    else
    	printf("Cannot write error_tg bro..\n");
}*/

/** NandaX
 *  Write pose history in pose_history.csv file
 ***/
void write_pose_history(vector<double> pose, unsigned int timeStamp)
{
	ofstream pose_file("data/pose_history.csv", std::ios_base::out | std::ios_base::app);
    if (pose_file.is_open()){
		
		pose_file << timeStamp << "," << setprecision(16) << pose[0] << "," << setprecision(16) << pose[1] << "," << setprecision(16) << pose[2] << "\n";
		
		pose_file.close();
		
		//printf("Update pose to pose_history.csv!\n");
    }
    else
    	printf("Cannot write pose history Bero..\n");
   
}



/** NandaX
 *  Main function
 ***/
int main(int argc, char** argv)
{	
	//printf("argv[1] = %s\n", argv[1]);
	
	if(atoi(argv[1]) == 1){
		calibrationMode = true;
		printf("Entering calibration moddooo!\n");
	}
	
	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	Mat threshold;
	Mat HSV;
	
	AColour color1, color2, startPoint;

	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}
	else{
		//NandaX: Green
		color1.setName("Midori");
		/* 0934 */
		//color1.setHSVMin(Scalar(49, 92, 78));
		/* 1314 - Mendung */
		//color1.setHSVMin(Scalar(44, 52, 36));
		/* 1630 - Ujan */
		//color1.setHSVMin(Scalar(44, 56, 12));
		/* 0941 - Cerah Alhamdulillah*/
		//color1.setHSVMin(Scalar(49, 92, 36));
		/* 0859 - Sedeng gak terik */
		//color1.setHSVMin(Scalar(44, 87, 18));
		/* 1006 - Panas sedeng mungkin lailatul qadar */
		//color1.setHSVMin(Scalar(73, 37, 39));
		/* MIDORI HIGH */
		//color1.setHSVMin(Scalar(44, 88, 18));
		/* MIDORI 0911 */
		color1.setHSVMin(Scalar(44, 59, 18));
		
		/* 0934 */
		//color1.setHSVMax(Scalar(87, 208, 145));
		/* 1314 - Mendung */
		//color1.setHSVMax(Scalar(94, 255, 255));
		/* 1630 - Ujan */
		//color1.setHSVMax(Scalar(94, 255, 255));
		/* 1004 - Panas sedeng mungkin lailatul qadar */
		/* MIDORI HIGH */
		color1.setHSVMax(Scalar(102, 255, 255));
		
		
		//NandaX: Blue Sky
		color2.setName("AoiSora");
		/* AOI SORA!!! */
		//color2.setHSVMin(Scalar(45, 87, 124));
		/* Mendung 1656 */
		color2.setHSVMin(Scalar(45, 87, 84));
		/* AOI SORA!!! */
		color2.setHSVMax(Scalar(118, 255, 255));
	}


    VideoCapture capture;
    capture.open(1);
    
	/*Nandax START*/ 
	FRAME_WIDTH = (int)capture.get(3);
	FRAME_HEIGHT = (int)capture.get(4);
	
	MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
	
	printf("WidthxHeight = %dx%d\n", FRAME_WIDTH, FRAME_HEIGHT);
	/*Nandax END*/
	
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	bool theFirst = true;
	unsigned int timeStamp = 0;
	
	/* Time leap */
	struct timeb start, end;
	int time_leap;
	
	/* Get goal coordinate */
	vector<int> goal_coordinate = readGoalCoordinate();
	bool adjustGoal = false;
	// X
	if(goal_coordinate[0] > 150){
		goal_coordinate[0] = 150;
		adjustGoal = true;
	}
	else if(goal_coordinate[0] < 10){
		goal_coordinate[0] = 10;
		adjustGoal = true;
	}
	// Y
	if(goal_coordinate[1] > 130){
		goal_coordinate[1] = 130;
		adjustGoal = true;
	}
	else if(goal_coordinate[1] < 10){
		goal_coordinate[1] = 10;
		adjustGoal = true;
	}
	// Save new goal
	if(adjustGoal){
		write_goal(goal_coordinate);
		printf("Save new goal! :D\n");
	}
	
	/* Prepare pose history */
	vector< vector<double> > poseHistory;
	ofstream pose_file("data/pose_history.csv");
    if (pose_file.is_open()){
		
		pose_file << "";
		
		pose_file.close();
		
		printf("Clean pose_history.csv!\n");
    }
    else
    	printf("Cannot clean pose history Bero..\n");
	
	
	/* "Infinite" loop */
	while(1){
		
		//ftime(&start);
		
		//store image to matrix
		capture.read(cameraFeed);
		
		//convert frame from BGR to HSV colorspace
		cvtColor(cameraFeed,HSV,COLOR_BGR2HSV);

		if(calibrationMode){
			//if in calibration mode, we track objects based on the HSV slider values.
			inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
			morphOps(threshold);	
			//trackFilteredObject(threshold,HSV,cameraFeed);
			trackFilteredObject(color1, 2, threshold, cameraFeed);
			
			namedWindow(windowName2, CV_WINDOW_AUTOSIZE);
			imshow(windowName2, threshold);
		}
		else{			
			inRange(HSV, color1.getHSVMin(), color1.getHSVMax(), threshold);
			morphOps(threshold);
			trackFilteredObject(color1, 2, threshold, cameraFeed);
			
			inRange(HSV, color2.getHSVMin(), color2.getHSVMax(), threshold);
			morphOps(threshold);
			trackFilteredObject(color2, 1, threshold, cameraFeed);

			//NandaX: Estimate Naml pose
			vector<double> namlPose = estimatePoseCPx(color1, color2);
			//printf("\nNaml pose in cartesian pixel is:\n(x, y, theta) = (%f, %f, %f)\n", namlPose[0], namlPose[1], namlPose[2]);
			
			//NandaX: Mark start point
			if(theFirst){
				startPoint.setName("START");
				startPoint.setXCoord(namlPose[0]);
				startPoint.setYCoord(cartesianPxYToPixel(namlPose[1], FRAME_HEIGHT));
				theFirst = false;
			}
			
			//NandaX: Convert Naml pose from cartesian px to grid
			namlPose = estimatePoseGrid(namlPose, FRAME_HEIGHT);
			//printf("\nNaml pose grid is:\n(x, y, theta) = (%f, %f, %f)\n", namlPose[0], namlPose[1], namlPose[2]);
			//double err_tg = calculateErrorToGoal(goal_coordinate, namlPose);
			//printf("Heading error = %lf\n\n", err_tg);
			
			write_pose(namlPose);
			//write_pose_X((int)(namlPose[0]));
			//write_pose_Y((int)(namlPose[1]));
			//write_pose_theta(namlPose[2]);
			
			//write_heading_err(err_tg);
			
			write_pose_history(namlPose, timeStamp);
			poseHistory.push_back(namlPose);
			
			
			
			//NandaX: Draws!
			drawNamlTrail(poseHistory, cameraFeed);
			drawBlackNest(readBlackNest(), cameraFeed);
			drawObject(startPoint, 0, cameraFeed);
			drawGoalCoordinate(goal_coordinate, cameraFeed);
			drawObject2(namlPose, timeStamp, cameraFeed);
			//printf("\n############  YO-I.....  ############\n\n");
			
			namlPose.clear();
			namlPose.shrink_to_fit();
		}
		/*
		namedWindow(windowName2, CV_WINDOW_AUTOSIZE);
		imshow(windowName2, threshold);
		*/
		namedWindow(windowName, CV_WINDOW_AUTOSIZE);
		imshow(windowName, cameraFeed);

		
		timeStamp++;
		//delay 30ms so that screen can refresh.
		//image will not appear without this waitKey() command
		//waitKey(30);
				
		waitKey(1);
		//usleep(1000);
		
		//ftime(&end);
		//time_leap = (int) (1000.0 * (end.time - start.time) + (end.millitm - start.millitm));
		
		//printf("Time leaper = %d ms.\n", time_leap);
		
	}
	
	return 0;
}
