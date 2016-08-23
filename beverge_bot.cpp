/** @file beverage_bot.cpp
 *  @authors D. Chen
 *  @date  April 2014
 *  @brief A food/drink offering program developed for a robot to detect people through laser and interact with them.
 */
 
#include <libplayerc++/playerc++.h>
#include <libplayerc++/playerclient.h>
#include <iostream>
#include "args.h"
#include <stdlib.h>

using namespace std;
using namespace boost;

#define RAYS 32
#define SEGMENT_GAP 0.10
#define LEG_WIDTH_MAX 0.10
#define LEG_WIDTH_MIN 0.04 
#define LEG_DISTANCE_SUM_MAX 0.4
#define LEG_DISTANCE_SUM_MIN 0.25
#define LEG_JUMP_GAP 0.6
#define LEG_DIAGONAL_MIN 0
#define LEG_DIAGONAL_MAX 0.3
#define PI 3.14159265
#define CIRCLE_ANGLE_MAX 160
#define CIRCLE_ANGLE_MIN 80
#define CIRCLE_ANGLE_STD_MAX 21
#define CIRCLE_ANGLE_STD_MIN 0
#define IDLE 0	//robot rotating to find people
#define APPROACHING 1 //robot moving to the person
#define BEGIN 2	//robot whithin range and servicing a person
#define COMPLETE 3 //robot completed the service
#define YAW_SPEED_DEFAULT 0.4	//robot default rotating speed

float personArray[500][8] = {{0}};
float legArray[500][4] = {{0}};
int segmentArray[500][500] = {{0}};
int validPoints[500] = {0};
int segment = 0;
float xSpeed = 0;
float yawSpeed = 0;
int serviceState = IDLE;

mutex mtx;

using namespace PlayerCc;

// returns distance between two points
float distbtwpoints(player_point_2d_t point1, player_point_2d_t point2){
	float length = point1.px - point2.px;
	float width = point1.py - point2.py;
	float distance = (length * length) + (width * width);
	distance = sqrt(distance);

	return distance;
}

// divide detected valid points into segments
void segmentation(PlayerClient *robot, LaserProxy *lp){

	robot->Read();
	int index = 0;
	segment = 0;
	
	// initializing the segmentArray
	for (int i = 0; i <= 360; i++){
		for(int j = 0; j <= 360; j++){
			segmentArray[i][j] = 0;
		}
	}

	// Get a valid points array with all points whithin 2m
	for(int laserNum = 0; laserNum <= 360; laserNum++){
		if(lp->GetRange(laserNum) <= 2.0){
			validPoints[index] = laserNum;
			if(index < 360){
				index++;
			}	
		}	
	}

	// Put valid points into segments and store into segment array so easier to deal with in the future
	int point = 0;
	for(int pointNum = 0; pointNum < index; pointNum++){
		player_point_2d_t point1 = lp->GetPoint(validPoints[pointNum]);
		player_point_2d_t point2 = lp->GetPoint(validPoints[pointNum + 1]);
		// if two valid points are close then form a segment else two segments
		if(distbtwpoints(point1, point2) < SEGMENT_GAP){
			segmentArray[segment][point] = validPoints[pointNum];
			point++;
			if(pointNum == 359){
				segmentArray[segment][point] = validPoints[pointNum + 1];
				pointNum++;
			} 
		}

		else{
			if(point != 0){
				segmentArray[segment][point] = validPoints[pointNum];
			}
			if(pointNum == 359){
				segmentArray[segment + 1][0] = validPoints[pointNum + 1];
				pointNum++;
			}
			segment++;
			point = 0;
		}
	}
}

//The main decision making section
void moveToPeople(PlayerClient *robot, LaserProxy *lp, Position2dProxy *pp){
	bool checkWidth = false;
	bool checkLeg = false;
	bool checkCircle = false;
	int endpointindex = 0;
	int legNum = 0;
	int personNum = 0;
	float segmentWidth = 0;
	float jumpDistance = 0;
	double angleDestination = 0;
	double angleEnd = 0;
	double angleNext = 0;
	double angle = 0;
	double angleAverage = 0;
	double variance = 0;
	double stdDev = 0;
	float distEnd = 0;
	float distNext = 0;
	bool distCorrect = true;
	bool angleCorrect = true;
	int service = IDLE;
	float dist1 = 0;
	float dist2 = 0;
	float distSum = 0;
	bool checkDistSum = false;
	bool checkDiagonal = false;
	float boxLength = 0;
	float boxWidth = 0;
	float diagonal = 0;
	float length = 0;
	float lenClosest = 0;


	for(;;){
		// initializing leg array
		for(int i = 0; i <= 500; i++){
			for(int j = 0; j <= 4; j++){
				legArray[i][j] = 0;
			}
		}

		// initializing person array
		for(int i = 0; i <= 500; i++){
			for(int j = 0; j <= 8; j++){
				personArray[i][j] = 0;
			}
		}

		robot->Read();
		segmentation(robot, lp);
		xSpeed = 0;
		yawSpeed = 0;

		legNum = 0;
		// go through each segment and check their leg characterictics
		for(int i = 0; i <= 360; i++){

			
			if (segmentArray[i][0] != 0){

				// find endpoint index of the segment
				endpointindex = 0;
				while (endpointindex < 360){
					if (segmentArray[i][endpointindex + 1] != 0){
						endpointindex++;
					}
					else{
						break;
					}
				}

				player_point_2d_t startpoint = lp->GetPoint(segmentArray[i][0]);
				player_point_2d_t endpoint = lp->GetPoint(segmentArray[i][endpointindex]);
				
				// find segment width
				segmentWidth = distbtwpoints(startpoint, endpoint); 

				// check segment width with leg width characteristic
				checkWidth = false;
				if(segmentWidth > LEG_WIDTH_MIN and segmentWidth < LEG_WIDTH_MAX){
					checkWidth = true;
				}

				// Inscribed Angle Variance
				// find an average angle of all points between the start and end points in the segment
				angleAverage = 0;
				for(int j = 1; j < endpointindex; j++){
					player_point_2d_t midpoint = lp->GetPoint(segmentArray[i][j]);
					dist1 = distbtwpoints(startpoint, midpoint);
					dist2 = distbtwpoints(endpoint, midpoint);
					angle = acos((pow(dist1, 2.0) + pow(dist2, 2.0) - pow(segmentWidth, 2.0))/(2.0 * dist1 * dist2)) * 180.0 / PI;
					angleAverage = angleAverage + angle;
				}

				angleAverage = angleAverage / (endpointindex - 1);

				// find the standard deviation of these angles
				variance = 0;
				stdDev = 0;
				for(int j = 1; j < endpointindex; j++){
					player_point_2d_t midpoint = lp->GetPoint(segmentArray[i][j]);
					dist1 = distbtwpoints(startpoint, midpoint);
					dist2 = distbtwpoints(endpoint, midpoint);
					angle = acos((pow(dist1, 2.0) + pow(dist2, 2.0) - pow(segmentWidth, 2.0)) / (2.0 * dist1 * dist2)) * 180.0 / PI;
					variance = variance + pow((angle - angleAverage), 2.0);
				}

				variance = variance / (endpointindex - 1);
				stdDev = sqrt(variance);

				// check the average angle and standard deviation with leg characteristic				
				checkCircle = false;
				if ((angleAverage > CIRCLE_ANGLE_MIN and angleAverage < CIRCLE_ANGLE_MAX) and (stdDev > CIRCLE_ANGLE_STD_MIN and stdDev < CIRCLE_ANGLE_STD_MAX)){
					checkCircle = true;
				}
				
				// find sum of distances between every two consecutive points in a segment
				distSum = 0;
				for (int j = 0; j < endpointindex; j++){
					player_point_2d_t pointNow = lp->GetPoint(segmentArray[i][j]);
					player_point_2d_t pointNext = lp->GetPoint(segmentArray[i][j + 1]);
					distSum = distSum + distbtwpoints(pointNow, pointNext);
				}

				checkDistSum = false;
				// check it with leg characterictic
				if (distSum < LEG_DISTANCE_SUM_MAX and distSum > LEG_DISTANCE_SUM_MIN){
					checkDistSum = true;
				}

				// find the diagonal of a segment bounding box, the bounding box frames all the points in the segment 
				boxLength = 0;
				boxWidth = 0;
				diagonal = 0;
				length = 0;
				lenClosest = 0;

				boxWidth = startpoint.py - endpoint.py;
				if(boxWidth < 0){
					boxWidth = boxWidth * -1.0;
				}

				lenClosest = startpoint.px;
				for (int j = 0; j <= endpointindex; j++){
					if (lp->GetPoint(segmentArray[i][j]).px < lenClosest){
						lenClosest = lp->GetPoint(segmentArray[i][j]).px;
					}
				}

				for (int j = 0; j <= endpointindex; j++){
					length = lp->GetPoint(segmentArray[i][j]).px - lenClosest;
					if (length < 0){
						length = length * -1.0;
					}

					if (length > boxLength){
						boxLength = length; 
					}
				}

				diagonal = sqrt(pow(boxWidth, 2.0)+pow(boxLength, 2.0));

				// check the diagnal of the segment bounding box with leg characteric
				checkDiagonal = false;
				if(diagonal > LEG_DIAGONAL_MIN and diagonal < LEG_DIAGONAL_MAX){
					checkDiagonal = true;
				}


				
				checkLeg = false;
				// for simplication only do the Inscribed Angle Variance
				// else a more complete checking is to include all the characterics of the leg
				// and determine a final result  based on the weighting of each feature
				if(checkCircle){
					checkLeg = true;
				}
				
				// store valid segments into leg array
				if(checkLeg){
					legArray[legNum][0] = lp->GetPoint(segmentArray[i][0]).px;
					legArray[legNum][1] = lp->GetPoint(segmentArray[i][0]).py;
					legArray[legNum][2] = lp->GetPoint(segmentArray[i][endpointindex]).px;
					legArray[legNum][3] = lp->GetPoint(segmentArray[i][endpointindex]).py;
					legNum++;
				}
			}
		}
	

		personNum = 0;
		if(legNum > 1){
			for(int i = 1; i < legNum; i++){
				//find jump distance between two segments and hence detect whether they are a pair of legs or not
				jumpDistance = pow(legArray[i][0] - legArray[i - 1][2], 2.0) + pow(legArray[i][1] - legArray[i - 1][3], 2.0);
				jumpDistance = sqrt(jumpDistance);

				// if they are, store them into the person array, which contains all valid people
				if(jumpDistance < LEG_JUMP_GAP){
					for(int j = 0; j <= 3; j++){
						personArray[personNum][j] = legArray[i - 1][j]; 
						personArray[personNum][j + 4] = legArray[i][j];
					}
					personNum++;
					i++;
				}
			}
		}

		//don't move during servicing
		if (serviceState == BEGIN){
			xSpeed = 0;
			yawSpeed = 0;
		}

		else if((serviceState == IDLE or serviceState == APPROACHING) and personNum > 0){
			angleEnd = atan(personArray[0][3] / personArray[0][2]) * 180 / PI;
			angleNext = atan(personArray[0][5] / personArray[0][4]) * 180 / PI;
			angleDestination = angleEnd + (angleNext - angleEnd) / 2;
			angleCorrect = true;
			
			// rotate the robot so that it faces the person
			if(abs(angleDestination) > 2.0){
				yawSpeed = angleDestination * 2 / 100.0;
				angleCorrect = false;
			}

			distEnd = personArray[0][2];
			distNext = personArray[0][4];
			distCorrect = true;
			// if the robot is not whihtin 1m to a person then keep moving
			if(distEnd > 1.0 or distNext > 1.0){
				xSpeed = 0.3;
				distCorrect = false;
			}

			// if the robot is whithin 1m and 2 degrees to the center of the person then start servicing else keep moving 
			if (angleCorrect and distCorrect){
				service = BEGIN;
			}
			else{
				service = APPROACHING;
			}

		}
		
		// once completed a service, turn so that the robot would not see the same person again
		else if (serviceState == COMPLETE){
			service = IDLE;
			pp->SetSpeed(0, 0.8);
			sleep(3.0);
		}
		
		// if doesn't find any people then keep rotate at default speed
		else if (serviceState == IDLE and personNum == 0){
			xSpeed = 0;
			yawSpeed = YAW_SPEED_DEFAULT;
			
		}

		// if doesn't see any people then rotate at default speed and go to IDLE
		else if(personNum == 0)
		{
			xSpeed = 0;
			yawSpeed = YAW_SPEED_DEFAULT;
			service = IDLE;
		}

		mtx.lock(); // mutex lock for critical section 
		serviceState = service; // this is critical since it's the control state sharing across two threads
		mtx.unlock();


		pp->SetSpeed(xSpeed, yawSpeed);	// after decision made, finally set the speed

	}
}

void speak(SpeechProxy *speech){
	for(;;){
		if (serviceState == BEGIN){
			speech->Say("Would you like something to eat or drink");
			sleep(3.0);	// so it doesn't overflow the speech buffer

		}
		else if (serviceState == COMPLETE){
			speech->Say("Thank you");
			sleep(3.0);
		}
	}
}

void sonar(SonarProxy *son){
	for(;;){
		//if hands are whitin 30cm on each side of the robot means service completed
		if(((*son)[1] < 0.30 and (*son)[15] < 0.30 ) and serviceState == BEGIN){
			mtx.lock();
			serviceState = COMPLETE;
			mtx.unlock();
		}
	}
}

void terminalOutput(){
	for(;;){
		if(serviceState == IDLE){
			cout << "I'm looking for a person around...\n" << endl;
			sleep(1.0);	// so it doesn't overflow the terminal screen
		}
		else if(serviceState == APPROACHING){
			cout << "Found a person! Let's move to that person\n" << endl;
			sleep(1.0);
		}
		else if (serviceState == BEGIN){
			cout << "I'm servicing a person whinthin 1m\n" << endl;
			sleep(1.0);
		}
		else if (serviceState == COMPLETE){
			cout << "Service completed, Let's go find the next person\n" << endl;
			sleep(1.0);
		}
	}
}

int main(int argc, char **argv)
{
	parse_args(argc, argv); //Parses command line arguments

	try
	{
		PlayerClient robot(gHostname, gPort);
		Position2dProxy pp(&robot, gIndex);
		LaserProxy lp(&robot, gIndex);
		SonarProxy son(&robot, gIndex);
		SpeechProxy speech(&robot, gIndex);

		cout << robot << endl;
		
		pp.SetMotorEnable(true);
		pp.SetSpeed(0.0, 0.0);
		
		sleep(2.0);

		//avoid laserProxy[90] segfault on first loop
		robot.Read();

		thread move(moveToPeople, &robot, &lp, &pp);
		thread talk(speak, &speech);
		thread confirmComplete(sonar, &son);
		thread whatsUpTo(terminalOutput);

		move.join();
		talk.join();
		confirmComplete.join();
		whatsUpTo.join();

	}
	catch (PlayerCc::PlayerError e)
	{
		cerr << e << endl;
		return -1;
	}
}
