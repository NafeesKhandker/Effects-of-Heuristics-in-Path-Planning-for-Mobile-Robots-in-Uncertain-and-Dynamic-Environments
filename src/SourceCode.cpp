//============================================================================
// Name        : SourceCode.cpp
// Author      : Khandker M. Qaiduzzaman
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <sys/time.h>
#include <cmath>
#include <vector>
#include <limits>
#include <stdio.h>
#include <unistd.h>
#include <libplayerc++/playerc++.h>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <stack>
#define EPSILON    (1.0E-1)
//#include <values.h>

using namespace PlayerCc;
using namespace std;

static double inc = 0;
static double distMat[1000][1000];

struct Item
{
      char name[16];
      double x;
      double y;
}typedef item_t;

struct milestones
{
	double x;
	double y;
};

double fRand(double fMin, double fMax)
{
	double f = (double) rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

unsigned GetTickCount()
{
        struct timeval tv;
        if(gettimeofday(&tv, NULL) != 0)
                return 0;

        return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

bool isGoal(double gX, double gY, double cX, double cY)
{

	double absDiffX = fabs(fabs(gX) - fabs(cX));
	double absDiffY = fabs(fabs(gY) - fabs(cY));

	if ((absDiffX < EPSILON) && (absDiffY < EPSILON))
	{
		return true;
	}
	return false;
}
/*
struct Item {
	char name[16];
	double x;
	double y;
}typedef item_t;
*/


/**
 Randomly assigns new speeds into the given addresses.
 This function will always write to the given addresses.
 @param *forwardSpeed the address of the forward speed
 variable you want this function to change.
 @param *turnSpeed the address of the turn speed variable
 you want this function to change.
 */
/*
void Wander(double *forwardSpeed, double *turnSpeed) {
	int maxSpeed = 1;
	int maxTurn = 90;
	double fspeed, tspeed;

	//fspeed is between 0 and 10
	fspeed = rand() % 11;
	//(fspeed/10) is between 0 and 1
	fspeed = (fspeed / 10) * maxSpeed;

	tspeed = rand() % (2 * maxTurn);
	tspeed = tspeed - maxTurn;

	*forwardSpeed = fspeed;
	*turnSpeed = tspeed;
}
*/

/**
 Checks sonars for obstacles and updates the given addresses
 with wheel speeds. This function will write to the addresses
 only if there is an obstacle present. Very basic obstacle avoidance only.
 @param *forwardSpeed the address of the forward speed variable
 you want this function to change.
 @param *turnSpeed the address of the turn speed variable you
 want this function to change.
 @param &sp The sonar proxy that you want this function to monitor.
 */
/*
void AvoidObstacles(double *forwardSpeed, double *turnSpeed, RangerProxy &sp) {
	//will avoid obstacles closer than 40cm
	double avoidDistance = 0.4;
	//will turn away at 60 degrees/sec
	int avoidTurnSpeed = 60;

	//left corner is sonar no. 2
	//right corner is sonar no. 3
	if (sp[2] < avoidDistance) {
		*forwardSpeed = 0;
		//turn right
		*turnSpeed = (-1) * avoidTurnSpeed;
		printf("avoiding obstacle\n");
		return;
	} else if (sp[3] < avoidDistance) {
		*forwardSpeed = 0;
		//turn left
		*turnSpeed = avoidTurnSpeed;
		printf("avoiding obstacle\n");
		return;
	} else if ((sp[0] < avoidDistance) && (sp[1] < avoidDistance)) {
		//back off a little bit
		*forwardSpeed = -0.2;
		*turnSpeed = avoidTurnSpeed;
		printf("avoiding obstacle\n");
		return;
	}

	return; //do nothing
}
*/

/**
 If blobs have been detected this function will turn the robot
 towards the largest blob. This will be the closest blob (hopefully!).
 If called this function will always overwrite information in the
 given addresses.
 @param *forwardSpeed the address of the forward speed variable
 you want this function to change.
 @param *turnSpeed the address of the turn speed variable you
 want this function to change.
 @param &bfp The blobfinder proxy that you want this function
 to monitor.
 */
/*
void MoveToItem(double *forwardSpeed, double *turnSpeed, BlobfinderProxy &bfp) {
	int i, centre;
	int noBlobs = bfp.GetCount();
	playerc_blobfinder_blob_t blob;
	int turningSpeed = 5; // in deg/s

	number of pixels away from the image centre a blob
	 can be to be in front of the robot
	int margin = 10;

	int biggestBlobArea = 0;
	int biggestBlob = 0;

	//find the largest blob
	for (i = 0; i < noBlobs; i++) {
		//get blob from proxy
		playerc_blobfinder_blob_t currBlob = bfp[i];

		// (.area is a negative cast into an unsigned int! oops.)
		if (abs((int) currBlob.area) > biggestBlobArea) {
			biggestBlob = i;
			biggestBlobArea = abs((int) currBlob.area);
		}
	}
	blob = bfp[biggestBlob];
	//printf("biggest blob is %i with area %d\n",biggestBlob,biggestBlobArea);

	// find centre of image
	centre = bfp.GetWidth() / 2;

	//adjust turn to centre the blob in image
	if the blob's centre is within some margin of the image
	 centre then move forwards, otherwise turn so that it is
	 centred.
	//printf("blob.x=%d, c=%d\n",blob.x,centre);
	//blob to the left of centre
	if (blob.x < centre - margin) {
		*forwardSpeed = 0;
		//turn left
		*turnSpeed = turningSpeed;
		//printf("turning left\n");
	}
	//blob to the right of centre
	else if (blob.x > centre + margin) {
		*forwardSpeed = 0;
		//turn right
		*turnSpeed = -turningSpeed;
		//printf("turning right\n");
	}
	//otherwise go straight ahead
	else {
		*forwardSpeed = 0.1;
		*turnSpeed = 0;
		//printf("straight on\n");
	}

	return;
}
*/

/**
 Fills the item list array with the names and positions of items
 in the simulation
 @param itemList this is the item list which contains the names
 and positions of all the items in the simulation.
 @param simProxy the simulation proxy for the Player/Stage simulation.
 */
void RefreshItemList(item_t *itemList, SimulationProxy &simProxy)
{
	int i;

	//get the poses of the oranges
	for (i = 0; i < 4; i++)
	{
		char orangeStr[] = "orange%d";
		sprintf(itemList[i].name, orangeStr, i + 1);
		double dummy;  //dummy variable, don't need yaws.
		simProxy.GetPose2d(itemList[i].name, itemList[i].x, itemList[i].y, dummy);
	}

	//get the poses of the cartons
	/*for (i = 4; i < 8; i++) {
		char cartonStr[] = "carton%d";
		sprintf(itemList[i].name, cartonStr, i - 3);
		double dummy;  //dummy variable, don't need yaws.
		simProxy.GetPose2d(itemList[i].name, itemList[i].x, itemList[i].y,
				dummy);
	}*/

	return;
}

/**
 Finds an item in the simulation which is near the robot's teeth.
 @param itemList this is the item list which contains the names and
 positions of all the items in the simulation.
 @param listLength The number of items in the simulation
 @param sim the simulation proxy for the Player/Stage simulation.
 @return returns the index of the item in the array which is within
 the robot's teeth. If no item is found then this will return -1.
 */
/*int FindItem(item_t *itemList, int listLength, SimulationProxy &sim) {

	 This function works by creating a search area just
	 in front of the robot's teeth. The search circle is a
	 fixed distance in front of the robot, and has a
	 fixed radius.
	 This function finds objects within this search circle
	 and then deletes the closest one.


	//radius of the search circle
	double radius = 0.375;

	//The distance from the centre of the robot to
	//the centre of the search circle
	double distBotToCircle = 0.625;
	double robotX, robotY, robotYaw;
	double circleX, circleY;

	//find the robot...
	sim.GetPose2d((char*) "bob1", robotX, robotY, robotYaw);

	now we find the centre of the search circle.
	 this is distBotToCircle metres from the robot's origin
	 along its yaw

	horizontal offset from robot origin
	circleX = distBotToCircle * cos(robotYaw);

	vertical offset from robot origin
	circleY = distBotToCircle * sin(robotYaw);

	//find actual centre relative to simulation.
	circleX = robotX + circleX;
	circleY = robotY + circleY;

	 to find which items are within this circle we
	 find their Euclidian distance to the circle centre.
	 Find the closest one and if it's distance is smaller than
	 the circle radius then return its index

	double smallestDist = 1000000;
	int closestItem = 0;
	int i;

	for (i = 0; i < listLength; i++) {
		double x, y, dist;

		// get manhattan distance from circle centre to item
		x = circleX - itemList[i].x;
		y = circleY - itemList[i].y;

		//find euclidian distance from circle centre to item
		dist = (x * x) + (y * y);
		dist = sqrt(dist);

		if (dist < smallestDist) {
			smallestDist = dist;
			closestItem = i;
		}
	}

	if (smallestDist > (radius + distBotToCircle)) {
		printf("no objects were close enough, false alarm!\n");
		return -1;
	}

	return closestItem;
}*/



class Node
{
    public:

    double x;
    double y;
    double Gn;
    double Hn;
    Node *parent = NULL;

    Node(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    vector<Node*> getSuccessors(vector<Node*> points)
    {
        vector<Node*> successors;

        //Get the index from points vector
        int node_index;

        for(int i = 0; i < points.size(); i++)
        {
            if(points.at(i) == this)
            {
                node_index = i;
                break;
            }
        }

        //Get the successor nodes
        for(int i = 0; i < points.size(); i++)
        {
            if(distMat[node_index][i] != numeric_limits<double>::max())
            {
                successors.push_back(points.at(i));
            }
        }
        return successors;
    }
};

double EuclidianDist(Node*, Node*);
vector<Node*> Initialization(void);
int indexOf(vector<Node*>, Node*);
void AstarAlgo(void);

int main(int argc, char *argv[])
{
	/*need to do this line in c++ only*/
	using namespace PlayerCc;

	PlayerClient robot("localhost", 6665);
	Position2dProxy p2dProxy(&robot, 0);
	RangerProxy sonarProxy(&robot, 0);
	SimulationProxy simProxy(&robot, 0);

	//BlobfinderProxy blobProxy(&robot, 0);
	//RangerProxy laserProxy(&robot, 1);
    //double forwardSpeed, turnSpeed;

	item_t itemList[8];

	RefreshItemList(itemList, simProxy);

	srand(time(NULL));

	//enable motors
	p2dProxy.SetMotorEnable(1);

	//request geometries
	p2dProxy.RequestGeom();
	sonarProxy.RequestGeom();
	//laserProxy.RequestGeom();
	//laserProxy.RequestConfigure();
	//blobfinder doesn't have geometry

	//p2dProxy.SetSpeed(0.1, dtor(0.5));
	//p2dProxy.GoTo(0,4,0);
	//p2dProxy.GoTo(4,0,0);
	//p2dProxy.SetSpeed(5, 5, 0);

	vector<milestones> waypoints;

	for (int i = 0; i < 5; i++)
	{
		double x = fRand(-6.00, 6.00);
		double y = fRand(-6.00, 6.00);

		milestones ms;
		ms.x = x;
		ms.y = y;

		waypoints.push_back(ms);

	}

	for (int i = 0; i < waypoints.size(); i++)
	{

		std::cout << waypoints.at(i).x << " " << waypoints.at(i).y << "\n";
	}

	printf("\n");

	int i = 0;
	int j = 0;
	milestones p;
	double startTime = GetTickCount();


	while (true)
	{
		double currentTime = GetTickCount() - startTime;
		robot.Read();

		if( currentTime >= 3000 ) //3 seconds.
		{
		//do
			for(int i = 0; i < 4; i++)
			{
				double x = fRand(-6.00, 6.00);
			    double y = fRand(-6.00, 6.00);
				if(sqrt(pow((x-p2dProxy.GetXPos()),2) + pow((y-p2dProxy.GetYPos()), 2)) > 1)
				{
					simProxy.SetPose2d(itemList[i].name, fRand(-6.00, 6.00), fRand(-6.00, 6.00), 0);
				}
				else
					printf("imposing");
			}
		    RefreshItemList(itemList, simProxy);
			//Reset the timer.
			startTime = GetTickCount();
		}




		if (i == 0)
		{
			//j = 0;
			p = waypoints.at(j);
			p2dProxy.GoTo(p.x, p.y, 0);
			i++;
		}

		if ((isGoal(p.x, p.y, p2dProxy.GetXPos(), p2dProxy.GetYPos())))
		{
			printf("Checkpoint %d --> x: %lf y: %lf completed... \n", j + 1,
					p.x, p.y);
			j++;
			if (j >= waypoints.size())
			{
				printf("\nGoal Reached\n");
				break;
			} else
				i = 0;

			//p2dProxy.ResetOdometry();
			//p2dProxy.GoTo(0,4,0);
			//j++;
		}

		if((sonarProxy[2] < 1.000) || (sonarProxy[3] < 1.000) || (sonarProxy[4] < 1.00) || sonarProxy[5] < 1.00)
		{
			printf("\nObstacle detected\n");
			break;
		}


	}
}


