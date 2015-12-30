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
PlayerClient robot("localhost", 6665);
Position2dProxy p2dProxy(&robot, 0);
RangerProxy sonarProxy(&robot, 0);
SimulationProxy simProxy(&robot, 0);
int noOfPoints = 50;

struct Item
{
      char name[16];
      double x;
      double y;
}typedef item_t;

/*struct milestones
{
	double x;
	double y;
};*/

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
void Initialization(void);
int indexOf(vector<Node*>, Node*);
vector<Node*> AstarAlgo(void);
vector<Node*> points;

class Graph
{

    public:

    Graph()
    {
        // Initialize each cell as infinity
        for(int i = 0; i < points.size(); i++)
        {
            for(int j = 0; j < points.size(); j++)
            {
                distMat[i][j] = numeric_limits<double>::max();
                distMat[j][i] = numeric_limits<double>::max();
            }
        }


/*
        // Connect the nodes by level
        for(int i = 0; i < points.size(); i++)
        {
            Node* node1 = points.at(i);

            double testNext = node1->x + inc;

            for(int j = 0; j < points.size(); j++)
            {
                Node* node2 = points.at(j);

                if(node2->x == testNext)
                {
                    double distMeasure = EuclidianDist(node1, node2);
                    distMat[i][j] = distMeasure;
                    distMat[j][i] = distMeasure;
                }
            }
        }*/

       // Connect all nodes
        for(int i = 0; i < points.size(); i++)
        {
            Node* node1 = points.at(i);

            for(int j = 0; j < points.size(); j++)
            {
                Node* node2 = points.at(j);

                if(i != j)
                {
                    double distMeasure = EuclidianDist(node1, node2);
                    distMat[i][j] = distMeasure;
                    distMat[j][i] = distMeasure;
                }
            }
        }
        //
        //
        //
    }
};

double EuclidianDist(Node* node1, Node* node2)
{
    double distance = 0.0;
    return distance = sqrt(pow((node1->x - node2->x), 2) + pow((node1->y - node2->y), 2));
}

void Initialization()
{
    srand (time(NULL));
    robot.Read();

    Node *start = new Node(p2dProxy.GetXPos(), p2dProxy.GetYPos());
    Node *goal = new Node(5.0, 0.0);


    //Random Points
    points.push_back(start);

    for(int i = 0; i < noOfPoints; i++)
    {
    	double x = -6.0 + ((double)rand() / (RAND_MAX / (6.00- (-6.00))));
    	double y = -6.0 + ((double)rand() / (RAND_MAX / (6.00- (-6.00))));
    	Node *point = new Node(x, y);
    	points.push_back(point);
    }
    points.push_back(goal);

    /*double distStart2Goal = EuclidianDist(start, goal);
    double noOfSegment = 3;
    inc = distStart2Goal / noOfSegment;
    double x = start->x;

    points.push_back(start);

    for(int i = 0; i < noOfSegment - 1; i++)
    {
        x = x + inc;
        int r = rand() % 4 + 1;

        for(int j = 0; j < r; j++)
        {
            double y = -6.0 + ((double)rand() / ( RAND_MAX / (6.00- (-6.00)) ) );
            //cout << x << " " << y << endl;
            Node *point = new Node(x, y);
            points.push_back(point);
        }

    }

    points.push_back(goal);*/
}


int indexOf(vector<Node*> container, Node* node)
{
    for(int i = 0; i < container.size(); i++)
    {
        if(container.at(i) == node)
            return i;
    }
    return -1;
}

vector<Node*> AstarAlgo()
{

    //Take the start and goal node
    Node* node_start = points.at(0);
    Node* node_goal = points.at(points.size()-1);

    //Print start and goal points
    cout << "Start point is: (" << node_start->x << "," << node_start->y << ")" << endl << "Goal point is : (" << node_goal->x << "," << node_goal->y << ")" << endl;

    //Print all points
    cout << endl << "All points are : " << endl;
    for(int i = 0; i < points.size(); i++)
    {
        Node* p = points.at(i);
        cout << "(" << p->x << "," << p->y << ")";
    }

    //Show the distance matrix
    cout << endl << endl << "Distance Matrix :" << endl;
    for(int i = 0; i < points.size(); i++)
    {
        for(int j = 0; j < points.size(); j++)
        {
            cout << distMat[i][j] << " ";
        }
        cout << endl;
    }

    //Main process starts
    vector<Node*> open;
    vector<Node*> closed;

    open.push_back(node_start);

    node_start->Gn = 0;
    node_start->Hn = 0; //EuclidianDist(node_start, node_goal);

    //double node_start_Fn = node_start.Gn + node_start.Hn;

    while(!open.empty())
    {
        Node* node_current;
        int min_index = 0;
        double min_Fn = numeric_limits<double>::max();

        for(int i = 0; i < open.size(); i++)
        {
            Node* p = open.at(i);
            if((p->Gn + p->Hn) < min_Fn)
            {
                min_Fn = p->Gn + p->Hn;
                min_index = i;
            }
        }

        node_current = open.at(min_index);
        open.erase(open.begin()+min_index);


        if(node_current == node_goal)
        {
            node_goal->parent = node_current->parent;
            break;
        }

        vector<Node*> successors = node_current->getSuccessors(points);



        //For each successor nodes perform
        for(int i = 0; i < successors.size(); i++)
        {
            Node* node_successor = successors.at(i);

            double successor_current_w = EuclidianDist(node_current, node_successor);
            double successor_current_cost = node_current->Gn + successor_current_w;

            int oFound = indexOf(open, node_successor);
            int cFound = indexOf(closed, node_successor);

            if(oFound > -1)
            {
                if(node_successor->Gn <= successor_current_cost)
                    continue;
            }
            else if(cFound > -1)
            {
                 if(node_successor->Gn <= successor_current_cost)
                    continue;

                 open.push_back(node_successor);
                 int index = indexOf(closed, node_successor);
                 closed.erase(closed.begin()+index);

            }
            else
            {
                //Add node_successor to the OPEN list
                open.push_back(node_successor);

				//Set h(node_successor) to be the heuristic distance to node_goal
                node_successor->Hn = 0;//EuclidianDist(node_successor, node_goal);
            }
            //Set g(node_successor) = successor_current_cost
            node_successor->Gn = successor_current_cost;

			//Set the parent of node_successor to node_current
			node_successor->parent = node_current;
        }

        //Add node_current to the CLOSED list
        closed.push_back(node_current);

    }

    Node* p = node_goal;
    stack<Node*> stack;
    vector<Node*> path;

    while(p != NULL)
    {
        stack.push(p);
        p = p->parent;
    }

    while(!stack.empty())
    {
    		Node* node = stack.top();
    	    stack.pop();
    	    path.push_back(node);
            //cout << "(" << node->x << "," << node->y << ")";
    }

    return path;

}

void printPath(vector<Node*> waypoints)
{
	cout << "\nPath is : " << endl;
	for(int i = 0; i < waypoints.size(); i++)
	{
		Node* node = waypoints.at(i);
	       cout << "(" << node->x << "," << node->y << ")";
	}

	cout<<endl;
}

void DynamicPlanning(item_t *items, bool flag)
{

	if(flag)
	{
		//Generate and initialize all points
		Initialization();

		//Create graph and connect the edges
		Graph *graph = new Graph();
	}

	item_t *itemList = items;

	vector<Node*> waypoints;
	waypoints = AstarAlgo();

	printPath(waypoints);

	Node* nodeGoal = points.at(points.size()-1);

	int i = 0;
	int j = 1;
	Node* p;
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

		//Main Process

		if (i == 0)
		{
			//j = 0;
			p = waypoints.at(j);
			p2dProxy.GoTo(p->x, p->y, 0);
			i++;
		}

		if ((isGoal(p->x, p->y, p2dProxy.GetXPos(), p2dProxy.GetYPos())))
		{
			printf("Checkpoint--> (x: %lf y: %lf) completed... \n", p->x, p->y);

			if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))
			{
				printf("\nGoal Reached\n");
				break;
			}
			else
			{
				i = 0;
				j++;
			}
		}

		if((sonarProxy[2] < 1.5) || (sonarProxy[3] < 1.5) || (sonarProxy[4] < 1.5) || (sonarProxy[5] < 1.5))
		{
			printf("\nObstacle detected\n");
			distMat[0][noOfPoints+1] = numeric_limits<double>::max();
			distMat[noOfPoints+1][0] = numeric_limits<double>::max();

		    waypoints = AstarAlgo();

			printPath(waypoints);
			i = 0;
			j = 1;
		}


	}


	/*if (!isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))
	{
		distMat[0][noOfPoints+1] = numeric_limits<double>::max();
		distMat[noOfPoints+1][0] = numeric_limits<double>::max();
		DynamicPlanning(itemList);
	}*/
}

int main(int argc, char *argv[])
{
	/*need to do this line in c++ only*/
	//using namespace PlayerCc;
	srand(time(NULL));
	simProxy.SetPose2d("iRobo", fRand(-6.00, 6.00), fRand(-6.00, 6.00), 0);




	//BlobfinderProxy blobProxy(&robot, 0);
	//RangerProxy laserProxy(&robot, 1);
    //double forwardSpeed, turnSpeed;

	item_t itemList[8];

	RefreshItemList(itemList, simProxy);



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



/*	for (int i = 0; i < 5; i++)
	{
		double x = fRand(-6.00, 6.00);
		double y = fRand(-6.00, 6.00);

		milestones ms;
		ms.x = x;
		ms.y = y;

		waypoints.push_back(ms);

	}*/
	printf("\n");

	DynamicPlanning(itemList, true);

}


