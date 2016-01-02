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
int noOfPoints = 5;

struct Item
{
      char name[16];
      double x;
      double y;
}typedef item_t;

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

	return;
}

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

int minDistance(double dist[], bool sptSet[])
{
   // Initialize min value
   double min = numeric_limits<double>::max(), min_index;

   for (int v = 0; v < points.size(); v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;

   return min_index;
}

vector<double> Dijkstra(int src)
{
     double dist[points.size()];     // The output array.  dist[i] will hold the shortest
                      // distance from src to i

     bool sptSet[points.size()]; // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized

     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < points.size(); i++)
     {
        dist[i] = numeric_limits<double>::max(), sptSet[i] = false;
     }

     // Distance of source vertex from itself is always 0
     dist[src] = 0;

     // Find shortest path for all vertices
     for (int count = 0; count < points.size()-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet);

       // Mark the picked vertex as processed
       sptSet[u] = true;

       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < points.size(); v++)
       {
         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!sptSet[v] && distMat[u][v] && dist[u] != numeric_limits<double>::max() && dist[u]+distMat[u][v] < dist[v])
         {
        	 dist[v] = dist[u] + distMat[u][v];
         }

       }
     }

     cout << "Original : " << endl;

     vector<double> heuristics;

     for(int i = 0; i < points.size(); i++)
     {
         //cout << dist[i] << " ";
         heuristics.push_back(dist[i]);
     }

     //cout << "no Of Points: " << points.size()<< endl;
     return heuristics;

}


class Graph
{

    public:

	void initGraph()
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
	}

    void ConnectAllNodes()
    {

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

    	        //Show the distance matrix
    	         /*   cout << endl << endl << "Distance Matrix :" << endl;
    	            for(int i = 0; i < points.size(); i++)
    	            {
    	                for(int j = 0; j < points.size(); j++)
    	                {
    	                    cout << distMat[i][j] << " ";
    	                }
    	                cout << endl;
    	            }*/

    }
    void AssignHeuristics()
    {
    	vector<double> heuristicsList = Dijkstra(points.size()-1);
    	for(int i = 0; i < points.size(); i++)
    	{
    		cout << heuristicsList.at(i) << " ";
    		points.at(i)->Hn = heuristicsList.at(i);
    	}
    }
};

double EuclidianDist(Node* node1, Node* node2)
{
    double distance = 0.0;
    return distance = sqrt(pow((node1->x - node2->x), 2) + pow((node1->y - node2->y), 2));
}

void Initialization()
{

	robot.Read();


    Node *start = new Node(p2dProxy.GetXPos(), p2dProxy.GetYPos());

    cout << "First Node is: " << start->x << "," << start->y << endl;

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
    //cout << "Start point is: (" << node_start->x << "," << node_start->y << ")" << endl << "Goal point is : (" << node_goal->x << "," << node_goal->y << ")" << endl;

    //Print all points
    cout << endl << "All points are : " << endl;
    for(int i = 0; i < points.size(); i++)
    {
        Node* p = points.at(i);
        cout << "(" << p->x << "," << p->y << ")";
    }

    //Show the distance matrix
    /*cout << endl << endl << "Distance Matrix :" << endl;
    for(int i = 0; i < points.size(); i++)
    {
        for(int j = 0; j < points.size(); j++)
        {
            cout << distMat[i][j] << " ";
        }
        cout << endl;
    }*/

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

float ROT_VEL = 10;

void rotateInPlace(float t, float a_mult)
{  //t being the turn angle and a_mult to modify the turning speed
  int tt = (unsigned int) (fabs(t)*float(1e6)/(ROT_VEL*a_mult));

  for(; tt>0; tt-=1000000)
  {
	p2dProxy.SetSpeed(0.0, (t>0?1:-1)*ROT_VEL*a_mult);
    usleep(std::max(std::min(1000000,tt),20000));
  }
  p2dProxy.SetSpeed(0.0,0.0);
}


void DynamicPlanning(item_t *items, bool flag)
{
	//srand (time(NULL));
	vector<Node*> waypoints;

	if(flag)
	{
		points.clear();
		//Generate and initialize all points
		Initialization();

		//Create graph and connect the edges
		Graph *graph = new Graph();
		graph->initGraph();
		graph->ConnectAllNodes();
		graph->AssignHeuristics();

		cout << "value of start to goal: "<< distMat[0][noOfPoints+1] << endl;
	}

	else
	{
		int secondIndex = indexOf(points, waypoints[1]);
		distMat[0][secondIndex] = numeric_limits<double>::max();
		distMat[secondIndex][0] = numeric_limits<double>::max();
		/*points.clear();
		Initialization();

		Graph *graph = new Graph();
		graph->initGraph();
		graph->ConnectAllNodes();

		distMat[0][noOfPoints+1] = numeric_limits<double>::max();
		distMat[noOfPoints+1][0] = numeric_limits<double>::max();*/

		cout << "value of start to goal (replan): "<< distMat[0][secondIndex] << endl;
	}

	item_t *itemList = items;

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
			    robot.Read();
				if(sqrt(pow((x-p2dProxy.GetXPos()),2) + pow((y-p2dProxy.GetYPos()), 2)) > .5)
				{
					simProxy.SetPose2d(itemList[i].name, x, y, 0);
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

		if((sonarProxy[3] < 1.5) || (sonarProxy[4] < 1.5))
		{
			printf("\nObstacle detected\n");
			//distMat[0][noOfPoints+1] = numeric_limits<double>::max();
			//distMat[noOfPoints+1][0] = numeric_limits<double>::max();

			int secondIndex = indexOf(points, waypoints[1]);

			cout<< secondIndex << endl;

			distMat[0][secondIndex] = numeric_limits<double>::max();
			distMat[secondIndex][0] = numeric_limits<double>::max();

		    waypoints = AstarAlgo();

			printPath(waypoints);
			i = 0;
			j = 1;
			//break;

			if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))
			{
				break;
			}

			continue;
		}


		if((sonarProxy[9] < 1.5) || (sonarProxy[14] < 1.5))
		{
			DynamicPlanning(itemList, true);

			if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))
			{
				break;
			}
		}
	}

	/*if (!isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))
	{
		//distMat[0][noOfPoints+1] = numeric_limits<double>::max();
		//distMat[noOfPoints+1][0] = numeric_limits<double>::max();
		//DynamicPlanning(itemList, true);
	}*/

}

int main(int argc, char *argv[])
{
	srand(time(NULL));
	double init_x = fRand(-6.00, 6.00);
	double init_y = fRand(-6.00, 6.00);
	cout << "robot at: (" << init_x << "," << init_y << ")"<< endl;

	simProxy.SetPose2d("iRobo",init_x , init_y, 0);
	//simProxy.SetPose2d("iRobo",-6 , -6, 0);

	item_t itemList[8];
	RefreshItemList(itemList, simProxy);

	//enable motors
	p2dProxy.SetMotorEnable(1);
	p2dProxy.RequestGeom();
	sonarProxy.RequestGeom();

	printf("\n");

	DynamicPlanning(itemList, true);

	/*points.clear();
			//Generate and initialize all points
	Initialization();

			//Create graph and connect the edge
	Graph *graph = new Graph();
	graph->initGraph();
	graph->ConnectAllNodes();
	graph->AssignHeuristics();

	cout << endl;
	for(int i = 0; i < points.size(); i++)
		cout << points.at(i)->Hn << " ";*/
	//rotateInPlace(90, 90);
}

