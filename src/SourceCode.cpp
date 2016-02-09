//============================================================================
// Name        : SourceCode.cpp
// Author      : Khandker M. Qaiduzzaman & Md. Aminul Islam
// Version     : 1.0
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
#include <queue>
#define EPSILON (1.0E-1)
#define NO_OF_POINTS 100

using namespace PlayerCc;
using namespace std;

static double distMat[NO_OF_POINTS][NO_OF_POINTS];
static double traveledDist = 0.0;
static double prevX = 0.0;
static double prevY = 0.0;
static double currentX = 0.0;
static double currentY = 0.0;
static int iter = 1;

PlayerClient robot("localhost", 6665);
Position2dProxy p2dProxy(&robot, 0);
RangerProxy sonarProxy(&robot, 0);
SimulationProxy simProxy(&robot, 0);

class Node;
class Item;
class Graph;

void Initialization(void);
void RefreshItemList(Item, SimulationProxy);
void CalcTraveledDist(void);
void printPath(vector<Node*>);
void DynamicPlanning(Item);
double fRand(double, double);
double EuclidianDist(Node*, Node*);
int indexOf(vector<Node*>, Node*);
int minDistance(double, bool);
unsigned GetTickCount(void);
bool isGoal(double, double, double, double);
vector<Node*> AstarAlgo(void);
vector<Node*> Dijkstra(void);
vector<Node*> GreedyBFS(void);
vector<Node*> OCFAStar(void);
vector<Node*> points;
vector<Node*> getSuccessors(vector<Node*>);
vector<double> CalcHeuristics(int);


class Item{
public:
	char name[16];
	double x;
	double y;
};

class Node {
public:

	double x;
	double y;
	double Gn;
	double Hn;
	Node *parent = NULL;
	bool visited = false;

	Node(double x, double y) {
		this->x = x;
		this->y = y;
	}

vector<Node*> getSuccessors(vector<Node*> points) {
	vector<Node*> successors;

	//Get the index from points vector
	int node_index;

	for (int i = 0; i < points.size(); i++) {
		if (points.at(i) == this) {
			node_index = i;
			break;
		}
	}

	//Get the successor nodes
	for (int i = 0; i < points.size(); i++) {
		if (distMat[node_index][i] != numeric_limits<double>::max()) {
				successors.push_back(points.at(i));
			}
		}
		return successors;
	}
};

class Graph {

public:

	void initGraph() {
		// Initialize each cell as infinity
		for (int i = 0; i < points.size(); i++) {
			for (int j = 0; j < points.size(); j++) {
				distMat[i][j] = numeric_limits<double>::max();
				distMat[j][i] = numeric_limits<double>::max();
			}
		}
	}

	void ConnectAllNodes() {

		// Connect all nodes
		for (int i = 0; i < points.size(); i++) {
			Node* node1 = points.at(i);

			for (int j = 0; j < points.size(); j++) {
				Node* node2 = points.at(j);

				if (i != j) {
					double distMeasure = EuclidianDist(node1, node2);
					distMat[i][j] = distMeasure;
					distMat[j][i] = distMeasure;
				}
			}
		}
	}
	void AssignHeuristics() {


		// h1 = Use the Euclidian Distance as Heuristics value
		/*for(int i = 0; i < points.size(); i++)
		 {
		 double heuristicDist = EuclidianDist(points.at(i), points.at(points.size()-1));
		 points.at(i)->Hn = heuristicDist;
		 }*/



		// h2 = Use the actual heuristics calculated by Dijkstra's Algorithm.
		vector<double> ActualHeuristicsList = CalcHeuristics(points.size()-1);
		 for(int i = 0; i < points.size(); i++)
		 {
		 cout << ActualHeuristicsList.at(i) << " ";
		 points.at(i)->Hn = ActualHeuristicsList.at(i);
		 }



		// h3 = h1 - h2 / NO_OF_POINTS
		/*vector<double> ActualHeuristicsList = CalcHeuristics(points.size()-1);

		 for(int i = 0; i < points.size(); i++)
		 {
		 double heuristicDist = EuclidianDist(points.at(i), points.at(points.size()-1));
		 points.at(i)->Hn = ActualHeuristicsList.at(i) - (heuristicDist/NO_OF_POINTS);
		 }*/




		// h4 = h2 - h1 / (i * NO_OF_POINTS)
		/*vector<double> ActualHeuristicsList = CalcHeuristics(points.size()-1);

    	for(int i = 0; i < points.size(); i++)
		{
		 double heuristicDist = EuclidianDist(points.at(i), points.at(points.size()-1));
		 points.at(i)->Hn = ActualHeuristicsList.at(i) - (heuristicDist/(iter*NO_OF_POINTS));
		}

		iter++;*/

	}
};


double fRand(double fMin, double fMax) {
	double f = (double) rand() / RAND_MAX;
	return fMin + f * (fMax - fMin);
}

unsigned GetTickCount() {
	struct timeval tv;
	if (gettimeofday(&tv, NULL) != 0)
		return 0;

	return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

bool isGoal(double gX, double gY, double cX, double cY) {

	double absDiffX = fabs(fabs(gX) - fabs(cX));
	double absDiffY = fabs(fabs(gY) - fabs(cY));

	if ((absDiffX < EPSILON) && (absDiffY < EPSILON)) {
		return true;
	}
	return false;
}

void RefreshItemList(Item *itemList, SimulationProxy &simProxy) {
	int i;

	//get the poses of the oranges
	for (i = 0; i < 4; i++) {
		char orangeStr[] = "orange%d";
		sprintf(itemList[i].name, orangeStr, i + 1);
		double dummy;  //dummy variable, don't need yaws.
		simProxy.GetPose2d(itemList[i].name, itemList[i].x, itemList[i].y, dummy);
	}

	return;
}

int minDistance(double dist[], bool sptSet[]) {
	// Initialize min value
	double min = numeric_limits<double>::max(), min_index;

	for (int v = 0; v < points.size(); v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}

double EuclidianDist(Node* node1, Node* node2) {
	double distance = 0.0;
	return distance = sqrt(
			pow((node1->x - node2->x), 2) + pow((node1->y - node2->y), 2));
}


int indexOf(vector<Node*> container, Node* node) {
	for (int i = 0; i < container.size(); i++) {
		if (container.at(i) == node)
			return i;
	}
	return -1;
}

void printPath(vector<Node*> waypoints) {
	cout << "\nPath is : " << endl;
	for (int i = 0; i < waypoints.size(); i++) {
		Node* node = waypoints.at(i);
		cout << "(" << node->x << "," << node->y << ")";
	}

	cout << endl;
}

void Initialization() {

	robot.Read();

	Node *start = new Node(p2dProxy.GetXPos(), p2dProxy.GetYPos());

	Node *goal = new Node(-4.0, 5.0);

	//Random Points
	points.push_back(start);

	for (int i = 0; i < NO_OF_POINTS - 2; i++) {

		double x = -6.0 + ((double) rand() / (RAND_MAX / (6.00 - (-6.00))));
		double y = -6.0 + ((double) rand() / (RAND_MAX / (6.00 - (-6.00))));
		Node *point = new Node(x, y);
		points.push_back(point);
	}
	points.push_back(goal);

}


void CalcTraveledDist() {
	currentX = p2dProxy.GetXPos();
	currentY = p2dProxy.GetYPos();

	traveledDist += sqrt(
			pow((currentX - prevX), 2) + pow((currentY - prevY), 2));
	prevX = p2dProxy.GetXPos();
	prevY = p2dProxy.GetYPos();
}

vector<double> CalcHeuristics(int source) {
	double cost[points.size()];
	bool sPathTreeSet[points.size()];

	for (int i = 0; i < points.size(); i++) {
		cost[i] = numeric_limits<double>::max();
		sPathTreeSet[i] = false;
	}

	cost[source] = 0;

	for (int count = 0; count < points.size() - 1; count++) {
		int u = minDistance(cost, sPathTreeSet);
		sPathTreeSet[u] = true;

		for (int v = 0; v < points.size(); v++) {
			if (!sPathTreeSet[v] && distMat[u][v] && (cost[u] + distMat[u][v]) < cost[v] && cost[u] != numeric_limits<double>::max()) {
				cost[v] = cost[u] + distMat[u][v];
			}
		}
	}

	vector<double> heuristics;

	for (int i = 0; i < points.size(); i++) {
		heuristics.push_back(cost[i]);
	}

	return heuristics;

}


vector<Node*> Dijkstra() {
	CalcTraveledDist();
	double dist[points.size()];
	int prev[points.size()];
	bool sptSet[points.size()];

	for (int i = 0; i < points.size(); i++) {
		dist[i] = numeric_limits<double>::max();
		sptSet[i] = false;
		prev[i] = 0;
	}

	dist[0] = 0;

	for (int count = 0; count < points.size() - 1; count++) {
		int u = minDistance(dist, sptSet);

		sptSet[u] = true;

		for (int v = 0; v < points.size(); v++) {
			if (!sptSet[v] && distMat[u][v]
					&& dist[u] != numeric_limits<double>::max()
					&& dist[u] + distMat[u][v] < dist[v]) {
				dist[v] = dist[u] + distMat[u][v];
				prev[v] = u;
			}

		}
	}

	stack<int> stk;
	vector<Node*> path;

	for (int i = 0; i < points.size(); i++) {
		if (i == points.size() - 1) {
			int j;
			stk.push(i);

			j = i;

			do {
				j = prev[j];
				stk.push(j);
				//cout << "<-" << j;

			} while (j != 0);

			cout << endl;
		}
	}

	while (!stk.empty()) {
		path.push_back(points.at(stk.top()));
		stk.pop();
	}

	return path;

}


vector<Node*> AstarAlgo() {
	CalcTraveledDist();

	Node* node_start = points.at(0);
	Node* node_goal = points.at(points.size() - 1);

	vector<Node*> open;
	vector<Node*> closed;

	open.push_back(node_start);

	node_start->Gn = 0;

	while (!open.empty()) {
		Node* node_current;
		int min_index = 0;
		double min_Fn = numeric_limits<double>::max();

		for (int i = 0; i < open.size(); i++) {
			Node* p = open.at(i);
			if ((p->Gn + p->Hn) < min_Fn) {
				min_Fn = p->Gn + p->Hn;
				min_index = i;
			}
		}

		node_current = open.at(min_index);
		open.erase(open.begin() + min_index);

		if (node_current == node_goal) {
			node_goal->parent = node_current->parent;
			break;
		}

		vector<Node*> successors = node_current->getSuccessors(points);

		//For each successor nodes perform
		for (int i = 0; i < successors.size(); i++) {
			Node* node_successor = successors.at(i);

			double successor_current_w = EuclidianDist(node_current,
					node_successor);
			double successor_current_cost = node_current->Gn
					+ successor_current_w;

			int oFound = indexOf(open, node_successor);
			int cFound = indexOf(closed, node_successor);

			if (oFound > -1) {
				if (node_successor->Gn <= successor_current_cost)
					continue;
			} else if (cFound > -1) {
				if (node_successor->Gn <= successor_current_cost)
					continue;

				open.push_back(node_successor);
				int index = indexOf(closed, node_successor);
				closed.erase(closed.begin() + index);

			} else {
				//Add node_successor to the OPEN list
				open.push_back(node_successor);

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

	while (p != NULL) {
		stack.push(p);
		p = p->parent;
	}

	while (!stack.empty()) {
		Node* node = stack.top();
		stack.pop();
		path.push_back(node);
		//cout << "(" << node->x << "," << node->y << ")";
	}

	return path;

}


vector<Node*> GreedyBFS() {

	CalcTraveledDist();

	//Take the start and goal node
	Node* node_start = points.at(0);
	Node* node_goal = points.at(points.size() - 1);

	//Main process starts
	vector<Node*> open;
	vector<Node*> closed;

	open.push_back(node_start);

	while (!open.empty()) {
		Node* node_current;
		int min_index = 0;
		double min_Fn = numeric_limits<double>::max();

		for (int i = 0; i < open.size(); i++) {
			Node* p = open.at(i);
			if (p->Hn < min_Fn) {
				min_Fn = p->Hn;
				min_index = i;
			}
		}

		node_current = open.at(min_index);
		open.erase(open.begin() + min_index);

		if (node_current == node_goal) {
			node_goal->parent = node_current->parent;
			break;
		}

		vector<Node*> successors = node_current->getSuccessors(points);

		//For each successor nodes perform
		for (int i = 0; i < successors.size(); i++) {
			Node* node_successor = successors.at(i);

			//double successor_current_w = EuclidianDist(node_current, node_successor);

			double successor_current_heuristic = node_current->Hn;

			int oFound = indexOf(open, node_successor);
			int cFound = indexOf(closed, node_successor);

			if (oFound > -1) {
				if (node_successor->Hn <= successor_current_heuristic)
					continue;
			} else if (cFound > -1) {
				if (node_successor->Hn <= successor_current_heuristic)
					continue;

				open.push_back(node_successor);
				int index = indexOf(closed, node_successor);
				closed.erase(closed.begin() + index);

			} else {
				//Add node_successor to the OPEN list
				open.push_back(node_successor);
			}
			//Set g(node_successor) = successor_current_cost

			node_successor->Hn = successor_current_heuristic;

			//Set the parent of node_successor to node_current
			node_successor->parent = node_current;
		}

		//Add node_current to the CLOSED list
		closed.push_back(node_current);

	}

	Node* p = node_goal;
	stack<Node*> stack;
	vector<Node*> path;

	while (p != NULL) {
		stack.push(p);
		p = p->parent;
	}

	while (!stack.empty()) {
		Node* node = stack.top();
		stack.pop();
		path.push_back(node);
	}

	return path;

}

vector<Node*> OCFAStar() {
	CalcTraveledDist();

	Node* node_start = points.at(0);
	Node* node_goal = points.at(points.size() - 1);

	vector<Node*> open;
	vector<Node*> closed;

	int w = 0;

	open.push_back(node_start);

	node_start->Gn = 0;

	while (!open.empty()) {
		Node* node_current;
		int min_index = 0;
		double min_Fn = numeric_limits<double>::max();

		for (int i = 0; i < open.size(); i++) {
			Node* p = open.at(i);
			if (((w * p->Gn) + p->Hn) < min_Fn) {
				min_Fn = (w * p->Gn) + p->Hn;
				min_index = i;
			}
		}

		node_current = open.at(min_index);
		open.erase(open.begin() + min_index);

		if (node_current == node_goal) {
			node_goal->parent = node_current->parent;
			break;
		}

		vector<Node*> successors = node_current->getSuccessors(points);

		//For each successor nodes perform
		for (int i = 0; i < successors.size(); i++) {
			Node* node_successor = successors.at(i);

			double successor_current_w = EuclidianDist(node_current,
					node_successor);
			double successor_current_cost = node_current->Gn
					+ successor_current_w;

			int oFound = indexOf(open, node_successor);
			int cFound = indexOf(closed, node_successor);

			if (oFound > -1) {
				if (node_successor->Gn <= successor_current_cost)
					continue;
			} else if (cFound > -1) {
				if (node_successor->Gn <= successor_current_cost)
					continue;

				open.push_back(node_successor);
				int index = indexOf(closed, node_successor);
				closed.erase(closed.begin() + index);

			} else {
				//Add node_successor to the OPEN list
				open.push_back(node_successor);

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

	while (p != NULL) {
		stack.push(p);
		p = p->parent;
	}

	while (!stack.empty()) {
		Node* node = stack.top();
		stack.pop();
		path.push_back(node);
		//cout << "(" << node->x << "," << node->y << ")";
	}

	return path;

}

void DynamicPlanning(Item *items) {

	vector<Node*> waypoints;

	points.clear();
	//Generate and initialize all points
	Initialization();

	//Create grfaph and connect the edges
	Graph *graph = new Graph();
	graph->initGraph();
	graph->ConnectAllNodes();
	graph->AssignHeuristics();

	Item *itemList = items;

	//waypoints = AstarAlgo();
	//waypoints = Dijkstra();
	//waypoints = GreedyBFS();
	waypoints = OCFAStar();

	printPath(waypoints);

	Node* nodeGoal = points.at(points.size() - 1);

	int i = 0;
	int j = 1;
	int overflow = 0;
	Node* p;
	double startTime = GetTickCount();

	bool check = false;

	while (true) {
		double currentTime = GetTickCount() - startTime;
		robot.Read();

		// Release dynamic obstacles
		/*if( currentTime >= 3000 ) //3 seconds.
		 {
		 //do
		 for(int i = 0; i < 4; i++)
		 {
		 double x = fRand(-6.00, 6.00);
		 double y = fRand(-6.00, 6.00);
		 //robot.Read();
		 if(sqrt(pow((x-p2dProxy.GetXPos()),2) + pow((y-p2dProxy.GetYPos()), 2)) > 1)
		 {
		 simProxy.SetPose2d(itemList[i].name, x, y, 0);
		 }
		 else
		 printf("imposing");
		 }
		 RefreshItemList(itemList, simProxy);
		 //Reset the timer.
		 startTime = GetTickCount();
		 }*/

		//Main Process
		if (i == 0) {
			//j = 0;
			p = waypoints.at(j);
			p2dProxy.GoTo(p->x, p->y, 0);
			i++;
		}

		if ((isGoal(p->x, p->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))) {
			printf("Checkpoint--> (x: %lf y: %lf) completed... \n", p->x, p->y);

			if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(),
					p2dProxy.GetYPos())) {
				printf("\nGoal Reached\n");
				break;
			} else {
				i = 0;
				j++;
			}
		}

		double avoidDist = 1.5;

		if ((sonarProxy[3] < avoidDist) || (sonarProxy[4] < avoidDist)) {
			printf("\nObstacle detected\n");
			//distMat[0][NO_OF_POINTS+1] = numeric_limits<double>::max();
			//distMat[NO_OF_POINTS+1][0] = numeric_limits<double>::max();

			int secondIndex = indexOf(points, waypoints[1]);

			cout << "waypoint 1 = " << waypoints[1] << "  secInd = "
					<< secondIndex << endl;

			distMat[0][secondIndex] = numeric_limits<double>::max();
			distMat[secondIndex][0] = numeric_limits<double>::max();
			overflow++;

			//waypoints = AstarAlgo();
			//waypoints = Dijkstra();
			//waypoints = GreedyBFS();
			waypoints = OCFAStar();

			printPath(waypoints);
			i = 0;
			j = 1;
			//break;

			if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(),
					p2dProxy.GetYPos())) {
				break;
			}

			if (overflow > NO_OF_POINTS) {
				DynamicPlanning(itemList);
			}

			check = true;

			continue;
		}

		if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(),
				p2dProxy.GetYPos())) {
			break;
		}

		double safeDist = 1.5;
		if ((sonarProxy[0] > safeDist) && (sonarProxy[1] > safeDist)
				&& (sonarProxy[2] > safeDist) && (sonarProxy[3] > safeDist)
				&& (sonarProxy[4] > safeDist) && (sonarProxy[5] > safeDist)
				&& (sonarProxy[6] > safeDist) && (sonarProxy[7] > safeDist)
				&& (sonarProxy[8] > safeDist) && (sonarProxy[9] > safeDist)
				&& (sonarProxy[10] > safeDist) && (sonarProxy[11] > safeDist)
				&& (sonarProxy[12] > safeDist) && (sonarProxy[13] > safeDist)
				&& (sonarProxy[14] > safeDist) && (sonarProxy[15] > safeDist)) {

			if (isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(),
					p2dProxy.GetYPos())) {
				break;
			}

			if (check == true)
				check = false, DynamicPlanning(itemList);

		}

		//DynamicPlanning(itemList, true);
	}

	/*if (!isGoal(nodeGoal->x, nodeGoal->y, p2dProxy.GetXPos(), p2dProxy.GetYPos()))
	 {
	 //distMat[0][NO_OF_POINTS+1] = numeric_limits<double>::max();
	 //distMat[NO_OF_POINTS+1][0] = numeric_limits<double>::max();
	 //DynamicPlanning(itemList, true);
	 }*/

}

int main(int argc, char *argv[]) {
	iter = 1;
	double startTime = GetTickCount();
	clock_t tStart = clock();

	srand(time(NULL));
	//double init_x = fRand(-6.00, 6.00);
	//double init_y = fRand(-6.00, 6.00);
	//cout << "robot at: (" << init_x << "," << init_y << ")"<< endl;

	simProxy.SetPose2d("iRobo", 6, -6, 120);
	//simProxy.SetPose2d("iRobo",-6 , -6, 0);

	Item itemList[8];
	RefreshItemList(itemList, simProxy);

	//enable motors
	p2dProxy.SetMotorEnable(1);
	p2dProxy.RequestGeom();
	sonarProxy.RequestGeom();

	printf("\n");

	prevX = p2dProxy.GetXPos();
	prevY = p2dProxy.GetYPos();

	DynamicPlanning(itemList);

	double currentTime = GetTickCount() - startTime;

	cout << "Time Taken: " << 2.5 * (currentTime / 1000) << endl;
	cout << "Distance Traveled = " << traveledDist << endl;
	return 0;
}

