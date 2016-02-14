//============================================================================
// Name        : SourceCode.cpp
// Author      : Khandker M. Qaiduzzaman & Md. Aminul Islam
// Version     : 1.0
// Copyright   : This code is done for the thesis work entitled "Heuristic-based
//				 Path Planning in Uncertain and Dynamic Environments for Mobile
//				 Robots." The header file playerc++.h is provided by Player/Stage
//				 project (open source) available at-
//				 http://playerstage.sourceforge.net/
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
Position2dProxy positionProxy(&robot, 0);
RangerProxy snrProxy(&robot, 0);
SimulationProxy simulationProxy(&robot, 0);

class Node;
class Item;
class Graph;

void Initialization(void);
void LoadObstacles(Item);
void CalcTraveledDist(void);
void printPath(vector<Node*>);
void DynamicPlanning(Item);
double fRand(double, double);
double EuclidianDist(Node*, Node*);
int indexOf(vector<Node*>, Node*);
int minDist(double, bool);
unsigned CountTck(void);
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

		int node_index;

		for (int i = 0; i < points.size(); i++) {
			if (points.at(i) == this) {
				node_index = i;
				break;
			}
		}

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

		for (int i = 0; i < points.size(); i++) {
			for (int j = 0; j < points.size(); j++) {
				distMat[i][j] = numeric_limits<double>::max();
				distMat[j][i] = numeric_limits<double>::max();
			}
		}
	}

	void ConnectAllNodes() {

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


double funcRand(double min, double max) {
	double r = (double) rand() / RAND_MAX;
	double rand = min + r * (max - min);
	return rand;
}
unsigned CountTck() {
	struct timeval tvalue;
	if (gettimeofday(&tvalue, NULL) != 0)
		return 0;
	unsigned tv1 = tvalue.tv_sec * 1000;
	unsigned tv2 = tvalue.tv_usec / 1000;
	return tv1 + tv2;
}

bool isGoal(double gX, double gY, double cX, double cY) {

	double absDiffX = fabs(fabs(gX) - fabs(cX));
	double absDiffY = fabs(fabs(gY) - fabs(cY));

	if ((absDiffX < EPSILON) && (absDiffY < EPSILON)) {
		return true;
	}
	return false;
}

void LoadObstacles(Item *obstaclesList) {
	int i;

	for (i = 0; i < 4; i++) {
		double empty;
		char obstaclesStr[] = "dynObs%d";
		sprintf(obstaclesList[i].name, obstaclesStr, i + 1);
		simulationProxy.GetPose2d(obstaclesList[i].name, obstaclesList[i].x, obstaclesList[i].y, empty);
	}

	return;
}

int minDist(double dist[], bool sPathTreeSet[]) {

	double minIndex = 0.0;
	double min = numeric_limits<double>::max();

	for (int v = 0; v < points.size(); v++)
		if (dist[v] <= min && sPathTreeSet[v] == false){
			min = dist[v];
			minIndex = v;
		}

	return minIndex;
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

	Node *start = new Node(positionProxy.GetXPos(), positionProxy.GetYPos());

	Node *goal = new Node(-4.0, 5.0);

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
	currentX = positionProxy.GetXPos();
	currentY = positionProxy.GetYPos();

	traveledDist += sqrt(
			pow((currentX - prevX), 2) + pow((currentY - prevY), 2));
	prevX = positionProxy.GetXPos();
	prevY = positionProxy.GetYPos();
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
		int u = minDist(cost, sPathTreeSet);
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
	bool sPathTreeSet[points.size()];

	for (int i = 0; i < points.size(); i++) {
		dist[i] = numeric_limits<double>::max();
		sPathTreeSet[i] = false;
		prev[i] = 0;
	}

	dist[0] = 0;

	for (int count = 0; count < points.size() - 1; count++) {
		int u = minDist(dist, sPathTreeSet);

		sPathTreeSet[u] = true;

		for (int v = 0; v < points.size(); v++) {
			if (!sPathTreeSet[v] && distMat[u][v]
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

	Node* nodeStart = points.at(0);
	Node* nodeGoal = points.at(points.size() - 1);

	vector<Node*> openList;
	vector<Node*> closedList;

	openList.push_back(nodeStart);

	nodeStart->Gn = 0;

	while (!openList.empty()) {
		Node* nodeCurrent;
		int minIndex = 0;
		double minFn = numeric_limits<double>::max();

		for (int i = 0; i < openList.size(); i++) {
			Node* p = openList.at(i);
			if ((p->Gn + p->Hn) < minFn) {
				minFn = p->Gn + p->Hn;
				minIndex = i;
			}
		}

		nodeCurrent = openList.at(minIndex);
		openList.erase(openList.begin() + minIndex);

		if (nodeCurrent == nodeGoal) {
			nodeGoal->parent = nodeCurrent->parent;
			break;
		}

		vector<Node*> successors = nodeCurrent->getSuccessors(points);

		for (int i = 0; i < successors.size(); i++) {
			Node* nodeSuccessor = successors.at(i);

			double successorCurrentDist = EuclidianDist(nodeCurrent,
					nodeSuccessor);
			double successorCurrentCost = nodeCurrent->Gn
					+ successorCurrentDist;

			int openFound = indexOf(openList, nodeSuccessor);
			int closedFound = indexOf(closedList, nodeSuccessor);

			if (openFound > -1) {
				if (nodeSuccessor->Gn <= successorCurrentCost)
					continue;
			} else if (closedFound > -1) {
				if (nodeSuccessor->Gn <= successorCurrentCost)
					continue;

				openList.push_back(nodeSuccessor);
				int index = indexOf(closedList, nodeSuccessor);
				closedList.erase(closedList.begin() + index);

			} else {
				openList.push_back(nodeSuccessor);

			}

			nodeSuccessor->Gn = successorCurrentCost;
			nodeSuccessor->parent = nodeCurrent;
		}

		closedList.push_back(nodeCurrent);

	}

	Node* p = nodeGoal;
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


vector<Node*> GreedyBFS() {

	CalcTraveledDist();

	Node* nodeStart = points.at(0);
	Node* nodeGoal = points.at(points.size() - 1);

	vector<Node*> openList;
	vector<Node*> closedList;

	openList.push_back(nodeStart);

	while (!openList.empty()) {
		Node* nodeCurrent;
		int minIndex = 0;
		double minFn = numeric_limits<double>::max();

		for (int i = 0; i < openList.size(); i++) {
			Node* p = openList.at(i);
			if (p->Hn < minFn) {
				minFn = p->Hn;
				minIndex = i;
			}
		}

		nodeCurrent = openList.at(minIndex);
		openList.erase(openList.begin() + minIndex);

		if (nodeCurrent == nodeGoal) {
			nodeGoal->parent = nodeCurrent->parent;
			break;
		}

		vector<Node*> successors = nodeCurrent->getSuccessors(points);

		for (int i = 0; i < successors.size(); i++) {
			Node* nodeSuccessor = successors.at(i);

			double successor_current_heuristic = nodeCurrent->Hn;

			int openFound = indexOf(openList, nodeSuccessor);
			int closedFound = indexOf(closedList, nodeSuccessor);

			if (openFound > -1) {
				if (nodeSuccessor->Hn <= successor_current_heuristic)
					continue;
			} else if (closedFound > -1) {
				if (nodeSuccessor->Hn <= successor_current_heuristic)
					continue;

				openList.push_back(nodeSuccessor);
				int index = indexOf(closedList, nodeSuccessor);
				closedList.erase(closedList.begin() + index);

			} else {
				openList.push_back(nodeSuccessor);
			}
			nodeSuccessor->Hn = successor_current_heuristic;
			nodeSuccessor->parent = nodeCurrent;
		}

		closedList.push_back(nodeCurrent);

	}

	Node* p = nodeGoal;
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

	Node* nodeStart = points.at(0);
	Node* nodeGoal = points.at(points.size() - 1);

	vector<Node*> openList;
	vector<Node*> closedList;

	int w = 0;

	openList.push_back(nodeStart);

	nodeStart->Gn = 0;

	while (!openList.empty()) {
		Node* nodeCurrent;
		int minIndex = 0;
		double minFn = numeric_limits<double>::max();

		for (int i = 0; i < openList.size(); i++) {
			Node* p = openList.at(i);
			if (((w * p->Gn) + p->Hn) < minFn) {
				minFn = (w * p->Gn) + p->Hn;
				minIndex = i;
			}
		}

		nodeCurrent = openList.at(minIndex);
		openList.erase(openList.begin() + minIndex);

		if (nodeCurrent == nodeGoal) {
			nodeGoal->parent = nodeCurrent->parent;
			break;
		}

		vector<Node*> successors = nodeCurrent->getSuccessors(points);

		for (int i = 0; i < successors.size(); i++) {
			Node* nodeSuccessor = successors.at(i);

			double successorCurrentDist = EuclidianDist(nodeCurrent,
					nodeSuccessor);
			double successorCurrentCost = nodeCurrent->Gn
					+ successorCurrentDist;

			int openFound = indexOf(openList, nodeSuccessor);
			int closedFound = indexOf(closedList, nodeSuccessor);

			if (openFound > -1) {
				if (nodeSuccessor->Gn <= successorCurrentCost)
					continue;
			} else if (closedFound > -1) {
				if (nodeSuccessor->Gn <= successorCurrentCost)
					continue;

				openList.push_back(nodeSuccessor);
				int index = indexOf(closedList, nodeSuccessor);
				closedList.erase(closedList.begin() + index);

			} else {
				openList.push_back(nodeSuccessor);

			}
			nodeSuccessor->Gn = successorCurrentCost;
			nodeSuccessor->parent = nodeCurrent;
		}
		closedList.push_back(nodeCurrent);

	}

	Node* p = nodeGoal;
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

void DynamicPlanning(Item *items) {

	vector<Node*> waypoints;

	points.clear();
	Initialization();

	Graph *graph = new Graph();
	graph->initGraph();
	graph->ConnectAllNodes();
	graph->AssignHeuristics();

	Item *obstaclesList = items;

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
	double startTime = CountTck();

	bool check = false;

	while (true) {
		double currentTime = CountTck() - startTime;
		robot.Read();

		if( currentTime >= 3000 )
		{
			for(int i = 0; i < 4; i++)
			{
				double x = funcRand(-6.00, 6.00);
				double y = funcRand(-6.00, 6.00);
				//robot.Read();
				if(sqrt(pow((x-positionProxy.GetXPos()),2) + pow((y-positionProxy.GetYPos()), 2)) > 1)
				{
					simulationProxy.SetPose2d(obstaclesList[i].name, x, y, 0);
				}
				else
					printf("imposing");
			}
			LoadObstacles(obstaclesList);

			startTime = CountTck();
		}

		if (i == 0) {
			p = waypoints.at(j);
			positionProxy.GoTo(p->x, p->y, 0);
			i++;
		}

		if ((isGoal(p->x, p->y, positionProxy.GetXPos(), positionProxy.GetYPos()))) {
			printf("Checkpoint--> (x: %lf y: %lf) completed... \n", p->x, p->y);

			if (isGoal(nodeGoal->x, nodeGoal->y, positionProxy.GetXPos(),
					positionProxy.GetYPos())) {
				printf("\nGoal Reached\n");
				break;
			} else {
				i = 0;
				j++;
			}
		}

		double avoidDist = 1.5;

		if ((snrProxy[3] < avoidDist) || (snrProxy[4] < avoidDist)) {
			printf("\nObstacle detected\n");

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

			if (isGoal(nodeGoal->x, nodeGoal->y, positionProxy.GetXPos(),
					positionProxy.GetYPos())) {
				break;
			}

			if (overflow > NO_OF_POINTS) {
				DynamicPlanning(obstaclesList);
			}

			check = true;

			continue;
		}

		if (isGoal(nodeGoal->x, nodeGoal->y, positionProxy.GetXPos(),
				positionProxy.GetYPos())) {
			break;
		}

		double safeDist = 1.5;
		if ((snrProxy[0] > safeDist) && (snrProxy[1] > safeDist)
				&& (snrProxy[2] > safeDist) && (snrProxy[3] > safeDist)
				&& (snrProxy[4] > safeDist) && (snrProxy[5] > safeDist)
				&& (snrProxy[6] > safeDist) && (snrProxy[7] > safeDist)
				&& (snrProxy[8] > safeDist) && (snrProxy[9] > safeDist)
				&& (snrProxy[10] > safeDist) && (snrProxy[11] > safeDist)
				&& (snrProxy[12] > safeDist) && (snrProxy[13] > safeDist)
				&& (snrProxy[14] > safeDist) && (snrProxy[15] > safeDist)) {

			if (isGoal(nodeGoal->x, nodeGoal->y, positionProxy.GetXPos(),
					positionProxy.GetYPos())) {
				break;
			}

			if (check == true)
				check = false, DynamicPlanning(obstaclesList);

		}

	}
}

int main(int argc, char *argv[]) {
	iter = 1;
	double startTime = CountTck();
	clock_t tStart = clock();

	srand(time(NULL));
	simulationProxy.SetPose2d("iRobo", 6, -6, 120);

	Item obstaclesList[8];
	LoadObstacles(obstaclesList);

	positionProxy.SetMotorEnable(1);
	positionProxy.RequestGeom();
	snrProxy.RequestGeom();

	printf("\n");

	prevX = positionProxy.GetXPos();
	prevY = positionProxy.GetYPos();

	DynamicPlanning(obstaclesList);

	double currentTime = CountTck() - startTime;

	cout << "Time Taken: " << 2.5 * (currentTime / 1000) << endl;
	cout << "Distance Traveled = " << traveledDist << endl;
	return 0;
}

