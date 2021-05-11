#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include "limits.h"

#define NUMDRIVERS 2
using namespace std;

/*	Each request has a start time, the starting location, the end location.
	Each request also tracks the amount of time the passenger was waiting. */
typedef struct {
	int startTime;
	int startLoc;
	int endLoc;
	int waitTime;
}request;

/*	Each uber driver keeps track of its next destination and the time at which it can respond to a new request. 
	The time at which the the driver can respond to a request is either the time at which it will complete the current request, if the driver is busy
	Or it is the current time, if the driver is idle*/
typedef struct {
	int destination;
	int timeAtResponse;
}Uber;

/*	A node in the branch and bound decision tree
	Each decision node tracks its level in the decision tree
	an upperbound for solutions that pass through this node
	and a lowerbound for solutions that pass through this node
	The decision node also has an array of two uber objects to keep track of the state of each uber
	resulting from the chain of decisions that lead to this node */
typedef struct {
	int level = -1;
	int waitTimeUpperBound = -1;
	int waitTimeLowerBound = -1;
	vector<int>decisionsSoFar;
	Uber drivers[NUMDRIVERS] = {};
}DecisionNode;

// Initialisation functions
vector<vector<int>> readGraph(const char* filename);
queue<request> getRequests(const char* filename);

// Dikstra's Algorithm functions
int minDistance(const vector<int>& dist, const set<int>& cSet);
vector<int> dijkstra(const vector<vector<int>>& graph, int start);
vector<vector<int>> getMinimumTravelTimes(const vector<vector<int>>& G);

// Branch and Bound functions
/* Calculates and returns the total wait time resulting from assinging to each pending request, the driver that can arrive
at the start location of the request first*/
int upperBoundGreedy(const vector<request>& reqs, const DecisionNode& N, const vector<vector<int>>& MinTimes);

/* Calculates and returns the sequence of decisions that optimises total wait time for the pending reqeusts*/
vector<int> BranchAndBound(const vector<request>& requests, const Uber& driver0, const Uber& driver1, const vector<vector<int>>& minTimes);


int main() {
	vector<vector<int>> cityNetwork;
	vector<vector<int>> minimumTimes;
	queue<request>futureRequests;
	vector<request>pendingRequests;
	queue<request>completedRequests;
	vector<int>optimalDecisions;

	// Variables for simulation passage of time in main loop
	int currentTime;	// The current time in each iteration of the loop
	//int nextRequestStartTime;	// Tracks when the next request will be 'received' by Uber
	bool newRequest;	// indicates whether a new request was received

	// Read network and request files
	string networkFile = "network.csv";
	string requestsFile = "supplementpickups.csv";

	cityNetwork = readGraph(networkFile.c_str());
	futureRequests = getRequests(requestsFile.c_str());
	minimumTimes = getMinimumTravelTimes(cityNetwork);
	
	Uber driver0, driver1;

	// Both drivers will start off at node 1 of the map
	driver0.destination = driver1.destination = 1;

	// Initialise the current time to 0
	driver0.timeAtResponse = driver1.timeAtResponse = currentTime = 0;

	// Get the time at which the next request will be 'received'
	newRequest = false;
	
	// While loop simulates passage of time. The loop continues until both the set of pending requests, and the set of future requests are empty.
	while (!(futureRequests.empty() && pendingRequests.empty())) {

		// Update timeatresponse for idle drivers
		if(driver0.timeAtResponse < currentTime) driver0.timeAtResponse = currentTime;
		if (driver1.timeAtResponse < currentTime) driver1.timeAtResponse = currentTime;

		// 'receive' all the requests 'sent' at this time
		while (!futureRequests.empty() && currentTime == futureRequests.front().startTime) {
			pendingRequests.push_back(futureRequests.front());
			futureRequests.pop();
			newRequest = true;
		}

		if (driver0.timeAtResponse <= currentTime){	// If Driver 0 is not busy (has completed last assigned request)
			// update timeatresponse to current time of non busy drivers, to reflect that they were idle.
			if (newRequest) {	// If there are new pending requests
				// Recalculate assignment of drivers to requests to optimise for most recent batch of pending requests
				optimalDecisions = BranchAndBound(pendingRequests, driver0, driver1, minimumTimes);
				newRequest = false;	
			}

			// Find first request assigned to driver 0, if any
			for (int i = 0; i < pendingRequests.size(); i++) {
				if (optimalDecisions[i] == 0) {	// If a pending request assigned to this driver
					int requestStartLoc, requestEndLoc;

					requestStartLoc = pendingRequests[i].startLoc;
					requestEndLoc = pendingRequests[i].endLoc;

					/*	Update wait time for that request
						Time at which driver arrives to request start location is the sum of the time at which the driver can respond to the request
						and the time taken to travel to the start location of the current request */
					pendingRequests[i].waitTime = driver0.timeAtResponse + minimumTimes[driver0.destination-1][requestStartLoc-1] - pendingRequests[i].startTime;
					
					// Update the time at which the driver can respond to a new request
					// This is the sum of the time at which it can respond to current request, the time to get to current request and the time to complete current request
					driver0.timeAtResponse = driver0.timeAtResponse + minimumTimes[driver0.destination-1][requestStartLoc-1] + minimumTimes[requestStartLoc-1][requestEndLoc-1];
					// Update destination location
					driver0.destination = requestEndLoc;
					
					// Mark request as complete
					completedRequests.push(pendingRequests[i]);
					pendingRequests.erase(pendingRequests.begin() + i);	// Erase from pending requests
				}

			}
		}

		// Exactly the same as for driver 0
		if (driver1.timeAtResponse <= currentTime) {			
			// Recalculate request assignment if there are new pending requests
			if (newRequest) {
				optimalDecisions = BranchAndBound(pendingRequests, driver0, driver1, minimumTimes);
				newRequest = false;
			}
			
			// Find first request assigned to driver 0, if any
			for (int i = 0; i < pendingRequests.size(); i++) {
				if (optimalDecisions[i] == 1) {
					int requestStartLoc, requestEndLoc;
					requestStartLoc = pendingRequests[i].startLoc;
					requestEndLoc = pendingRequests[i].endLoc;

					// Update wait time for request
					pendingRequests[i].waitTime = driver1.timeAtResponse + minimumTimes[driver1.destination-1][requestStartLoc-1] - pendingRequests[i].startTime;
					
					//Update info for driver
					driver1.timeAtResponse = driver1.timeAtResponse + minimumTimes[driver1.destination-1][requestStartLoc-1] + minimumTimes[requestStartLoc-1][requestEndLoc-1];
					driver1.destination = requestEndLoc;

					// Mark request as complete
					completedRequests.push(pendingRequests[i]);
					pendingRequests.erase(pendingRequests.begin() + i);
				}

			}
		}
		currentTime++;	// increment time

	}
	// At the end of the while loop all requests have been served

	// Calculate the sum of the wait times for all the requests
	int totalWaitTIme = 0;
	while (!completedRequests.empty()) {
		totalWaitTIme += completedRequests.front().waitTime;
		completedRequests.pop();
	}
	cout << "Total wait time: " << totalWaitTIme << endl;

	return 0;
}



/* Function definitions */
vector<vector<int>> readGraph(const char* filename) {
	string line, entry;
	vector<vector<int>> cityNetwork;
	// Open files
	ifstream networkFile(filename);
	if (networkFile.fail()) {
		cout << "Could not read file." << endl;
		exit(1);
	}

	/* Load city network information*/
	while (getline(networkFile, line)) {
		stringstream ss(line);
		vector<int> edges;
		while (getline(ss, entry, ',')) {
			edges.push_back(stoi(entry));
		}
		cityNetwork.push_back(edges);
	}

	return cityNetwork;
}

queue<request> getRequests(const char *filename) {
	string line, entry;
	queue<request> requests;
	// Open file
	ifstream requestsFile(filename);
	if (requestsFile.fail()) {
		cout << "Could not read file." << endl;
		exit(1);
	}

	// Load requests
	while (getline(requestsFile, line)) {
		request r;
		stringstream ss(line);

		// Get start time
		getline(ss, entry, ',');
		r.startTime = stoi(entry);

		// Get start location
		getline(ss, entry, ',');
		r.startLoc = stoi(entry);

		// Get end location
		getline(ss, entry, ',');
		r.endLoc = stoi(entry);

		r.waitTime = -1;
		// Add to queue
		requests.push(r);
	}

	return requests;
}


vector<int> dijkstra(const vector<vector<int>>& graph, int start)
{
	set<int>reached;
	set<int>candidate;

	vector<int>minTime;
	//Initialisation 
	minTime.resize(graph.size(), INT_MAX);		// Make array big enough for all nodes, initialise value to INT_MAX

	minTime.at(start) = 0;	// The minimum time to the starting node from the starting node is 0
	candidate.insert(start);	

	while (candidate.size() > 0) {
		// Find shortest path from the set of vertices not yet processed
		int u = minDistance(minTime, candidate);

		reached.insert(u);
		candidate.erase(u);

		// Iterate through neighbours of u
		for (int v = 0; v < graph.size(); v++) {
			if (!graph[u][v]) 
				continue;	// Do nothing if vertex is not a neighbour
			else if (reached.count(v) > 0)
				continue; // Do nothing if vertex already reached

			if (minTime[u] + graph[u][v] < minTime[v]) {	// If there is a faster path to v from the start
				if (minTime[v] == INT_MAX)
					candidate.insert(v);
				minTime[v] = minTime[u] + graph[u][v];	//update minimum time to v from start
			}
		}
	}

	return minTime;
}

vector<vector<int>> getMinimumTravelTimes(const vector<vector<int>>& G) {
	vector<vector<int>>minTimes;
	vector<int>prev;

	// Calculate minimum distance from every node to every other node
	for (int i = 0; i < G.size(); i++) {
		vector<int> minTimeRow;
		minTimeRow = dijkstra(G, i);
		minTimes.push_back(minTimeRow);
	}

	return minTimes;
}


int minDistance(const vector<int>& dist, const set<int>& cSet) {
	int smallestDistanceNode = -1;
	int smallestDistance = INT_MAX;
	for (int i = 0; i < dist.size(); i++) {
		if (cSet.count(i) <= 0)
			continue; // Vertice not in candidate set, do nothing
		if (dist.at(i) < smallestDistance) {
			smallestDistance = dist.at(i);
			smallestDistanceNode = i;
		}
	}

	return smallestDistanceNode;
}

vector<int> BranchAndBound(const vector<request>& requests, const Uber& driver0, const Uber& driver1, const vector<vector<int>>& minTimes) {
	// Queue of ndoes in decision tree
	queue<DecisionNode> Q;
	vector<int> optimalDecisions;

	// the dummy node does not represent a decision, but simply initialises the decision tree with the state of each driver 
	DecisionNode dummyNode;
	dummyNode.level = -1;
	dummyNode.decisionsSoFar = {};
	dummyNode.waitTimeLowerBound = 0;
	dummyNode.waitTimeUpperBound = 0;
	dummyNode.drivers[0] = driver0;
	dummyNode.drivers[1] = driver1;

	// Find an upper bound for the whole decision tree. 
	int overallUpperBound = upperBoundGreedy(requests, dummyNode, minTimes);

	Q.push(dummyNode);

	while (!Q.empty()) {	// While there are live solutions

		// Choose a partial solution, u
		DecisionNode u = Q.front();
		Q.pop();

		// If there is nothing on next level i.e u is the last node of a full solution, do nothing
		if (u.level == requests.size() - 1)
			continue;

	// Can branch in two directions from u
	// Either assign driver 0 to take the next pending request
		DecisionNode v;
		v.level = u.level + 1;
		
		// Update the decision made at this node
		v.decisionsSoFar = u.decisionsSoFar;	// The decisions made in the path leading up to v
		v.decisionsSoFar.push_back(0);	// Decision at this level 

		// Find lower bound for wait on this node
		// Add total wait time leading up to v to lowerbound
		v.waitTimeLowerBound = u.waitTimeLowerBound + u.drivers[0].timeAtResponse + minTimes[u.drivers[0].destination-1][requests[v.level].startLoc-1] - requests[v.level].startTime;

		// update information for drivers if this branch in decision tree is taken
		// Information for driver0 changes, since it was assigned a new request
		v.drivers[0].timeAtResponse = u.drivers[0].timeAtResponse + minTimes[u.drivers[0].destination - 1][requests[v.level].startLoc - 1] + minTimes[requests[v.level].startLoc - 1][requests[v.level].endLoc - 1];
		v.drivers[0].destination = requests[v.level].endLoc;
		
		// Other driver has not taken on a new request, and so retains info from parent node
		v.drivers[1] = u.drivers[1];

		// Find upperbound extended through this node
		v.waitTimeUpperBound = upperBoundGreedy(requests, v, minTimes);

		// If the lowerBound is greater than the overall upperbound, there are no feasible solutions through this node
		if (v.waitTimeLowerBound > overallUpperBound) {
			// Do nothing. Pruning the subtree rooted at this node
		}
		// Otherwise, there is a possible solution through this node, push node to Q
		else {
			Q.push(v);

			// If v is the last node of a full solution and has better or equal upperbound compared to overall upperbound
			if ((v.level == requests.size() - 1) && (v.waitTimeUpperBound <= overallUpperBound)) {
				// Update overall upperbound and overall decisions
				overallUpperBound = v.waitTimeUpperBound;
				optimalDecisions = v.decisionsSoFar;
			}
		}

	// Or can assign driver 1 to take this request.
		DecisionNode w;

		// Update decision made at this node
		w.decisionsSoFar = u.decisionsSoFar;	// Decisions leading up to w
		w.decisionsSoFar.push_back(1);	// Decision at this level
		w.level = u.level + 1;

		// Find lower bound for wait on this node
		w.waitTimeLowerBound = u.waitTimeLowerBound + u.drivers[1].timeAtResponse + minTimes[u.drivers[1].destination-1][requests[w.level].startLoc-1] - requests[w.level].startTime;

		// update information for drivers if this branch in decision tree is taken
		// Information for driver1 changes, since it was assigned a new request
		w.drivers[1].timeAtResponse = u.drivers[1].timeAtResponse + minTimes[u.drivers[1].destination - 1][requests[w.level].startLoc - 1] + minTimes[requests[w.level].startLoc - 1][requests[w.level].endLoc - 1];
		w.drivers[1].destination = requests[w.level].endLoc;
		
		// Info for other driver is the same as it was in parent node
		w.drivers[0] = u.drivers[0];

		// Find upperbound extended through this node
		w.waitTimeUpperBound = upperBoundGreedy(requests, w, minTimes);

		// If the lowerBound is greater than the overall upperbound, there are no feasible solutions through this node
		if (w.waitTimeLowerBound > overallUpperBound) {
			// Do nothing. Pruning the subtree rooted at this node
		}
		// Otherwise, there is a possible solution through this node, push to Q
		else {
			Q.push(w);
			// If w is the last node of a full solution and has better or equal upperbound compared to overall upperbound
			if ((w.level == requests.size() - 1) && (w.waitTimeUpperBound <= overallUpperBound)) {
				// Update overall upperbound and overall decisions
				overallUpperBound = w.waitTimeUpperBound;
				optimalDecisions = w.decisionsSoFar;
			}
		}
	}

	return optimalDecisions;
}

/* The greedy algorithm assigns to each successive request, the driver that can get there first
	Upperbound is the lowerbound at the root of the subtree, plus the cummulative sum of the wait times from
	assignign drivers to requests using the greedy method
*/
int upperBoundGreedy(const vector<request>& reqs, const DecisionNode& N, const vector<vector<int>>& MinTimes)
{
	Uber tempDriver0 = N.drivers[0];
	Uber tempDriver1 = N.drivers[1];
	int driver0ArrivalTime, driver1ArrivalTime;
	
	// The upper bound through this node is at least the cost so far at this node
	int upperBound = N.waitTimeLowerBound;

	// For each remaining request in the decision tree
	for (int i = N.level + 1; i < reqs.size(); i++) {
		// Calculate for each driver, the time it can arrive at the response start location
		driver0ArrivalTime = tempDriver0.timeAtResponse + MinTimes[tempDriver0.destination-1][reqs[i].startLoc-1];
		driver1ArrivalTime = tempDriver1.timeAtResponse + MinTimes[tempDriver1.destination-1][reqs[i].startLoc-1];

		// If driver 0 can arrive at the request start time first
		if (driver0ArrivalTime < driver1ArrivalTime) {
			// Add wait time resulting from assigning driver 0 to the request
			upperBound += driver0ArrivalTime - reqs[i].startTime;

			// Update driver information
			tempDriver0.timeAtResponse = tempDriver0.timeAtResponse + MinTimes[tempDriver0.destination-1][reqs[i].startLoc-1] + MinTimes[reqs[i].startLoc-1][reqs[i].endLoc-1];
			tempDriver0.destination = reqs[i].endLoc;
		}
		else {
			// Add wait time resulting from assigning driver 0 to the request
			upperBound += driver1ArrivalTime - reqs[i].startTime;

			// Update driver information
			tempDriver1.timeAtResponse = tempDriver1.timeAtResponse + MinTimes[tempDriver1.destination-1][reqs[i].startLoc-1] + MinTimes[reqs[i].startLoc-1][reqs[i].endLoc-1];
			tempDriver1.destination = reqs[i].endLoc;
		}
	}
	return upperBound;
}