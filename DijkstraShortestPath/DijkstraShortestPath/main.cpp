// Napon Krassner (Jett)
// CSC 3430
// HW 4
// Description: Dijkstra's shortest path algorithm. The program is for adjacency matrix
// representation of the graph.

#include <stdio.h>
#include <limits.h>

// Number of vertices in the graph
#define V 9

// Function declaration
void dijkstra(int graph[V][V], int start);
int minDistance(int dist[], bool sptSet[]);
void printSolution(int dist[], int n, int parent[], int start);
void printPath(int parent[], int j);


// Main program with given graph to run dijkstra shortest path algorithm
int main()
{
	// Graph copied from provided GraphMatrix.txt
	int graph[V][V] = { { 0,  4,  0,  0,  0,  0,  0,  8,  0 },
	{ 4,  0,  8,  0,  0,  0,  0, 11,  0 },
	{ 0,  8,  0,  7,  0,  4,  0,  0,  2 },
	{ 0,  0,  7,  0,  9, 14,  0,  0,  0 },
	{ 0,  0,  0,  9,  0, 10,  0,  0,  0 },
	{ 0,  0,  4, 14, 10,  0,  2,  0,  0 },
	{ 0,  0,  0,  0,  0,  2,  0,  1,  6 },
	{ 8, 11,  0,  0,  0,  0,  1,  0,  7 },
	{ 0,  0,  2,  0,  0,  0,  6,  7,  0 } };

	dijkstra(graph, 0);

	return 0;
}


// Parameters:	2-D array of int - graph matrix
//				int - starting point
// Return:	nothing
// Description: Funtion for Dijkstra's shortest path algorithm for a matrix graph.
void dijkstra(int graph[V][V], int start)
{
	int dist[V];		// The output array, shortest distance from start to i
	bool sptSet[V];		// true if vertex i in shortest path tree or shortest distance from start to i is finalized
	int parent[V];		// Parent array to store shortest path tree

	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < V; i++)
	{
		parent[start] = -1;	// initialize starting point
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}

	// Distance of starting point from starting point is always 0
	dist[start] = 0;

	// Find shortest path for all vertices
	for (int count = 0; count < V - 1; count++)
	{
		// Pick the minimum distance vertex from the set of
		// vertices not yet processed. u is always equal to starting point
		// in first iteration.
		int u = minDistance(dist, sptSet);

		// Mark the picked vertex as processed
		sptSet[u] = true;

		// Update dist value of the adjacent vertices of the
		// picked vertex.
		for (int v = 0; v < V; v++)

			// Update dist[v] only if is not in sptSet, there is
			// an edge from u to v, and total weight of path from
			// starting point to v through u is smaller than current value of
			// dist[v]
			if (!sptSet[v] && graph[u][v] &&
				dist[u] + graph[u][v] < dist[v])
			{
				parent[v] = u;
				dist[v] = dist[u] + graph[u][v];
			}
	}

	// print the constructed distance array
	printSolution(dist, V, parent, start);
}


// Parameters:	int array - output array for shortest distance
//				bool array - shortest path tree
// Return: int - index of minimum distance vertex
// Description: function to find the vertex with minimum distance
// value, from the set of vertices not yet included in shortest
// path tree
int minDistance(int dist[], bool sptSet[])
{
	// Initialize min value
	int min = INT_MAX;
	int min_index;		// used to return the index

	for (int v = 0; v < V; v++)
		if (sptSet[v] == false && dist[v] <= min)
			min = dist[v], min_index = v;

	return min_index;
}


// Parameters:	int array - output array of distance for all vertices
//				int - number of vertices
//				int array - array of shortest path tree
// Return:	nothing
// Description: Print the constructed distance array
void printSolution(int dist[], int n, int parent[], int start)
{
	printf("Vertex\t  Distance from Source: %d", start);
	for (int i = 0; i < V; i++)
	{
		printf("\n%d \t\t %d\t\tPath: %d", i, dist[i], start);
		printPath(parent, i);
	}
	printf("\n");
}


// Parameters:	int array - array of shortest path tree
//				int - end point to print
// Return:	nothing
// Description: Function to print shortest path from source to j using parent array
void printPath(int parent[], int j)
{
	// Base Case : If j is source
	if (parent[j] == -1)
		return;

	printPath(parent, parent[j]);

	printf("-->%d", j);
}