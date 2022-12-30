#ifndef _GRAPH_ALGORITHMS_H_
#define _GRAPH_ALGORITHMS_H_
#include "Graph.h"

class GraphAlgorithms {
public:
    static void BFS(Graph g);
    static bool isBipartite(Graph g);
    static void DFS(Graph g);
    static std::vector<double> singleSourceShortestPath(Graph g, int s);
    static std::vector<std::pair<int, int>> prims(Graph g);
    static std::vector<std::pair<int, int>> kruskals(Graph g);
    static void TSP(Graph g, int s);
    static void maxFlow(Graph g, int s);
};

#endif
