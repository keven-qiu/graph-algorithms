#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <vector>
#include <list>
#include <utility>

class Graph {
    int n; // |V|
    // (vertex, weight)
    std::vector<std::list<std::pair<int, double>>> adjList;
    bool directed;
    bool weighted;
public:
    Graph(int n, bool directed, bool weighted);

    int getSize();

    bool isDirected();
    bool isWeighted();

    void resizeGraph(int n);

    void addEdge(int v, int w, double weight);
    std::vector<std::tuple<int, int, double>> getEdgeList();

    std::list<std::pair<int, double>> getNeighbours(int v);

    bool isNeighbour(int u, int v);
};

#endif
