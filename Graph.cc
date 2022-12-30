#include "Graph.h"
#include <tuple>

Graph::Graph(int n, bool directed = false, bool weighted = false) {
    this->n = n;
    resizeGraph(n);
    this->directed = directed;
    this->weighted = weighted;
}

int Graph::getSize() {
    return this->n;
}

bool Graph::isDirected() {
    return this->directed;
}

bool Graph::isWeighted() {
    return this->weighted;
}

void Graph::resizeGraph(int n) {
    this->adjList.resize(n);
}

void Graph::addEdge(int v, int w, double weight = 1.0) {
    this->adjList[v].push_back(std::make_pair(w, weight));
    if (!this->isDirected()) {
        this->adjList[w].push_back(std::make_pair(v, weight));
    }
}

std::vector<std::tuple<int, int, double>> Graph::getEdgeList() {
    std::vector<std::tuple<int, int, double>> edgeList;
    for (int v = 0; v < getSize(); ++v) {
        for (auto &w: adjList[v]) {
            if (!isEdgeInList(v, w.first, edgeList)) {
                edgeList.push_back({v, w.first, w.second});
            }
        }
    }
    return edgeList;
}

bool isEdgeInList(int v, int w, std::vector<std::tuple<int, int, double>> l) {
    for (auto &edge: l) {
        if ((std::get<0>(edge) == v && std::get<1>(edge) == w) || (std::get<0>(edge) == w && std::get<1>(edge) == v))
            return true;
    }
    return false;
}

std::list<std::pair<int, double>> Graph::getNeighbours(int v) {
    return this->adjList[v];
}

bool Graph::isNeighbour(int u, int v) {
    std::list<std::pair<int, double>> adjacentVertices;
    for (auto &w: adjacentVertices) {
        if (w.first == v) return true;
    }
    return false;
}
