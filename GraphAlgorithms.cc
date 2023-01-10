#include "GraphAlgorithms.h"
#include "UnionFind.h"
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <limits>
#include <functional>

void GraphAlgorithms::BFS(Graph& g) {
    std::vector<bool> visited(g.getSize(), false);

    for (int s = 0; s < g.getSize(); ++s) {
        if (!visited[s]) {
            std::queue<int> q;
            visited[s] = true;
            q.push(s);
            while (!q.empty()) {
                int u = q.front();
                q.pop();

                std::cout << u << " ";

                for (auto v: g.getNeighbours(u)) {
                    if (!visited[v.first]) {
                        visited[v.first] = true;
                        q.push(v.first);
                    }
                }
            }
        }
    }
    std::cout << std::endl;
}

bool GraphAlgorithms::isBipartite(Graph& g) {
    std::vector<int> colouring(g.getSize(), -1);

    for (int s = 0; s < g.getSize(); ++s) {
        if (colouring[s] == -1) {
            std::queue<int> q;
            colouring[s] = 0;
            q.push(s);

            while (!q.empty()) {
                int u = q.front();
                q.pop();

                if (g.isNeighbour(u, u)) return false;

                for (auto v: g.getNeighbours(u)) {
                    if (colouring[v.first] == -1) {
                        colouring[v.first] = 1 - colouring[u];
                        q.push(v.first);
                    } else if (colouring[v.first] == colouring[u]) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

std::vector<double> shortestPathBFS(Graph& g, int s) {
    std::vector<double> level(g.getSize(), 0);
    std::vector<bool> visited(g.getSize(), false);
    std::queue<int> q;

    visited[s] = true;
    level[s] = 0;
    q.push(s);
    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (auto v: g.getNeighbours(u)) {
            if (!visited[v.first]) {
                visited[v.first] = true;
                level[v.first] = level[u] + 1;
                q.push(v.first);
            }
        }
    }
    return level;
}

void GraphAlgorithms::DFS(Graph& g) {
    std::vector<bool> visited(g.getSize(), false);
    for (int s = 0; s < g.getSize(); ++s) {
        if (!visited[s]) {
            DFSHelper(g, s, visited);
        }
    }
}

void DFSHelper(Graph& g, int s, std::vector<bool>& visited) {
    visited[s] = true;
    std::cout << s << std::endl;

    for (auto &v: g.getNeighbours(s)) {
        if (!visited[v.first])
            DFSHelper(g, v.first, visited);
    }
}

std::vector<double> dijkstra(Graph& g, int s) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int,int>>> pq;

    std::vector<double> distance(g.getSize(), std::numeric_limits<double>::max());

    pq.push(std::make_pair(0, s));
    distance[s] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (auto &v: g.getNeighbours(u)) {
            if (distance[v.first] > distance[u] + v.second) {
                distance[v.first] = distance[u] + v.second;
                pq.push(std::make_pair(distance[v.first], v.first));
            }
        }
    }
    return distance;
}

std::vector<double> GraphAlgorithms::singleSourceShortestPath(Graph& g, int s) {
    if (g.isWeighted())
        return dijkstra(g, s);
    else
        return shortestPathBFS(g, s);
}

std::vector<std::pair<int, int>> GraphAlgorithms::prims(Graph& g) {
    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int,int>>> pq;

    int s = 0;

    std::vector<int> key(g.getSize(), std::numeric_limits<double>::max());
    std::vector<std::pair<int, int>> mstEdges(g.getSize(), {-1, -1}); // parent of vertex
    std::vector<bool> visited(g.getSize(), false);

    pq.push(std::make_pair(0, s));
    key[s] = 0;

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue;

        visited[u] = true;
        for (auto &v: g.getNeighbours(u)) {
            if (!visited[v.first] && key[v.first] > v.second) {
                key[v.first] = v.second;
                pq.push(std::make_pair(key[v.first], v.first));
                mstEdges.push_back({u, v.first});
            }
        }
    }
    return mstEdges;
}

bool sortTupleByThirdElement(const std::tuple<int, int, double>& a, const std::tuple<int, int, double>& b) {
    return std::get<2>(a) < std::get<2>(b);
}

std::vector<std::pair<int, int>> GraphAlgorithms::kruskals(Graph& g) {
    std::vector<std::pair<int, int>> mstEdges;
    std::vector<std::tuple<int, int, double>> edgeList = g.getEdgeList();
    std::sort(edgeList.begin(), edgeList.end(), sortTupleByThirdElement);
    
    double mstCost = 0;

    UnionFind uf(g.getSize());
    for (auto &edge: edgeList) {
        int u = std::get<0>(edge);
        int v = std::get<1>(edge);
        int weight = std::get<2>(edge);

        if (uf.find(u) != uf.find(v)) {
            uf.unite(u, v);
            mstCost += weight;
            mstEdges.push_back({u, v});
        }
    }
    return mstEdges;
}

void GraphAlgorithms::TSP(Graph& g, int s) {

}

double maxFlowBFS(Graph& g, int s, int t, std::vector<int>& parent, std::vector<std::vector<double>>& capacity) {
    for (int i = 0; i < parent.size(); ++i) {
        parent[i] = -1;
    }
    parent[s] = -2;
    std::queue<std::pair<int, double>> q;
    q.push({s, std::numeric_limits<double>::max()});

    while (!q.empty()) {
        std::pair<int, double> u = q.front();
        q.pop();

        for (auto &v: g.getNeighbours(u.first)) {
            if (parent[v.first] == -1 && capacity[u.first][v.first]) {
                parent[v.first] = u.first;
                double newFlow = std::min(v.second, capacity[u.first][v.first]);
                if (v.first == t)
                    return newFlow;
                q.push({v.first, newFlow});
            }
        }
    }
    return 0;
}

double GraphAlgorithms::maxFlow(Graph& g, int s, int t) {
    std::vector<std::vector<double>> capacity;
    double maximumFlow = 0;
    std::vector<int> parent(g.getSize());

    while (true) {
        double newFlow = maxFlowBFS(g, s, t, parent, capacity);
        if (newFlow == 0.0) break;
        maximumFlow += newFlow;
        int curr = t;
        while (curr != s) {
            int parentCurr = parent[curr];
            capacity[parentCurr][curr] -= newFlow;
            capacity[curr][parentCurr] += newFlow;
            curr = parentCurr;
        }
    }
    return maximumFlow;
}
