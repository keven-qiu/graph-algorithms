#ifndef _UNION_FIND_H_
#define _UNION_FIND_H_
#include <vector>

class UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;
public:
    UnionFind(int n);
    bool inSet(int x);
    bool addSet(int x);
    int find(int x);
    void unite(int x, int y);
};

#endif
