#include "UnionFind.h"
#include <algorithm>

UnionFind::UnionFind(int n) {
    parent.resize(n, -1);
    rank.resize(n, -1);
}

bool UnionFind::inSet(int x) {
    return std::find(parent.begin(), parent.end(), x) != parent.end();
}

int UnionFind::find(int x) {
    if (parent[x] == -1) return x;
    parent[x] = find(parent[x]);
    return parent[x];
}

void UnionFind::unite(int x, int y) {
    int a = find(x);
    int b = find(y);

    if (a != b) {
        if (rank[a] < rank[b])
            parent[a] = b;
        else if (rank[a] > rank[b])
            parent[b] = a;
        else {
            parent[b] = a;
            ++rank[a];
        }
    }
}
