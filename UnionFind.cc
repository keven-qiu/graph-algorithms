#include "UnionFind.h"

UnionFind::UnionFind(int n) {
    parent.resize(n, -1);
    rank.resize(n, -1);
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
