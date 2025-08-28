
#include "dist_table.h"

DistTable::DistTable(const Instance* ins)
        : ins(ins), V(ins->G.size), D(ins->N, std::vector<int>(V, V)), TD(ins->N, std::vector<int>(V, V)), prev(V, std::vector<int>(V, -1)), sample(ins->goals)
{
    setup();
}

void DistTable::setup()
{
    for (size_t i = 0; i < ins->N; ++i) {
        open.emplace_back();
        openForTD.emplace_back();
        auto n = ins->goals[i];
        open[i].push(n);
        openForTD[i].push(n);
        D[i][ins->goals[i]] = 0;
        TD[i][ins->goals[i]] = 0;
    }
}

uint DistTable::get(int i, int v)
{
    if (D[i][v] < V) return D[i][v];

    while (!open[i].empty()) {
        auto n = open[i].front();
        open[i].pop();
        const int d_n = D[i][n];
        for (auto m : ins->G.adjList[n]) {
            const int d_m = D[i][m];
            if (d_n + 1 >= d_m) continue;
            D[i][m] = d_n + 1;
            open[i].push(m);
        }
        if (n == v) return d_n;
    }
    return V;
}

uint DistTable::getTarget(int i, int v)
{
    if (TD[i][v] < V) return TD[i][v];

    if (openForTD[i].empty()){
        int t = ins->goals[i];
        openForTD[i].push(t);
        TD[i][t] = 0;
        prev[i][t] = -1;
    }

    while (!openForTD[i].empty()) {
        auto n = openForTD[i].front();
        openForTD[i].pop();
        const int d_n = TD[i][n];
        for (auto m : ins->G.adjList[n]) {
            const int d_m = TD[i][m];
            if (d_n + 1 >= d_m) continue;
            TD[i][m] = d_n + 1;
            prev[i][m] = n;
            openForTD[i].push(m);
        }
        if (n == v) return d_n;
    }
    return V;
}

std::vector<int> DistTable::getTargetPath(int i, int v) {
    std::vector<int> path;

    if (TD[i][v] == V)
    {
        getTarget(i, v);
    }

    int current = v;
    while (current != -1)
    {
        path.push_back(current);
        current = prev[i][current];
    }
    // std::reverse(path.begin(), path.end());
    return path;
}

void DistTable::updateTargets(std::vector<int> &newTargets) {
    for (int i = 0; i < ins->N; ++i) {
        if (sample[i] != newTargets[i]){
            sample[i] = newTargets[i];
            std::fill(D[i].begin(), D[i].end(), ins->G.size);
            open[i] = std::queue<int>();
            open[i].push(newTargets[i]);
            D[i][newTargets[i]] = 0;
        }
    }
}