
#ifndef ADRRT_DIST_TABLE_H
#define ADRRT_DIST_TABLE_H
#include "instance.h"
#include "queue"

struct DistTable {
    const uint V;
    const Instance *ins;

    std::vector<std::vector<int>> D;  // This distance table is used by the sampling points for each iteration
    std::vector<std::vector<int>> TD;  // Target dist_table
    std::vector<std::vector<int>> prev;

    std::vector<int> getTargetPath(int i, int v);
    std::vector<int> sample;

    std::vector<std::queue<int> > open;
    std::vector<std::queue<int> > openForTD;

    uint get(int i, int v);
    uint getTarget(int i, int v);

    DistTable(const Instance* ins);

    void setup();

    void updateTargets(std::vector<int> &newTargets);
};


#endif //ADRRT_DIST_TABLE_H
