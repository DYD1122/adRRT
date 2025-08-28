
#ifndef ADRRT_INSTANCE_H
#define ADRRT_INSTANCE_H
#pragma once
#include "graph.h"
#include "algorithm"

struct Instance {
    const Graph G;
    std::vector<int> starts;
    std::vector<int> goals;
    const int N;
    const int L;

    Instance(const std::string& scen_filename, const std::string& map_filename,
             int N = 1);
    ~Instance() = default;
};


#endif //ADRRT_INSTANCE_H
