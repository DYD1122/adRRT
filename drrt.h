
#ifndef ADRRT_DRRT_H
#define ADRRT_DRRT_H
#include "dist_table.h"
#include "utils.h"
#include "metrics.h"
#include <unordered_map>
#include <flann/flann.hpp>
#include <chrono>

struct VectorHash
{
    size_t operator()(const std::vector<int> &v) const
    {
        size_t seed = v.size();
        for (auto &i : v)
        {
            seed ^= std::hash<int>{}(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

struct adRRTNode
{
    double cost;
    std::vector<int> point;
    uint prevId;
    int parentsNum;

    adRRTNode(std::vector<int> point, uint prev, int itSinceCon, double cost)
        : point(std::move(point)), prevId(prev), parentsNum(itSinceCon), cost(cost) {}
};

struct AdRRT
{
    const Instance *ins;
    DistTable D;
    int seed;
    int timeLimit;
    int delta;

    std::mt19937 MT;

    std::vector<int> sample;
    flann::Index<flann::L1<float>> TreeIndex;
    flann::SearchParams params;
    std::vector<int> A;
    std::vector<int> occupied_now;
    std::vector<int> occupied_next;
    std::vector<int> C_next;
    std::vector<bool> visited;

    std::unordered_set<int> highPrioritiesGroup;
    std::vector<float> priorities;
    std::unordered_set<std::vector<int>, VectorHash> closed;

    std::vector<adRRTNode *> Tree;
    std::vector<std::vector<int>> Edges;
    std::vector<int> nearestNodeIndex;
    std::unique_ptr<std::unordered_map<std::vector<int>, uint, VectorHash>> mapPointToId;

    std::vector<std::vector<int>> targetField;

    Metrics metrics;
    AdRRT(Instance *ins, int _timeLimit, int _seed, int _delta);

    // Initialization parameters
    void initialization();
    // Update sampling domain
    void updateRandomDomain(int range);
    // Accept reject sampling
    void randomSample();
    // flann index configuration
    void addPointToIndex(flann::Index<flann::L1<float>> &index, std::vector<int> &newNode) const;
    // Use BFS to check for proxies that have not reached their target point
    void findNotReachAgents(std::vector<int> &currentNode);
    // Get the point in the tree closest to the sampling point
    std::vector<int> getNearestNode(std::vector<int> &sample, flann::Index<flann::L1<float>> &index, uint num) const;
    // Move using PIBT
    std::vector<std::vector<int>> getNewConfigs(std::vector<int> &locations);
    // Priority inheritance with backtracking
    bool PIBT(std::vector<int> &qnew, std::vector<int> &qnear, std::vector<int> target, int robot_idx);
    // Using a dynamic priority version of PIBT as a connector
    bool connectToTarget(const std::vector<int> &start, uint lastAdded, int maxStep);
    // Build paths using connectors
    std::vector<std::vector<int>> getConnectedRoutes(const std::vector<int> &locations, int maxStep);
    // Retrace the path to build all agents
    void getBackpropPlan(std::vector<std::vector<int>> &plan, uint start);
    // Add node to tree
    void addTree(uint currentNodeIndex, std::vector<std::vector<int>> &routes);
    // Optimizing paths for rewiring
    void rewire(std::vector<std::vector<int>> &edge, int &qnearId);
    // Determine whether two adjacent time steps can be directly connected
    bool isConnected(std::vector<int> &start, std::vector<int> &end) const;
    // Expansion operation
    void expand(int iteration);

    std::vector<std::vector<int>> run();

    ~AdRRT()
    {
        for (auto &node : Tree)
        {
            delete node;
        }
        uint veclen = TreeIndex.size();
        for (uint i = 0; i < veclen; ++i)
        {
            delete[] TreeIndex.getPoint(i);
        }
    }
};

#endif // ADRRT_DRRT_H
