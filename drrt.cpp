
#include "drrt.h"

AdRRT::AdRRT(Instance *ins, int _timeLimit, int _seed, int _delta)
    : TreeIndex(flann::KDTreeIndexParams()), ins(ins), D(DistTable(ins)), timeLimit(_timeLimit), seed(_seed), delta(_delta)
{
    params.checks = 64;
    params.cores = 4;
}

void AdRRT::addPointToIndex(flann::Index<flann::L1<float>> &index, std::vector<int> &newNode) const
{
    auto *p = new float[2 * newNode.size()];
    int id = 0;
    for (auto &i : newNode)
    {
        p[id++] = static_cast<float>(ins->G.vPts[i]->x);
        p[id++] = static_cast<float>(ins->G.vPts[i]->y);
    }
    flann::Matrix<float> n(p, 1, 2 * newNode.size());
    index.addPoints(n);
}

void AdRRT::initialization()
{
    sample.resize(ins->N);
    A.resize(ins->N);
    std::iota(A.begin(), A.end(), 0);
    occupied_now.resize(ins->G.size, -1);
    occupied_next.resize(ins->G.size, -1);

    Tree.emplace_back(new adRRTNode(ins->starts, 0, 0, 0));

    mapPointToId = std::make_unique<std::unordered_map<std::vector<int>, uint, VectorHash>>();
    mapPointToId->emplace(ins->starts, 0);

    auto *indexStart = new float[2 * ins->N];
    int index = 0;
    for (auto &i : ins->starts)
    {
        indexStart[index++] = static_cast<float>(ins->G.vPts[i]->x);
        indexStart[index++] = static_cast<float>(ins->G.vPts[i]->y);
    }
    flann::Matrix<float> s(indexStart, 1, 2 * ins->N);
    TreeIndex = flann::Index<flann::L1<float>>(s, flann::KDTreeIndexParams());
    TreeIndex.buildIndex();

    targetField.resize(ins->N);
    priorities.resize(ins->N);
    C_next.reserve(5);
    visited.resize(ins->G.size, false);
    MT = getRandomGenerator(seed);
    nearestNodeIndex.reserve(10);
}

void AdRRT::updateRandomDomain(int range)
{

    for (int i = 0; i < ins->N; i++)
    {
        targetField[i].clear();
        std::unordered_set<int> closedNodes;
        std::vector<int> path = D.getTargetPath(i, Tree.back()->point[i]);
        for (auto &step : path)
        {
            targetField[i].push_back(step);
            closedNodes.emplace(step);
        }
        uint begin = 0;
        uint end = targetField[i].size();
        for (uint dist = 1; dist <= range; dist++)
        {
            for (uint j = begin; j < end; j++)
            {
                int node = targetField[i][j];
                for (auto neighbour : ins->G.adjList[node])
                {
                    if (!closedNodes.count(neighbour))
                    {
                        targetField[i].push_back(neighbour);
                        closedNodes.emplace(neighbour);
                    }
                }
            }
            begin = end;
            end = targetField[i].size();
        }
    }
}

void AdRRT::randomSample()
{
    for (int i : A)
    {
        if (!highPrioritiesGroup.count(i))
        {
            sample[i] = ins->goals[i];
            continue;
        }
        bool accepted = false;
        while (!accepted)
        {
            uint selectedIndex = getRandomInt(MT, 0, targetField[i].size() - 1);
            uint dist = D.getTarget(i, targetField[i][selectedIndex]);

            if (dist < delta)
            {
                accepted = true;
            }
            else
            {
                double rejectProbability = exp(-(dist * dist) / 2 * (delta * delta)) / delta * std::pow(2 * M_PI, 0.5);
                if (getRandomDouble(MT, 0, 1) < rejectProbability)
                {
                    accepted = true;
                }
            }
            if (accepted)
            {
                sample[i] = targetField[i][selectedIndex];
            }
        }
    }
}

void AdRRT::addTree(uint currentNodeIndex, std::vector<std::vector<int>> &routes)
{
    for (auto &route : routes)
    {

        auto &currentNode = Tree[currentNodeIndex];

        uint newIndex = Tree.size();
        auto *newNode = new adRRTNode(route, currentNodeIndex, currentNode->parentsNum + 1, 0);

        Tree.push_back(newNode);
        addPointToIndex(TreeIndex, route);

        mapPointToId->emplace(route, newIndex);
        currentNodeIndex = newIndex;
    }
}

std::vector<int> AdRRT::getNearestNode(std::vector<int> &samples, flann::Index<flann::L1<float>> &index, uint num) const
{
    std::vector<float> q;
    for (int &p : samples)
    {
        q.push_back(static_cast<float>(ins->G.vPts[p]->x));
        q.push_back(static_cast<float>(ins->G.vPts[p]->y));
    }
    flann::Matrix<float> query(&q[0], 1, q.size());
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;

    index.knnSearch(query, indices, dists, num, params);

    return indices[0];
}

std::vector<std::vector<int>> AdRRT::getNewConfigs(std::vector<int> &locations)
{
    std::vector<int> step(sample.size(), -1);
    Edges.emplace_back(locations);
    for (int q : highPrioritiesGroup)
    {
        priorities[q] += 100.0f;
    }
    std::shuffle(A.begin(), A.end(), getRandomGenerator());
    std::sort(A.begin(), A.end(),
              [&](int i, int j)
              { return priorities[i] > priorities[j]; });

    int iter = getRandomInt(MT, 1, ins->L);

    for (int k = 0; k <= iter; ++k)
    {
        auto &node = Edges.back();
        if (node == sample)
        {
            break;
        }

        for (int i = 0; i < sample.size(); ++i)
        {
            int v = node[i];
            occupied_now[v] = i;
        }

        for (auto j : A)
        {
            if (step[j] == -1)
            {
                PIBT(step, node, sample, j);
            }
        }
        std::fill(occupied_next.begin(), occupied_next.end(), -1);
        std::fill(occupied_now.begin(), occupied_now.end(), -1);

        if (closed.count(step))
        {
            break;
        }
        Edges.emplace_back(step);

        closed.insert(step);
        step.assign(sample.size(), -1);
    }
    return Edges;
}

std::vector<std::vector<int>> AdRRT::getConnectedRoutes(const std::vector<int> &locations, int maxStep)
{
    std::vector<std::vector<int>> finalPlan;
    std::vector<int> step(ins->N, -1);
    std::vector<std::vector<int>> temLocation = {locations};
    for (int q = 0; q < ins->N; ++q)
    {
        priorities[q] = (float)D.getTarget(q, locations[q]) / (float)ins->N;
    }

    for (int stepCount = 0; stepCount < maxStep; ++stepCount)
    {
        for (int i = 0; i < ins->N; ++i)
        {
            int v = temLocation.back()[i];
            occupied_now[v] = i;
        }

        for (int k = 0; k < ins->N; ++k)
        {
            if (D.getTarget(k, temLocation.back()[k]) != 0)
            {
                priorities[k] += 1;
            }
            else
            {
                priorities[k] -= floor(priorities[k]);
            }
        }
        std::sort(A.begin(), A.end(),
                  [&](int i, int j)
                  { return priorities[i] > priorities[j]; });

        for (int j : A)
        {
            if (step[j] == -1)
            {
                PIBT(step, temLocation.back(), ins->goals, j);
            }
        }
        temLocation.push_back(step);

        std::fill(occupied_next.begin(), occupied_next.end(), -1);
        std::fill(occupied_now.begin(), occupied_now.end(), -1);

        step.assign(sample.size(), -1);

        if (temLocation.back() == ins->goals)
        {
            finalPlan = temLocation;
            return finalPlan;
        }
    }

    return temLocation;
}

bool AdRRT::connectToTarget(const std::vector<int> &start, uint lastAdded, int maxStep)
{
    std::vector<std::vector<int>> routes = getConnectedRoutes(start, maxStep);
    if (routes.back() != ins->goals)
    {
        return false;
    }
    uint previous;
    for (int i = 1; i < routes.size(); ++i)
    {

        if (i == 1)
        {
            previous = lastAdded;
        }
        else
        {
            previous = Tree.size() - 1;
        }
        addPointToIndex(TreeIndex, routes[i]);
        Tree.emplace_back(new adRRTNode(routes[i], previous, 0, 0));
        mapPointToId->emplace(routes[i], Tree.size() - 1);
    }
    return true;
}

bool AdRRT::PIBT(std::vector<int> &q_new, std::vector<int> &q_near, std::vector<int> target, int robot_idx)
{

    C_next.clear();
    for (int v : ins->G.adjList[q_near[robot_idx]])
    {
        C_next.push_back(v);
    }
    C_next.push_back(q_near[robot_idx]);
    std::shuffle(C_next.begin(), C_next.end(), MT);

    std::sort(C_next.begin(), C_next.end(), [&](int a, int b)
              {
        if (target[robot_idx] == ins->goals[robot_idx]){
            return D.getTarget(robot_idx, a) < D.getTarget(robot_idx, b);
        }else{
            return D.get(robot_idx, a) < D.get(robot_idx, b);
        } });

    for (int v : C_next)
    {
        if (occupied_next[v] != -1)
        {
            continue;
        }
        int j = occupied_now[v];
        if (j != -1 and q_new[j] == q_near[robot_idx])
        {
            continue;
        }
        q_new[robot_idx] = v;
        occupied_next[v] = robot_idx;

        if (j != -1 and (q_new[j] == -1) and (not PIBT(q_new, q_near, target, j)))
        {
            continue;
        }
        return true;
    }

    q_new[robot_idx] = q_near[robot_idx];
    occupied_next[q_near[robot_idx]] = robot_idx;
    return false;
}

void AdRRT::getBackpropPlan(std::vector<std::vector<int>> &finalPlan, uint start)
{
    std::vector<std::vector<int>> plan;
    uint current = start;
    std::vector<int> lastPoint;
    while (current != 0)
    {
        std::vector<int> &currentPoint = Tree[current]->point;

        if (currentPoint != lastPoint)
        {
            plan.push_back(currentPoint);
            lastPoint = currentPoint;
        }
        current = Tree[current]->prevId;
    }
    std::reverse(plan.begin(), plan.end());

    finalPlan.resize(plan[0].size(), std::vector<int>(plan.size()));

    for (uint i = 0; i < plan.size(); i++)
    {
        for (uint j = 0; j < plan[0].size(); j++)
        {
            finalPlan[j][i] = plan[i][j];
        }
    }
}

void AdRRT::findNotReachAgents(std::vector<int> &currentNode)
{
    for (int i = 0; i < ins->N; ++i)
    {
        int v = currentNode[i];
        occupied_now[v] = i;
    }

    for (int i : A)
    {
        if (currentNode[i] != ins->goals[i])
        {
            if (!visited[i])
            {
                highPrioritiesGroup.insert(i);
                std::queue<int> open;
                open.push(i);
                visited[i] = true;

                while (!open.empty())
                {
                    int n = open.front();
                    open.pop();
                    int position = currentNode[n];
                    for (int m : ins->G.adjList[position])
                    {
                        if (occupied_now[m] != -1)
                        {
                            if (!visited[occupied_now[m]])
                            {
                                highPrioritiesGroup.insert(occupied_now[m]);
                                open.push(occupied_now[m]);
                                visited[occupied_now[m]] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    std::fill(visited.begin(), visited.end(), false);
    std::fill(occupied_now.begin(), occupied_now.end(), -1);
}

bool AdRRT::isConnected(std::vector<int> &start, std::vector<int> &end) const
{

    bool allNeighbors = true;
    std::set<std::pair<int, int>> detectSwitch;
    for (size_t k = 0; k < start.size(); ++k)
    {
        if (!(start[k] == end[k] || std::count(ins->G.adjList[end[k]].begin(), ins->G.adjList[end[k]].end(), start[k])))
        {
            allNeighbors = false;
            break;
        }
    }

    if (!allNeighbors)
    {
        return false;
    }

    for (size_t i = 0; i < start.size(); ++i)
    {
        std::pair<int, int> switchPair(std::min(start[i], end[i]), std::max(start[i], end[i]));
        if (detectSwitch.count(switchPair) > 0)
        {
            return false;
        }
        else
        {
            detectSwitch.emplace(switchPair);
        }
    }

    return true;
}

void AdRRT::rewire(std::vector<std::vector<int>> &edge, int &q_nearId)
{
    if (TreeIndex.size() < 10)
    {
        nearestNodeIndex = getNearestNode(edge.front(), TreeIndex, TreeIndex.size());
    }
    else
    {
        nearestNodeIndex = getNearestNode(edge.front(), TreeIndex, 10);
    }

    nearestNodeIndex.erase(std::remove_if(nearestNodeIndex.begin(), nearestNodeIndex.end(), [this, q_nearId](int a)
                                          { return a == q_nearId || Tree[a]->parentsNum > Tree[q_nearId]->parentsNum; }),
                           nearestNodeIndex.end());

    if (nearestNodeIndex.empty())
    {
        return;
    }

    std::sort(nearestNodeIndex.begin(), nearestNodeIndex.end(), [this](int a, int b)
              { return Tree[a]->parentsNum < Tree[b]->parentsNum; });
    double h = INFINITY;
    for (int i : nearestNodeIndex)
    {
        if (isConnected(Tree[i]->point, edge.front()) && i != 0)
        {
            double currentH = 0;
            for (int j = 0; j < ins->N; ++j)
            {
                currentH += D.getTarget(j, Tree[i]->point[j]);
            }
            if (currentH < h)
            {
                h = currentH;
                q_nearId = i;
            }
        }
    }
}
void AdRRT::expand(int iteration)
{

    if (iteration % 50 == 0)
    {
        delta += 1;
        updateRandomDomain(delta);
    }
    double random = getRandomDouble(MT, 0.0, 1.0);
    if (random < 0.3)
    {
        sample = ins->goals;
    }
    else
    {
        randomSample();
    }
    D.updateTargets(sample);

    int nearId = -1;
    double costs = INFINITY;

    if (Tree.size() < 10)
    {
        nearestNodeIndex = getNearestNode(sample, TreeIndex, TreeIndex.size());
    }
    else
        nearestNodeIndex = getNearestNode(sample, TreeIndex, 10);

    for (int i : nearestNodeIndex)
    {
        double d = 0;
        for (int j = 0; j < sample.size(); ++j)
        {
            d += D.get(j, Tree[i]->point[j]);
        }
        if (d < costs)
        {
            costs = d;
            nearId = i;
        }
    }

    auto &q_near = Tree[nearId]->point;

    getNewConfigs(q_near);
    rewire(Edges, nearId);
    highPrioritiesGroup.clear();
    findNotReachAgents(Edges.back());

    addTree(static_cast<size_t>(nearId), Edges);
    Edges.clear();
}

std::vector<std::vector<int>> AdRRT::run()
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    initialization();

    int iteration = 0;
    // main loop
    bool foundPath = false;
    uint targetAt = 0;

    auto startWhile = std::chrono::high_resolution_clock::now();

    while (!foundPath)
    {
        std::cout << iteration << std::endl;
        auto nowWhile = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(nowWhile - startWhile).count();
        expand(iteration);

        bool isExecuteConnect = true;
        uint distToGoal = 0;
        for (int i = 0; i < ins->N; ++i)
        {
            distToGoal = std::max(distToGoal, D.getTarget(i, Tree[Tree.size() - 1]->point[i]));
        }

        if (distToGoal > ins->L + 1)
        {
            for (int q = 0; q < ins->N; ++q)
            {
                priorities[q] = (float)D.getTarget(q, Tree[Tree.size() - 1]->point[q]) / (float)ins->N;
            }
            std::sort(A.begin(), A.end(),
                      [&](int i, int j)
                      { return priorities[i] > priorities[j]; });
            isExecuteConnect = false;
        }
        if (isExecuteConnect)
        {
            if (connectToTarget(Tree[Tree.size() - 1]->point, Tree.size() - 1, ins->L))
            {
                targetAt = Tree.size() - 1;
                iteration++;
                std::cout << "Connect!" << std::endl;
                break;
            }
        }

        if (duration > timeLimit * 1000)
        {
            std::cout << "Time limit exceeded." << std::endl;
            return {};
        }

        if (foundPath)
        {
            break;
        }
        iteration++;
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto planningTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    std::vector<std::vector<int>> finalPlan;

    getBackpropPlan(finalPlan, targetAt);

    metrics = Metrics(ins, finalPlan, planningTime);

    return finalPlan;
}
