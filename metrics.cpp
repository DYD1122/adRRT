
#include "metrics.h"

std::ostream &operator<<(std::ostream &stream, const Metrics &stats)
{

    stream << "Time[ms]: " << stats.time << "\n";
    stream << "Makespan: " << stats.makeSpan << "\n";
    stream << "Sum Of Costs: " << stats.sumCosts << "\n";

    return stream;
}

bool checkTwpPlan(Instance *ins, std::vector<std::vector<int>> plan, std::string &outputFile)
{
    size_t max = 0;
    for (size_t i = 0; i < plan.size(); i++)
    {
        if (plan[i].size() > max)
        {
            max = plan[i].size();
        }
        if (!plan[i].empty())
        {
            if (plan[i][0] != ins->starts[i] || plan[i][plan[i].size() - 1] != ins->goals[i])
            {
                std::cout << "Some agents did not reach the target point\n";
                return false;
            }
        }
    }

    std::vector<std::vector<int>> realPlan(max, std::vector<int>(plan.size(), -1));
    for (size_t i = 0; i < plan.size(); i++)
    {
        for (size_t j = 0; j < plan[i].size(); j++)
        {
            realPlan[j][i] = plan[i][j];
        }
    }

    std::ofstream ofs(outputFile);
    size_t agentIndex = 0;
    for (auto &i : plan)
    {
        ofs << "Agent" << agentIndex << " -> ";
        for (size_t j = 0; j < i.size(); j++)
        {
            int idx = i[j];
            int row = idx / ins->G.width;
            int column = idx % ins->G.width;

            if (j > 0)
                ofs << ",";
            ofs << "(" << row << "," << column << ")";
        }
        ofs << std::endl;
        agentIndex++;
    }
    ofs.close();
    for (size_t i = 1; i < realPlan.size(); i++)
    {
        std::unordered_set<int> current;
        std::set<std::pair<int, int>> detectSwitch;
        int nullCount = 0;
        for (size_t j = 0; j < realPlan[i].size(); j++)
        {
            int curr = realPlan[i][j];
            int prev = realPlan[i - 1][j];
            if (curr != prev)
            {
                bool possible = false;
                for (int n : ins->G.adjList[prev])
                {
                    if (n == curr)
                    {
                        possible = true;
                        break;
                    }
                }
                if (!possible)
                {
                    return false;
                }
            }

            if (realPlan[i][j] != -1)
            {
                if (current.count(realPlan[i][j]) > 0)
                {
                    std::cout << "Vertex conflict\n";

                    return false;
                }
                else
                {
                    current.emplace(realPlan[i][j]);
                }
            }
            else
            {
                nullCount++;
            }
            if (realPlan[i][j] != -1 && realPlan[i - 1][j] != -1)
            {
                std::pair<int, int> switchPair(std::min(realPlan[i][j], realPlan[i - 1][j]), std::max(realPlan[i][j], realPlan[i - 1][j]));
                if (detectSwitch.count(switchPair) > 0)
                {
                    std::cout << "Swap conflict\n";
                    return false;
                }
                else
                {
                    detectSwitch.emplace(switchPair);
                }
            }
        }
    }
    return true;
}