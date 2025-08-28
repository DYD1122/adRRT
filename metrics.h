
#include "dist_table.h"
#include <unordered_set>
#include <set>

struct Metrics
{
    uint makeSpan = 0;
    uint sumCosts = 0;
    long long time = 0;

    Metrics() = default;
    Metrics(const Instance *ins, std::vector<std::vector<int>> &plan, long long _time)
        : time(_time)
    {
        if (!plan.empty())
        {
            makeSpan = plan[0].size();
        }
        sumCosts = 0;
        for (uint i = 0; i < plan.size(); i++)
        {
            if (!plan[i].empty())
            {
                for (uint j = plan[i].size() - 1; true; j--)
                {
                    if (plan[i][j] != ins->goals[i])
                    {
                        sumCosts += j + 1;
                        break;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
    }
};

std::ostream &operator<<(std::ostream &stream, const Metrics &stats);

bool checkTwpPlan(Instance *ins, std::vector<std::vector<int>> plan, std::string &outputFile);