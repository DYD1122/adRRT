
#include "utils.h"

std::mt19937 getRandomGenerator(std::optional<int> seed) {
    if (seed.has_value()) {
        return std::mt19937(seed.value());
    } else {
        return std::mt19937(1);
    }
}

int getRandomInt(std::mt19937& MT, int from, int to)
{
    std::uniform_int_distribution<int> r(from, to);
    return r(MT);
}

float getRandomFloat(std::mt19937& MT, float from, float to)
{
    std::uniform_real_distribution<float> r(from, to);
    return r(MT);
}

double getRandomDouble(std::mt19937& MT, double from, double to)
{
    std::uniform_real_distribution<double> r(from, to);
    return r(MT);
}
