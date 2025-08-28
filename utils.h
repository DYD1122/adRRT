
#ifndef ADRRT_UTILS_H
#define ADRRT_UTILS_H
#include <random>
#include <optional>

std::mt19937 getRandomGenerator(std::optional<int> seed = std::nullopt);

int getRandomInt(std::mt19937& MT, int from, int to);

float getRandomFloat(std::mt19937& MT, float from, float to);

double getRandomDouble(std::mt19937& MT, double from, double to);

#endif //ADRRT_UTILS_H
