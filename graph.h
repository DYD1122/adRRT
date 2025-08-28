
#ifndef ADRRT_GRAPH_H
#define ADRRT_GRAPH_H
#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

using uint = unsigned int;

template <class T>
struct Point
{
    T x, y;
    Point(T _x, T _y);

    bool operator==(const Point &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct Graph
{
    std::vector<Point<int> *> vPts;
    std::vector<std::vector<int>> adjList;

    std::vector<std::vector<char>> map;
    int height, width;
    uint size;

    Graph(const std::string &filename);

    static int getIndex(int x, int y, int width);
    static bool isValid(int x, int y, int height, int width);
    void addEdgeIfValid(int x, int y, int currentIndex);

    ~Graph();
};

#endif // ADRRT_GRAPH_H
