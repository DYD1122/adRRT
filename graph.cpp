
#include "graph.h"

template <class T>
Point<T>::Point(T _x, T _y)
    : x(_x), y(_y)
{
}

Graph::Graph(const std::string &filename) : height(0), width(0), size(0)
{
    std::ifstream in(filename);
    if (!in)
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        throw std::runtime_error("Failed to open map file");
    }

    std::string key;
    while (in >> key)
    {
        if (key == "height")
        {
            in >> height;
        }
        else if (key == "width")
        {
            in >> width;
        }
        else if (key == "map")
        {
            break;
        }
    }

    map.resize(height, std::vector<char>(width));

    adjList.resize(height * width);
    vPts.resize(height * width, nullptr);

    std::string line;
    std::getline(in, line);

    for (int i = 0; i < height; i++)
    {
        std::getline(in, line);
        for (int j = 0; j < width; j++)
        {
            map[i][j] = line[j];
        }
    }

    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            int currentIndex = getIndex(i, j, width);
            auto p = new Point<int>(i, j);
            vPts[currentIndex] = p;
            if (map[i][j] == '.')
            {
                addEdgeIfValid(i - 1, j, currentIndex);
                addEdgeIfValid(i, j + 1, currentIndex);
                addEdgeIfValid(i + 1, j, currentIndex);
                addEdgeIfValid(i, j - 1, currentIndex);
            }
        }
    }
    size = vPts.size();
}

int Graph::getIndex(int x, int y, int width)
{
    return x * width + y;
}

bool Graph::isValid(int x, int y, int height, int width)
{
    return x >= 0 && x < height && y >= 0 && y < width;
}

void Graph::addEdgeIfValid(int x, int y, int currentIndex)
{
    if (isValid(x, y, height, width) && map[x][y] == '.')
    {
        int neighborIndex = getIndex(x, y, width);
        adjList[currentIndex].push_back(neighborIndex);
    }
}

Graph::~Graph()
{
    for (auto &n : vPts)
    {
        delete n;
    }
}