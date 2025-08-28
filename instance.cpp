
#include "instance.h"

Instance::Instance(const std::string& scen_filename, const std::string& map_filename, int N)
        :G(Graph(map_filename)), starts(std::vector<int>()), goals(std::vector<int>()), N(N), L(std::max(G.height, G.width))
{
    std::ifstream file(scen_filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << scen_filename << std::endl;
        throw std::runtime_error("Failed to open scenario file");
    }
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::vector<int> tokens;

        while (std::getline(ss, token, '\t')) {
            if (!token.empty() && std::all_of(token.begin(), token.end(), ::isdigit)) {
                tokens.push_back(std::stoi(token));
            }
        }

        if (tokens.size() >= 6) {
            int mapWidth = tokens[2];
            int startX = tokens[3];
            int startY = tokens[4];
            int endX = tokens[5];
            int endY = tokens[6];

            int startIndex = startY * mapWidth + startX;
            int endIndex = endY * mapWidth + endX;
            starts.push_back(startIndex);
            goals.push_back(endIndex);
        }
        if (starts.size() == N){
            break;
        }
    }
    if (starts.size() < N) {
        throw std::runtime_error("Scenario file does not contain enough start-goal pairs");
    }
}
