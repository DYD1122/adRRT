#include "drrt.h"
#include "cxxopts.hpp"

int main(int argc, char* argv[]) {
    try {
        cxxopts::Options options("adRRT_Algorithm", "An improved dRRT solver for MAPF");

        options.add_options()
                ("m,map", "Map file path", cxxopts::value<std::string>()->default_value("map/random-32-32-20.map"))
                ("s,scen", "Scenario file path", cxxopts::value<std::string>()->default_value("instance/random-32-32-20-even-1.scen"))
                ("o,output", "Output file path", cxxopts::value<std::string>()->default_value("output.txt"))
                ("n,number", "Number of agents", cxxopts::value<int>()->default_value("100"))
                ("t,time", "Time limit in seconds", cxxopts::value<int>()->default_value("60"))
                ("r,seed", "Random number seed", cxxopts::value<int>()->default_value("4"))
                ("d,delta", "threshold", cxxopts::value<int>()->default_value("2"))
                ("h,help", "Print usage");

        auto result = options.parse(argc, argv);

        if (result.count("help")) {
            std::cout << options.help() << std::endl;
            return 0;
        }

        std::string map_name = result["map"].as<std::string>();
        std::string scen_name = result["scen"].as<std::string>();
        std::string output_name = result["output"].as<std::string>();
        int N = result["number"].as<int>();
        int timeLimit_sec = result["time"].as<int>();
        int seed = result["seed"].as<int>();
        int delta = result["delta"].as<int>();
        try {
            Instance ins(scen_name, map_name, N);
            AdRRT drrt(&ins, timeLimit_sec, seed, delta);
            std::vector<std::vector<int>> plan = drrt.run();
            std::cout << drrt.metrics;
            std::cout << checkTwpPlan(&ins, plan, output_name);
        }catch (const std::exception& e){
            std::cerr << "Exception caught: " << e.what() << std::endl;
            return 1;
        }

    } catch (const cxxopts::exceptions::exception& e) {
        std::cerr << "Error parsing options: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}

