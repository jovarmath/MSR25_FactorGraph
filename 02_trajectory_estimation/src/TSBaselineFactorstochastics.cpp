#include "../include/TSBaselinestochastics.hpp"
#include <Eigen/Core>


void TSbaselinestochastics::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    try {
        sigbase.x = json_data["sigbaseline"]["x"];
        sigbase.y = json_data["sigbaseline"]["y"];
        sigbase.z = json_data["sigbaseline"]["z"];

    } catch (nlohmann::json::exception& e) {
        throw std::runtime_error("Error parsing JSON: " + std::string(e.what()));
    }
}

TSbaselinestochastics::Sig TSbaselinestochastics::getSigxyz() const { return sigbase; }

void TSbaselinestochastics::print() const {
    std::cout << "\n _______________________________________________________ " << std::endl;
    std::cout << "| ------------ TS Baseline stochastic settings ---------------- |" << std::endl;
    std::cout << "| sigma baseline x:         " << sigbase.x << "  [m]              |" << std::endl;
    std::cout << "| sigma baseline y:         " << sigbase.y << "  [m]              |" << std::endl;
    std::cout << "| sigma baseline z:         " << sigbase.z << "  [m]              |" << std::endl;
    std::cout << "| _____________________________________________________|" << std::endl;
}