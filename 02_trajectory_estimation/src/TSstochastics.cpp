#include "../include/TSstochastics.hpp"
#include <Eigen/Core>


void TSstochastics::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    try {
        sigpos.x = json_data["sigpos"]["x"];
        sigpos.y = json_data["sigpos"]["y"];
        sigpos.z = json_data["sigpos"]["z"];

    } catch (nlohmann::json::exception& e) {
        throw std::runtime_error("Error parsing JSON: " + std::string(e.what()));
    }
}

TSstochastics::Sig TSstochastics::getSigxyz() const { return sigpos; }

void TSstochastics::print() const {
    std::cout << "\n _______________________________________________________ " << std::endl;
    std::cout << "| ------------ TS stochastic settings ---------------- |" << std::endl;
    std::cout << "| sigma position x:         " << sigpos.x << "  [m]              |" << std::endl;
    std::cout << "| sigma position y:         " << sigpos.y << "  [m]              |" << std::endl;
    std::cout << "| sigma position z:         " << sigpos.z << "  [m]              |" << std::endl;
    std::cout << "| _____________________________________________________|" << std::endl;
}