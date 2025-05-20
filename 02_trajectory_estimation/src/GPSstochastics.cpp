#include "../include/GPSstochastics.hpp"

void GPSstochastics::loadFromFile(const std::string& filename) {
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

GPSstochastics::Sig GPSstochastics::getSigxyz() const { return sigpos; }

void GPSstochastics::print() const {
    std::cout << "\n _____________________________________________________ " << std::endl;
    std::cout << "| ------------ GPS stochastic settings --------------- |" << std::endl;
    std::cout << "| sigma position x:         " << std::fixed << std::setprecision(6) << sigpos.x << "  [m]              |" << std::endl;
    std::cout << "| sigma position y:         " << std::fixed << std::setprecision(6) << sigpos.y << "  [m]              |" << std::endl;
    std::cout << "| sigma position z:         " << std::fixed << std::setprecision(6) << sigpos.z << "  [m]              |" << std::endl;
    std::cout << "| _____________________________________________________|" << std::endl;
}