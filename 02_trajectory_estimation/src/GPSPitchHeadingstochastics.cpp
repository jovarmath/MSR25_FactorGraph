#include "../include/GPSPitchHeadingstochastics.hpp"

void GPSPitchHeadingstochastics::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    try {
        sigpitch = json_data["sigpitch"];
        sigheading = json_data["sigheading"];

    } catch (nlohmann::json::exception& e) {
        throw std::runtime_error("Error parsing JSON: " + std::string(e.what()));
    }
}

double GPSPitchHeadingstochastics::getSigPitch() const { return sigpitch; }
double GPSPitchHeadingstochastics::getSigHeading() const { return sigheading; }

void GPSPitchHeadingstochastics::print() const {
    std::cout << "\n _____________________________________________________ " << std::endl;
    std::cout << "| ------ GPS pitch heading stochastic settings ------- |" << std::endl;
    std::cout << "| sigma position x:         " << std::fixed << std::setprecision(6) << sigpitch << "  [rad]            |" << std::endl;
    std::cout << "| sigma position y:         " << std::fixed << std::setprecision(6) << sigheading << "  [rad]            |" << std::endl;
    std::cout << "| _____________________________________________________|\n" << std::endl;
}