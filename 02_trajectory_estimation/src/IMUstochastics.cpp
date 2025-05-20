// src/Config.cpp

#include "../include/IMUstochastics.hpp"



void IMUstochastics::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    try {
        sigacc.x = json_data["sigacc"]["x"];
        sigacc.y = json_data["sigacc"]["y"];
        sigacc.z = json_data["sigacc"]["z"];

        siggyro.x = json_data["siggyro"]["x"];
        siggyro.y = json_data["siggyro"]["y"];
        siggyro.z = json_data["siggyro"]["z"];

        sigaccbias.x = json_data["sigaccbias"]["x"];
        sigaccbias.y = json_data["sigaccbias"]["y"];
        sigaccbias.z = json_data["sigaccbias"]["z"];

        siggyrobias.x = json_data["siggyrobias"]["x"];
        siggyrobias.y = json_data["siggyrobias"]["y"];
        siggyrobias.z = json_data["siggyrobias"]["z"];
    } catch (nlohmann::json::exception& e) {
        throw std::runtime_error("Error parsing JSON: " + std::string(e.what()));
    }
}

IMUstochastics::Sig IMUstochastics::getSigAcc() const { return sigacc; }
IMUstochastics::Sig IMUstochastics::getSigGyro() const { return siggyro; }
IMUstochastics::Sig IMUstochastics::getSigAccBias() const { return sigaccbias; }
IMUstochastics::Sig IMUstochastics::getSigGyroBias() const { return siggyrobias; }

void IMUstochastics::print() const {
    std::cout << "\n _____________________________________________________ " << std::endl;
    std::cout << "| ------------ IMU stochastic settings --------------- |" << std::endl;
    std::cout << "| sigma acceleration x:         " << std::fixed << std::setprecision(6) << sigacc.x << "  [m/s^2]      |" << std::endl;
    std::cout << "| sigma acceleration y:         " << std::fixed << std::setprecision(6) << sigacc.y << "  [m/s^2]      |" << std::endl;
    std::cout << "| sigma acceleration z:         " << std::fixed << std::setprecision(6) << sigacc.z << "  [m/s^2]      |" << std::endl;
    std::cout << "|                                                      |" << std::endl;
    std::cout << "| sigma gyroscope x:            " << std::fixed << std::setprecision(6) << siggyro.x << "  [rad/s]      |" << std::endl;
    std::cout << "| sigma gyroscope y:            " << std::fixed << std::setprecision(6) << siggyro.y << "  [rad/s]      |" << std::endl;
    std::cout << "| sigma gyroscope z:            " << std::fixed << std::setprecision(6) << siggyro.z << "  [rad/s]      |" << std::endl;
    std::cout << "|                                                      |" << std::endl;
    std::cout << "| sigma acceleration bias x:    " << std::fixed << std::setprecision(6) << sigaccbias.x << "  [m/s^2]      |" << std::endl;
    std::cout << "| sigma acceleration bias y:    " << std::fixed << std::setprecision(6) << sigaccbias.y << "  [m/s^2]      |" << std::endl;
    std::cout << "| sigma acceleration bias z:    " << std::fixed << std::setprecision(6) << sigaccbias.z << "  [m/s^2]      |" << std::endl;
    std::cout << "|                                                      |" << std::endl;
    std::cout << "| sigma gyroscope bias x:       " << std::fixed << std::setprecision(6) << siggyrobias.x << "  [rad/s]      |" << std::endl;
    std::cout << "| sigma gyroscope bias y:       " << std::fixed << std::setprecision(6) << siggyrobias.y << "  [rad/s]      |" << std::endl;
    std::cout << "| sigma gyroscope bias z:       " << std::fixed << std::setprecision(6) << siggyrobias.z << "  [rad/s]      |" << std::endl;
    std::cout << "| _____________________________________________________|" << std::endl;
}