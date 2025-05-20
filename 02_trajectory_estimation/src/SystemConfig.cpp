#include "../include/SystemConfig.hpp"
#include <Eigen/Core>

void SystemConfig::loadFromFile(const std::string& filename) {
    std::ifstream file(filename + "SystemConfig.json");
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    try {
        dxyz_gps1 = Eigen::Vector3d( json_data["GPS1"]["leverarm"]["x"],
                                     json_data["GPS1"]["leverarm"]["y"],
                                     json_data["GPS1"]["leverarm"]["z"] );

        dxyz_gps2 = Eigen::Vector3d( json_data["GPS2"]["leverarm"]["x"],
                                     json_data["GPS2"]["leverarm"]["y"],
                                     json_data["GPS2"]["leverarm"]["z"] );                     

        dxyz_prism1 = Eigen::Vector3d( json_data["Prism1"]["leverarm"]["x"],
                                       json_data["Prism1"]["leverarm"]["y"],
                                       json_data["Prism1"]["leverarm"]["z"] ); 

        dxyz_prism2 = Eigen::Vector3d( json_data["Prism2"]["leverarm"]["x"],
                                       json_data["Prism2"]["leverarm"]["y"],
                                       json_data["Prism2"]["leverarm"]["z"] ); 

    } catch (nlohmann::json::exception& e) {
        throw std::runtime_error("Error parsing JSON: " + std::string(e.what()));
    }
}

Eigen::Vector3d SystemConfig::getLeverarmGps1() const { return dxyz_gps1; }
Eigen::Vector3d SystemConfig::getLeverarmGps2() const { return dxyz_gps2; }
Eigen::Vector3d SystemConfig::getLeverarmPrism1() const { return dxyz_prism1; }
Eigen::Vector3d SystemConfig::getLeverarmPrism2() const { return dxyz_prism2; }

void SystemConfig::print() const {
    std::cout << "\n ___________________________________________________________ " << std::endl;
    std::cout << "| ----------------- System settings ----------------------- |" << std::endl;
    std::cout << "| GPS1 leverarm:         [" << std::fixed << std::setprecision(4) << dxyz_gps1.x() << ", " << dxyz_gps1.y()<< ", " << dxyz_gps1.z() << "]  [m]   " << std::endl;
    std::cout << "| GPS2 leverarm:         [" << std::fixed << std::setprecision(4) << dxyz_gps2.x()<< ", " << dxyz_gps2.y()<< ", " << dxyz_gps2.z() << "]     [m]   " << std::endl;
    std::cout << "| Prsim1 leverarm:       [" << std::fixed << std::setprecision(4) << dxyz_prism1.x()<< ", " << dxyz_prism1.y()<< ", " << dxyz_prism1.z() << "]   [m] " << std::endl;
    std::cout << "| Prsim2 leverarm:       [" << std::fixed << std::setprecision(4) << dxyz_prism2.x()<< ", " << dxyz_prism2.y()<< ", " << dxyz_prism2.z() << "]     [m]   " << std::endl;
    std::cout << "| __________________________________________________________|" << std::endl;
}