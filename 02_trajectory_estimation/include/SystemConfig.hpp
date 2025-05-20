// include/Config.h

#ifndef CONFIG_SystemConfig
#define CONFIG_SystemConfig

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <Eigen/Core>

class SystemConfig {
public:

    Eigen::Vector3d dxyz_gps1;
    Eigen::Vector3d dxyz_gps2;
    Eigen::Vector3d dxyz_prism1;
    Eigen::Vector3d dxyz_prism2;

private:    

public:
    SystemConfig() = default;

    void loadFromFile(const std::string& filename);

    Eigen::Vector3d getLeverarmGps1() const;
    Eigen::Vector3d getLeverarmGps2() const;
    Eigen::Vector3d getLeverarmPrism1() const;
    Eigen::Vector3d getLeverarmPrism2() const;

    void print() const;
};

#endif // CONFIG_H
