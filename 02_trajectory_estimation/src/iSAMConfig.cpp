#include "../include/iSAMConfig.hpp"
#include <Eigen/Core>


void iSAMConfig::loadFromFile(const std::string& filename) {
    std::ifstream file(filename + "iSAMConfig.json");
    if (!file.is_open()) {
        throw std::runtime_error("Error opening file: " + filename);
    }

    nlohmann::json json_data;
    file >> json_data;
    file.close();

    try {

        // Read boolean for Measurement Updates
        useGPS = json_data["MeasurementUpdates"]["useGPS"];
        useGPSheading = json_data["MeasurementUpdates"]["useGPSheading"];
        useTS1 = json_data["MeasurementUpdates"]["useTS"];
        if (json_data["MeasurementUpdates"].contains("useTS2")) {
            useTS2 = json_data["MeasurementUpdates"]["useTS2"];
        } else {
            useTS2 = false;
        }
        if (json_data["MeasurementUpdates"].contains("useTSbaseline")) {
            useTSbaseline = json_data["MeasurementUpdates"]["useTSbaseline"];
        } else {
            useTSbaseline = false;
        }       

        // Initial state stochastics
        InitialStateSigma_.sigxyz = Eigen::Vector3d(json_data["Initialsig"]["position"]["x"],
                                                    json_data["Initialsig"]["position"]["y"],
                                                    json_data["Initialsig"]["position"]["z"]);

        InitialStateSigma_.sigvxvyvz = Eigen::Vector3d(json_data["Initialsig"]["velocity"]["x"],
                                                       json_data["Initialsig"]["velocity"]["y"],
                                                       json_data["Initialsig"]["velocity"]["z"]);

        InitialStateSigma_.sigrpy = Eigen::Vector3d(json_data["Initialsig"]["orientation"]["x"],
                                                  json_data["Initialsig"]["orientation"]["y"],
                                                  json_data["Initialsig"]["orientation"]["z"]);

        // Read preintegration sigma
        PreintegrationSig = Eigen::Vector3d(json_data["PreintegrationSig"]["x"],
                                            json_data["PreintegrationSig"]["y"],
                                            json_data["PreintegrationSig"]["z"]);


        initialtime = json_data["InitialTime"];

        // isam parameter
        relinearizeSkip = json_data["iSAM"]["relinearizeSkip"];
        relinearizeThreshold = json_data["iSAM"]["relinearizeThreshold"];
        poserate = json_data["iSAM"]["poserate"];
        updaterate = json_data["iSAM"]["updaterate"];

    } catch (nlohmann::json::exception& e) {
        throw std::runtime_error("Error parsing JSON: " + std::string(e.what()));
    }
}

bool iSAMConfig::getuseGPS() const { return useGPS; }; 
bool iSAMConfig::getuseGPSheading() const { return useGPSheading; };
bool iSAMConfig::getuseTS1() const { return useTS1; }; 
bool iSAMConfig::getuseuseTS2() const { return useTS2; };
bool iSAMConfig::getuseuseTSbaseline() const { return useTSbaseline; };

double iSAMConfig::getinitialtime() const { return initialtime; };

//iSAMConfig::InitialstateSig iSAMConfig::getInitialStateSigma() const { return InitialStateSigma_; }; 

void iSAMConfig::print() const {
    std::cout << "\n _________________________________________________________ " << std::endl;
    std::cout << "| ----------------- ISAM settings ----------------------- |" << std::endl;
    std::cout << "| Measurements include:      " << "                             |" << std::endl;
    std::cout << "| Use GPS:          " << std::fixed << std::setprecision(6) << useGPS << "                                     |" << std::endl;
    std::cout << "| Use GPS heading   " << std::fixed << std::setprecision(6) << useGPSheading << "                                     |" << std::endl;
    std::cout << "| Use TS1:           " << std::fixed << std::setprecision(6) << useTS1 << "                                     |" << std::endl;
    std::cout << "| Use TS2:   " << std::fixed << std::setprecision(6) << useTS2 << "                                     |" << std::endl;
    std::cout << "| Use TSbaseline:   " << std::fixed << std::setprecision(6) << useTSbaseline << "                                     |" << std::endl;
    std::cout << "|                                                         |" << std::endl;
    std::cout << "| Initial pose stochastics:   " << "                            |" << std::endl;
    std::cout << "|   - Position:     [" << std::fixed << std::setprecision(4) << InitialStateSigma_.sigxyz.x() << ", " 
                                                                               << InitialStateSigma_.sigxyz.y() << ", "
                                                                               << InitialStateSigma_.sigxyz.z() << "]    [m]       |" << std::endl;
    std::cout << "|   - Velocity:     [" << std::fixed << std::setprecision(4) << InitialStateSigma_.sigvxvyvz.x() << ", " 
                                                                               << InitialStateSigma_.sigvxvyvz.y() << ", "
                                                                               << InitialStateSigma_.sigvxvyvz.z() << "]    [m/s]     |" << std::endl;                                                                            
    std::cout << "|   - Orientation:  [" << std::fixed << std::setprecision(4) << InitialStateSigma_.sigrpy.x() << ", " 
                                                                               << InitialStateSigma_.sigrpy.y() << ", "
                                                                               << InitialStateSigma_.sigrpy.z() << "]    [rad]     |" << std::endl;
    std::cout << "|                                                         |" << std::endl;

    std::cout << "| Initial time:   " << initialtime << "    [s]                          |" << std::endl;
    std::cout << "|                                                         |" << std::endl;
    std::cout << "| Preintegration sigma:         [" << PreintegrationSig.x() << ", " << PreintegrationSig.y() << ", " << PreintegrationSig.z() << "]  |" << std::endl;
    std::cout << "|                                                         |" << std::endl;
    std::cout << "| ISAM parameters:           " << "                             |" << std::endl;
    std::cout << "|   - relinearizeSkip:       " << std::fixed << std::setprecision(0) << relinearizeSkip << " [-]                        |" << std::endl;
    std::cout << "|   - relinearizeThreshold:  " << std::fixed << std::setprecision(4) << relinearizeThreshold << " [-]                   |" << std::endl;
    std::cout << "|   - dt pose:               " << std::fixed << std::setprecision(4) << poserate << " [s]                   |" << std::endl;
    std::cout << "|   - updaterate:            " << std::fixed << std::setprecision(0) << updaterate << " [-]                       |" << std::endl;
    std::cout << "| ________________________________________________________|" << std::endl;
}