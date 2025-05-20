#ifndef CONFIG_iSAMConfig
#define CONFIG_iSAMConfig

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <Eigen/Core>

class iSAMConfig {
public:

    struct InitialstateSig {
        Eigen::Vector3d sigxyz;
        Eigen::Vector3d sigvxvyvz;
        Eigen::Vector3d sigrpy;
    };

    bool useGPS;
    bool useGPSheading;
    bool useTS1;
    bool useTS2;
    bool useTSbaseline;
    double initialtime;

    Eigen::Vector3d PreintegrationSig;

    int relinearizeSkip;
    double relinearizeThreshold;
    double poserate;
    int updaterate;   

    InitialstateSig InitialStateSigma_;

private:    

public:
    iSAMConfig() = default;

    void loadFromFile(const std::string& filename);

    bool getuseGPS() const;
    bool getuseGPSheading() const;
    bool getuseTS1() const;
    bool getuseuseTS2() const;
    bool getuseuseTSbaseline() const;
    bool getInitialStateSigma() const;
    double getinitialtime() const;

    int getrelinearizeSkip() const;
    double getrelinearizeThreshold() const;
    double getposerate() const;
    int getupdaterate() const;

    void print() const;
};

#endif // CONFIG_H
