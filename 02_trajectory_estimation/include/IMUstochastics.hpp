// include/Config.h

#ifndef CONFIG_H
#define CONFIG_H

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

class IMUstochastics {
public:
    struct Sig {
        double x;
        double y;
        double z;
    };

    Sig sigacc;
    Sig siggyro;
    Sig sigaccbias;
    Sig siggyrobias;

private:    

public:
    IMUstochastics() = default;

    void loadFromFile(const std::string& filename);

    Sig getSigAcc() const;
    Sig getSigGyro() const;
    Sig getSigAccBias() const;
    Sig getSigGyroBias() const;

    void print() const;
};

#endif // CONFIG_H
