// include/Config.h

#ifndef CONFIG_H2
#define CONFIG_H2

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

class GPSstochastics {
public:
    struct Sig {
        double x;
        double y;
        double z;
    };

    Sig sigpos;

private:    

public:
    GPSstochastics() = default;

    void loadFromFile(const std::string& filename);

    Sig getSigxyz() const;

    void print() const;
};

#endif // CONFIG_H
