// include/Config.h

#ifndef CONFIG_H3
#define CONFIG_H3

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

class GPSPitchHeadingstochastics {
public:

    double sigpitch;
    double sigheading;

private:    

public:
    GPSPitchHeadingstochastics() = default;

    void loadFromFile(const std::string& filename);

    double getSigPitch() const;
    double getSigHeading() const;

    void print() const;
};

#endif // CONFIG_H
