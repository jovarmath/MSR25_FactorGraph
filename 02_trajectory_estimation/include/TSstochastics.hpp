// include/Config.h

#ifndef CONFIG_HTS
#define CONFIG_HTS

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <Eigen/Core>


class TSstochastics {
public:
    struct Sig {
        double x;
        double y;
        double z;
    };

    Sig sigpos;

private:    

public:
    TSstochastics() = default;

    void loadFromFile(const std::string& filename);

    Sig getSigxyz() const;

    void print() const;
};

#endif // CONFIG_H
