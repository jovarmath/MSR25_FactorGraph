// include/Config.h

#ifndef CONFIG_HTSb
#define CONFIG_HTSb

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <Eigen/Core>


class TSbaselinestochastics {
public:
    struct Sig {
        double x;
        double y;
        double z;
    };

    Sig sigbase;

private:    

public:
    TSbaselinestochastics() = default;

    void loadFromFile(const std::string& filename);

    Sig getSigxyz() const;

    void print() const;
};

#endif // CONFIG_H
