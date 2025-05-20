/**
 * @brief library to print menssages and other
 * @author JOSE ANGEL MORAGA POSSELT, MSc STUDENT UNI BONN
 * Nov 2021
 *
 * TODO:
 */

#ifndef MMS03_INCLUDE_PRINT_HPP
#define MMS03_INCLUDE_PRINT_HPP

#include <string>

namespace Print {
// Function to print the progress
void printProgress(float progress, int ts1, int ts2, int tsbase, int gp, int gp_head);

// Function to print the title of a space
void print_title( const char* title);

// Function to print sub-title
void print_title2(std::string title);

};  // namespace Print

#endif