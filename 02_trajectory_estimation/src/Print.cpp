/**
 * @brief functions to print progress
 * @author Felix Esser
 * Jan 2024
 */

#include "../include/Print.hpp"

#include <iostream>
#include <string>

void Print::printProgress(float progress, int ts1, int ts2, int tsbase, int gp, int gp_head) {

  int barWidth = 70;

  std::cout << "[";
  int pos = barWidth * progress;
  for (int i = 0; i < barWidth; ++i) {
    if (i < pos)
      std::cout << "=";
    else if (i == pos)
      std::cout << ">";
    else
      std::cout << " ";
  }
  //std::cout << "] " << int(progress * 100.0) << " % " << " | use data of: -> GNSS " << gp << " | GNSS Heading " << gp_head <<" | TS 1 " <<  ts1 << " | TS 2 " <<  ts2 << " | TS baseline " <<  tsbase << " |"  << "\r";
  std::cout << "] " << int(progress * 100.0) << " % "
          << " | use data of -> "
          << (gp ? "\033[32mGNSS\033[0m" : "\033[31mGNSS\033[0m") << " | "
          << (gp_head ? "\033[32mGNSS Heading\033[0m" : "\033[31mGNSS Heading\033[0m") << " | "
          << (ts1 ? "\033[32mTS 1\033[0m" : "\033[31mTS 1\033[0m") << " | "
          << (ts2 ? "\033[32mTS 2\033[0m" : "\033[31mTS 2\033[0m") << " | "
          << (tsbase ? "\033[32mTS baseline\033[0m" : "\033[31mTS baseline\033[0m")
          << " |" << "\r";

  std::cout.flush();
  std::cout << std::endl;
}

void Print::print_title( const char* title ) {
  std::cout << "\n-------------------------------------------------------------"
               "-----------------\n"
            << title
            << "\n-------------------------------------------------------------"
               "-----------------\n"
            << std::endl;
}

void Print::print_title2(std::string title) {
  std::string ap = "\n\n---- ";
  ap += title + " ----";

  if (ap.size() < 80) {
    while (ap.size() != 80) {
      ap += "-";
    }
  }

  std::cout << ap << std::endl;
}
