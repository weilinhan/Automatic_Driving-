/**
 * @file shutdown.cpp
 * @brief the implementation of the shutdown state.
 * @author PSAF
 * @date 2022-06-01
 */
#include <iostream>
#include "psaf_state_machine/basic_cup/states/shutdown.hpp"

void Shutdown::entry()
{
  std::cerr << "Critical error, can not recover, shutting down" << std::endl;
}
