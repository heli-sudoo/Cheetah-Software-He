/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "MHPC_LLController.h"

int main(int argc, char** argv) {
  main_helper(argc, argv, new MHPC_LLController());
  return 0;
}
