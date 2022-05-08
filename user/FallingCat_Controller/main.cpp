/*!
 * @file main.cpp
 * @brief Main Function for the falling cat controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "FallCat_Controller.hpp"

int main(int argc, char** argv) {
  main_helper(argc, argv, new FallCat_Controller());
  return 0;
}
