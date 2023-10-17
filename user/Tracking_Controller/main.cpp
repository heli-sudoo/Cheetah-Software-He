/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "TrackingController.h"

int main(int argc, char** argv) {
  main_helper(argc, argv, new Tracking_Controller());
  return 0;
}
