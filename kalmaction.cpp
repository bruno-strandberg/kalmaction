
#include <boost/program_options.hpp>
#include <iostream>
#include "Drone.h"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("x", po::value<int>(), "x in epsg 3301")
    ("y", po::value<int>(), "y in epsg 3301")
    ;

  // Parse command line arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  // Check if help option is specified
  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  // Check if option x is specified
  if (vm.count("x")) {
    std::cout << "Option x is set to " << vm["x"].as<int>() << ".\n";
  } else {
    std::cout << "Option x was not set.\n";
  }

  if (vm.count("y")) {
    std::cout << "Option x is set to " << vm["y"].as<int>() << ".\n";
  } else {
    std::cout << "Option y was not set.\n";
  }



  
  // Your code here
  Drone drone1;
  drone1.setDest(1000, 0);
  return 0;
  drone1.flyToDest(1);
  return 0;
}
