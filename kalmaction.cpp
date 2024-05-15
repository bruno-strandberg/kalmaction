
#include <boost/program_options.hpp>
#include <iostream>
#include "Drone.h"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("x", po::value<int>(), "x coordinate")
    ("y", po::value<int>(), "y coordinate")
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

  int x = 1000;
  int y = 1000;
  
  // Check if option x is specified
  if (vm.count("x")) {
    x = vm["x"].as<int>();
    std::cout << "Option x is set to " << x << std::endl;
  } else {
    std::cout << "Option x was not set, using default " << x << std::endl;
  }

  if (vm.count("y")) {
    y = vm["y"].as<int>();
    std::cout << "Option y is set to " << y << std::endl;
  } else {
    std::cout << "Option y was not set, using default " << y << std::endl;
  }


  // Your code here
  Drone drone1;
  drone1.setDest(x, y);
  //drone1.setDest(-1000, 1000);
  //drone1.setDest(1000, -1000);
  //drone1.setDest(-1000, -1000);
  drone1.flyToDest(1);
  return 0;
}
