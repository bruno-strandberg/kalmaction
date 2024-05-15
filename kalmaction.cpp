
#include <boost/program_options.hpp>
#include <iostream>
#include "Drone.h"
#include <filesystem>

namespace po = boost::program_options;

struct ParConf {
  double acc_std;
  double pos_std;
  ParConf(double _acc_std, double _pos_std) : acc_std(_acc_std), pos_std(_pos_std) {};
};

void runSim(ParConf conf, std::filesystem::path plots_path) {
  Drone drone1;
  drone1.setDest(1000, 1000);
  drone1.setAccStd(conf.acc_std);
  drone1.setPosStd(conf.pos_std);
  drone1.flyToDest(3);
  auto map = drone1.getMap();
  std::string pic_name = std::to_string(conf.acc_std) + "_" + std::to_string(conf.pos_std) + ".png";
  std::filesystem::path pic_path = plots_path / pic_name;
  try {
    cv::imwrite(pic_path.string(), map);
  }
  catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    std::cout << "Error: picture write failed" << std::endl;
  }
}

int main(int argc, char* argv[]) {

  bool run_exps = false;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "produce help message")
    ("x", po::value<int>(), "x coordinate")
    ("y", po::value<int>(), "y coordinate")
    ("run_experiments,r", po::bool_switch(&run_exps)->default_value(false), "Run experiments");
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
  
  std::vector<ParConf> par_confs;

  // configurations for scanning the effect of IMU std
  for (double _acc_std = 0.1; _acc_std <= 3; _acc_std += 0.1) {
    par_confs.push_back( ParConf(_acc_std, 3) );
  }

  // configurations for scanning the effect of POS std
  for (double _pos_std = 0.5; _pos_std <= 30; _pos_std += 1) {
    par_confs.push_back( ParConf(0.5, _pos_std) );
  }

  // configurations for increasing both to a breaking point
  double _acc_std = 0.1, _pos_std = 0.5;
  for (int i = 0; i < 10; i++) {
    par_confs.push_back( ParConf(_acc_std, _pos_std) );
    _acc_std += 0.5;
    _pos_std += 1.0;
  }

  
  if (run_exps) {
    std::cout << "NOTICE running experiments" << std::endl;
    std::filesystem::path dirPath("plots");
    if (!std::filesystem::exists(dirPath)) {
      auto success = std::filesystem::create_directory(dirPath);
      if (!success) {
        throw std::runtime_error("Failed to create the directory");
      }
    }
    for(const auto& conf: par_confs) {
      runSim(conf, dirPath);
    }
  }
  
  
  // Your code here
  Drone drone1;
  drone1.setDest(x, y);
  drone1.setAccStd(0.1);
  drone1.flyToDest(3);
  auto mat = drone1.getMap();
  cv::namedWindow("drone1");
  cv::imshow("drone1", mat);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return 0;
}
