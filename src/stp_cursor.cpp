#include "../include/stpTeleOperationCursor.h"
#include <jsoncpp/json/value.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

inline bool fileExists(const std::string& filename) {
  struct stat buffer;
  return (stat (filename.c_str(), &buffer) == 0);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "CURSOR");

  try {
    po::options_description desc("Allowed Options");
    desc.add_options()("help", "JSON");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (std::exception &e) {
    ROS_INFO("Error occured with command line parser: %s", e.what());
  }

  stpTeleOperationCursor cursor;

  while (ros::ok()) {
    cursor.Run();
  }

  return 0;
}