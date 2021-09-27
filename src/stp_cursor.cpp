#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

#include "../include/stpTeleOperationCursor.h"
#include "../include/stpJsonParser.h"
#include "../include/stpInputParser.h"


inline bool fileExists(const std::string& filename) {
  struct stat buffer;
  return (stat (filename.c_str(), &buffer) == 0);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "CURSOR");
  InputParser input(argc, argv);

  if (input.cmdOptionExists("-h")) {
    ROS_INFO("-h : Help lists all possible flags and their associated actions");
    ROS_INFO("-j : Expects a JSON file with all topic names necessary to initialize stpTeleOperationCursor Object");
    return -1;
  }
  const std::string& filename = input.getCmdOption("-j");
  if (!filename.empty()) {
    // Check if file exists
    if (fileExists(filename)) {
      // Parse JSON file with topic names
      stpTeleOperationCursor cursor(filename);
      while (ros::ok()) {
        cursor.Run();
      }
    } else {
      ROS_ERROR("%s does not exist. Could not load ROS Topic names", filename.c_str());
      return -2;
    }
  } else {
    ROS_ERROR("File name not properly allocated. Exiting.");
    return -3;
  }

  return 0;
}