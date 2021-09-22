//
// Created by dvrk-1804 on 2021-09-17.
//
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>

class Cursor {
 private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Timer timer;
  std::string direction = "right";
  geometry_msgs::TransformStamped mtmr_cp;

 public:
  Cursor() :
  nh(),
  pub(nh.advertise<geometry_msgs::TransformStamped>("cursor/transform", 10)),
  timer(nh.createTimer(ros::Duration(0.01), &Cursor::main_loop, this)) {
    mtmr_cp.transform.translation.x = 0;
    mtmr_cp.transform.translation.y = 0;
    mtmr_cp.transform.translation.z = -15;
  }

  void main_loop(const ros::TimerEvent &e) {
    try {
      timer.isValid();
      if (direction == "right") {
        mtmr_cp.transform.translation.x += fRand();
        if (mtmr_cp.transform.translation.x > 8.0) {
          direction = "left";
        }
      } else if (direction == "left") {
        mtmr_cp.transform.translation.x -= fRand();
        if (mtmr_cp.transform.translation.x < -8.0) {
          direction = "right";
        }
      }
      pub.publish(mtmr_cp);

    } catch (ros::Exception& e) {
      ROS_INFO("ROS exception thrown: %s", e.what());
    }
  }

  double fRand() {
    double f = (double)rand()/10000000000;
    return f;
  }

};

int main(int argc, char** argv) {
  std::srand(time(NULL));
  ros::init(argc, argv, "cursor");
  Cursor cursor;
  ros::spin();

  return 0;
}