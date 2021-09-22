//
// Created by dvrk-1804 on 2021-09-13.
//
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <vector>
#include <crtk_msgs/operating_state.h>

class mtmFollower {
 private:
  ros::NodeHandle nh;
  ros::Subscriber mtmr_primary_setpoint_js, mtmr_secondary_setpoint_js;
  ros::Publisher mtmr_secondary_servo_jp, mtmr_secondary_move_jp;
  ros::Timer timer;
  sensor_msgs::JointState primary_js, secondary_js;

  bool isAligned(const std::vector<double>& position_primary, const std::vector<double>& position_secondary) const {
    std::vector<double> delta;
    bool result = true;
    for (int i = 0; i < position_primary.size(); ++i) {
      delta.push_back(abs(position_primary[i] - position_secondary[i]));
      if (delta[i] <= 0.075) {
        delta[i] = 1;
      } else {
        delta[i] = 0;
      }
    }
    for (auto state:delta) {
      //std::cout << "state: " << state << '\n';
      state == 1 ? result &= true : result &= false;
    }
    //std::cout << "Results is: " << result << std::endl;
    return result;
  }
 public:
  mtmFollower() :
  nh(),
  mtmr_primary_setpoint_js(nh.subscribe("/MTMR_PRIMARY/measured_js", 10, &mtmFollower::rightPrimaryCallback, this)),
  mtmr_secondary_setpoint_js(nh.subscribe("/MTMR_SECONDARY/measured_js", 10, &mtmFollower::rightSecondaryCallback, this)),
  mtmr_secondary_servo_jp(nh.advertise<sensor_msgs::JointState>("/MTMR_SECONDARY/servo_jp", 10)),
  mtmr_secondary_move_jp(nh.advertise<sensor_msgs::JointState>("/MTMR_SECONDARY/move_jp", 10)),
  timer(nh.createTimer(ros::Duration(0.001), &mtmFollower::main_loop, this)) {
  }

  void rightPrimaryCallback(const sensor_msgs::JointState& msg) {
    try {
      //ROS_INFO("getting primary messages");
      primary_js.name = msg.name;
      primary_js.position = msg.position;
    } catch (ros::Exception& e) {
      ROS_ERROR("Error occurred: %s ", e.what());
    }
  }

  void rightSecondaryCallback(const sensor_msgs::JointState& msg) {
    try {
      //ROS_INFO("getting secondary messages");
      secondary_js.name = msg.name;
      secondary_js.position = msg.position;
    } catch (ros::Exception& e) {
      ROS_ERROR("Error occurred: %s ", e.what());
    }
  }

  void main_loop(const ros::TimerEvent &e) const {
    ROS_INFO("Time between callbacks: %f", (e.current_real - e.last_real).toSec());
    try {
      mtmr_primary_setpoint_js.getTopic();
      mtmr_secondary_setpoint_js.getTopic();
      timer.isValid();

      if (isAligned(primary_js.position, secondary_js.position)) {
        mtmr_secondary_servo_jp.publish(primary_js);
      } else {
        mtmr_secondary_move_jp.publish(primary_js);
      }


    } catch (ros::Exception& e) {
      ROS_ERROR("Error occurred: %s ", e.what());
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mtm_follower");
  mtmFollower node;
  ros::spin();
  return 0;
}

