/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#ifndef RosIntegration_H
#define RosIntegration_H
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <bb_msgs/battleCmd.h>
#include <string>

static geometry_msgs::Twist twist_message_controller;
static geometry_msgs::Twist twist_message_left;
static geometry_msgs::Twist twist_message_right;

static bb_msgs::battleCmd cmd_msg;

class RosIntegration {
  public:
    RosIntegration();
    ~RosIntegration();

    // twist topic

    static void
    twist_controller_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_controller = *twist_msg;
    }

    static void
    twist_left_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_left = *twist_msg;
    }

    static void
    twist_right_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_right = *twist_msg;
    }

    static void battleCallback(const bb_msgs::battleCmd::ConstPtr& msg){
        cmd_msg = *msg;
    }

    void twist_subscriber() {
        ros::Rate loop_rate(100);
        twist_controller_sub =
        n->subscribe("/cmd_vel", 100, twist_controller_callback);
        twist_left_sub = n->subscribe(
        "/micro_comm/robot_1_CMD", 50, battleCallback);
        twist_right_sub = n->subscribe(
        "/integration_node/rwheels_pub_topic", 100, twist_right_callback);
        loop_rate.sleep();
        ros::spinOnce();
    }

  private:
    ros::NodeHandle* n;

    // twist topics
    ros::Subscriber twist_controller_sub;
    ros::Subscriber twist_left_sub;
    ros::Subscriber twist_right_sub;
};

#endif // RosIntegration_H
