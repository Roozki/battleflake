/*
 * Created By: Ihsan Olawale, Rowan Zawadski
 * Created On: July 17th, 2022
 * Description: A node that subscribes and scans arcu codes on with the
 * realsense cameras
 */

#include <BattleMarker.h>

int main(int argc, char** argv) {
    // Setup your ROS node
    std::string node_name = "battle_marker";

    // Create an instance of your class
    BattleMarker battle_marker(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}
