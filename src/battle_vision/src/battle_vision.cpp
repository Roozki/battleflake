
#include <BattleVision.h>


int main(int argc, char **argv){
    // Setup your ROS node
    std::string node_name = "battle_vision";

    // Create an instance of your class
    BattleVision battle_vision(argc, argv, node_name);

    // Start up ros. This will continue to run until the node is killed
    ros::spin();

    // Once the node stops, return 0
    return 0;
}