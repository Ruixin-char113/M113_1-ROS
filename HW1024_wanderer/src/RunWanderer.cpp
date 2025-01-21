#include "Wanderer.h"

int main(int argc, char **argv){
    // Initiate new ROS node named "wanderer"
    ros::init(argc, argv, "wanderer");

    // Create new wanderer object
    Wanderer wanderer;

    // Start the movement
    wanderer.startMoving();

    return 0;
};