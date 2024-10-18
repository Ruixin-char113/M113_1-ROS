#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"

#define FORWARD_SPEED_MPS   2.0
#define DISTANCE_DEVIATION  0.1
#define THETA_DEVIATION     0.1
#define PI                  3.14
#define COMMAND_PARAMETER_N 2

// For turtle's current state
float turtle_x      = 5.5;
float turtle_y      = 5.5;
float turtle_theta  = 0.0;

// Store current move step
int moveStep = 0;

// Count distance
void countDistance(const float moveCommand[],const int readPointer, float *distance_x, float *distance_y, float *distance){
    *distance_x = moveCommand[readPointer]     - turtle_x;
    *distance_y = moveCommand[readPointer + 1] - turtle_y;
    *distance   = sqrt(pow(*distance_x, 2) + pow(*distance_y, 2));
}

// Set msg
geometry_msgs::Twist setMsg(const float moveCommand[], int *moveStep, const int mvStepLimit){
    //temp msg & default value
    geometry_msgs::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        //msg.angular.z= 0.0;
    /*
        FORMAT:
            msg.linear.x = FORWARD_SPEED_MPS;    
            msg.linear.y = FORWARD_SPEED_MPS;
            msg.angular.z = 3;
    */
    
    // Set moveCommand[readPointer] = x
    int readPointer = *moveStep * 2;

    // Count distance
    // float distance_x = moveCommand[readPointer]     - turtle_x;
    // float distance_y = moveCommand[readPointer + 1] - turtle_y;
    // float distance   = sqrt(pow(distance_x, 2) + pow(distance_y, 2));
    float distance_x = 0.0;
    float distance_y = 0.0;
    float distance   = 0.0;
    countDistance(moveCommand, readPointer, &distance_x, &distance_y, &distance);

    // Check position
    //if(abs(distance_x) < DISTANCE_DEVIATION && abs(distance_y) < DISTANCE_DEVIATION){
    if(distance < DISTANCE_DEVIATION){
        // // Avoid moveStep overflow
        // *moveStep += 1;
        // *moveStep %= mvStepLimit;
        // readPointer = *moveStep * 2;

        // float distance_nx= 
        // float diff_theta = turtle_theta - (acos(distance_x / distance) / PI) * 180;

        // Next node
        // if(diff_theta < THETA_DEVIATION){
            ROS_INFO("Node: %d, x: %.2f, y: %.2f, theta: %.2f", *moveStep, turtle_x, turtle_y, turtle_theta);
            // Avoid moveStep overflow
            *moveStep += 1;
            *moveStep %= mvStepLimit;
            readPointer = *moveStep * 2;            
        // }
        
        // ROS_INFO("diff_theta: %.2f", diff_theta);
        return msg;
    }

    msg.linear.x = distance_x * FORWARD_SPEED_MPS;
    msg.linear.y = distance_y * FORWARD_SPEED_MPS;
    //msg.angular.z= 
    
    //ROS_INFO("Distance x: %f, Distance y: %f", distance_x, distance_y);
    return msg;
}

// Topic messages callback
void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    //ROS_INFO("x: %.2f, y: %.2f, theta: %.2f", msg->x, msg->y, msg->theta);
    turtle_x     = msg->x;
    turtle_y     = msg->y;
    turtle_theta = msg->theta;
    // ROS_INFO("turtle_x: %f, turtle_y: %f", turtle_x, turtle_y);
}

int main(int argc, char **argv)
{
    // COMMAND_PARAMETER_N: x, y
    const float moveCommand[] = {
        5.5,    5.5,
        5.5,    0.5,
        10.5,   5.5,
        5.5,    10.5,
        0.5,    5.5,
        5.5,    0.5 
    };

    // Initialize the node
    ros::init(argc, argv, "move_turtle");
    ros::NodeHandle node;

    // A publisher for the movement data
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

    // A listener for pose
    ros::Subscriber sub = node.subscribe("turtle1/pose", 10, poseCallback);
        // Drive forward at a given speed.  The robot points up the x-axis.
    // The default constructor will set all commands to 0
    geometry_msgs::Twist msg;

    // Loop at 10Hz, publishing movement commands until we shut down
    int mvStepLimit = (sizeof(moveCommand) / sizeof(moveCommand[0])) / COMMAND_PARAMETER_N;
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");
    while (ros::ok()) {
        pub.publish(msg);
        ros::spinOnce(); // Allow processing of incoming messages

        // Update msg
        msg = setMsg(moveCommand, &moveStep, mvStepLimit);
        // ROS_INFO("moveStep: %d", moveStep);
        // ROS_INFO("x: %f, y: %f", msg.linear.x, msg.linear.y);

        rate.sleep();
    }
}