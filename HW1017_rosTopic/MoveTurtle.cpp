#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "math.h"

#define FORWARD_SPEED_MPS    2.0
#define ROTATIONAL_FREQUENCY 1.0

#define DISTANCE_DEVIATION   0.1
#define THETA_DEVIATION      0.01

#define PI                   3.14
#define COMMAND_PARAMETER_N  2

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

float countTheta(const float moveCommand[], const int tempMoveStep){
    int tempReadPointer = tempMoveStep * COMMAND_PARAMETER_N;
    float diff_theta  = 0.0;
    float distance_nx = 0.0;
    float distance_ny = 0.0;
    float distanceN   = 0.0;
    float dest_theta  = 0.0;

    countDistance(moveCommand, tempReadPointer, &distance_nx, &distance_ny, &distanceN);

    if(abs(distance_nx) < 0.1){
        if(distance_ny > 0)
            return PI / 2 - turtle_theta;
        else if(distance_ny < 0)
            return -(PI / 2) - turtle_theta;
        else
            return 0.0;
    }else
        dest_theta = acos(distance_nx / distanceN);

    // (X < X' && Y > Y') || (X > X' && Y < Y')
    // if((distance_nx > 0 && distance_ny <= 0) ||
    //    (distance_nx <= 0 && distance_ny > 0))
    if(distance_ny < 0)
        dest_theta = 0 - abs(dest_theta);
    else
        dest_theta = abs(dest_theta);

    diff_theta = dest_theta - turtle_theta;
    // ROS_INFO("dis_x: %.2f, dis_y: %.2f, diff_theta: %.2f", distance_nx, distance_ny, diff_theta);
    return diff_theta;
}

// Init msg
geometry_msgs::Twist initMsg(){
    geometry_msgs::Twist msg;
    /*
        FORMAT:
            msg.linear.x = FORWARD_SPEED_MPS;    
            msg.linear.y = FORWARD_SPEED_MPS;
            msg.angular.z = 3;
    */
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.angular.z= 0.0;

    return msg;
}

// Set msg
geometry_msgs::Twist setMsg(const float moveCommand[], int *moveStep, const int mvStepLimit, bool *outputFlag){
    float distance_x = 0.0;
    float distance_y = 0.0;
    float distance   = 0.0;
    float diff_theta = 0.0;

    // Init temp msg
    geometry_msgs::Twist msg;
    msg = initMsg();

    // Set moveCommand[readPointer] = x
    int readPointer = *moveStep * COMMAND_PARAMETER_N;

    // Count distance
    countDistance(moveCommand, readPointer, &distance_x, &distance_y, &distance);

    // Move to dest node
    if(!(distance < DISTANCE_DEVIATION)){
        diff_theta    = countTheta(moveCommand, *moveStep);
        msg.linear.x  = distance;
        msg.angular.z = diff_theta;
        //ROS_INFO("Distance x: %f, Distance y: %f", distance_x, distance_y);
        return msg;
    // Check position
    }else{
        // Print State
        if(*outputFlag == false){
            ROS_INFO("Node: %d, x: %.2f, y: %.2f, theta: %.2f", *moveStep, turtle_x, turtle_y, (turtle_theta / PI) * 180);
            *outputFlag = true;
        }

        // Count diff theta
        int tempMoveStep = (*moveStep + 1) % mvStepLimit;
        diff_theta = countTheta(moveCommand, tempMoveStep);

        // Rotate to dest_theta
        if(!(abs(diff_theta) < THETA_DEVIATION)){
            //ROS_INFO("diff_theta: %.2f", diff_theta);
            msg = initMsg();
            msg.angular.z = diff_theta * ROTATIONAL_FREQUENCY;
            return msg;

        // Next node
        }else{
            msg = initMsg();
            *outputFlag = false;
            // Avoid moveStep overflow
                *moveStep += 1;
                *moveStep %= mvStepLimit;
            return msg;
        }
    }
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
    // COMMAND_PARAMETER_N = 2 : x, y
    const float moveCommand[] = {
        5.54,    5.54,
        5.5,    0.5,
        10.5,   5.5,
        5.5,    10.5,
        0.5,    5.5,
        5.5,    0.5 
    };
    // Number of data
    int mvStepLimit = (sizeof(moveCommand) / sizeof(moveCommand[0])) / COMMAND_PARAMETER_N;

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

    bool outputFlag = false;
    // Loop at 10Hz, publishing movement commands until we shut down
    ros::Rate rate(10);
    ROS_INFO("Starting to move forward");
    while (ros::ok()) {
        pub.publish(msg);
        ros::spinOnce(); // Allow processing of incoming messages

        // Update msg
        msg = setMsg(moveCommand, &moveStep, mvStepLimit, &outputFlag);
        // ROS_INFO("moveStep: %d", moveStep);
        // ROS_INFO("x: %f, y: %f", msg.linear.x, msg.linear.y);

        rate.sleep();
    }
}