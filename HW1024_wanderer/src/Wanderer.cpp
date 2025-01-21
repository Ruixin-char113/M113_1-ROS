#include "Wanderer.h"
#include "geometry_msgs/Twist.h"
#include "stdlib.h"
#include "time.h"

Wanderer::Wanderer(){
    keepMoving    = true;
    turnRightFlag = false;
    lockTurn      = false;

    // Advertise a new publisher for the robot's velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the simulated robot's laser scan topic
    laserSub = node.subscribe("scan", 1, &Wanderer::scanCallback, this);
}

// Send a velocity command
void Wanderer::moveForward(){
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    commandPub.publish(msg);
}

// Send a zero velocity command
void Wanderer::stopMove(){
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    commandPub.publish(msg);
    ROS_INFO("STOP!");
}

// Send a angular plus rate command
void Wanderer::turnLeft(){
    geometry_msgs::Twist msg;
    msg.angular.z = 0.5;
    commandPub.publish(msg);
}

// Send a angular minus rate command
void Wanderer::turnRight(){
    geometry_msgs::Twist msg;
    msg.angular.z = -0.5;
    commandPub.publish(msg);
}

// Process the incoming laser scan message
void Wanderer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    bool isObstacleInFront      = false;
    bool isObstacleInLeftFront  = false;
    bool isObstacleInRightFront = false;

    // Find the closest range between of the front-left part
    int lminIndex    = ceil((L_MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int lmaxIndex    = floor((L_MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int lxtremeIndex = floor((L_XTREME_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for(int currIndex = lminIndex + 1; currIndex <= lmaxIndex; currIndex++){
        if(scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE){
            isObstacleInFront = true;
            break;
        }
    }

    for(int currIndex = lmaxIndex; currIndex <= lxtremeIndex; currIndex++){
        if(scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE){
            isObstacleInLeftFront = true;
            break;
        }
    }

    // Find the closest range between of the front-right part
    int rminIndex    = ceil((R_MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int rmaxIndex    = floor((R_MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    int rxtremeIndex = ceil((R_XTREME_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);

    for(int currIndex = rminIndex + 1; currIndex <= rmaxIndex; currIndex++){
        if(scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE){
            isObstacleInFront = true;
            break;
        }
    }

    for(int currIndex = rxtremeIndex + 1; currIndex <= rminIndex; currIndex++){
        if(scan->ranges[currIndex] < MIN_DIST_FROM_OBSTACLE){
            isObstacleInRightFront = true;
            break;
        }
    }

    // SELF
    // Check obstacle
    if(isObstacleInFront){
        keepMoving = false;

        // Left clear
        if(isObstacleInRightFront && !isObstacleInLeftFront){
            turnRightFlag = 0;
            return;

        // Right clear
        }else if(isObstacleInLeftFront && !isObstacleInRightFront){
            turnRightFlag = 1;
            return;

        // Random
        }else if(!lockTurn){
            srand(time(NULL));
            int x = rand();
            turnRightFlag = x % 2; 
            // Lock
                lockTurn = true;
            return;
        }
    }else{
        keepMoving = true;
        lockTurn   = false;
    }
}

void Wanderer::startMoving(){
    ros::Rate rate(10);
    ROS_INFO("Start moving");

    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while(ros::ok()){
        if(keepMoving)
            moveForward();
        else{
            stopMove();
            if(turnRightFlag)
                turnRight();
            else
                turnLeft();
        }
        ros::spinOnce();
        rate.sleep();
    }
}