#ifndef WANDER_BOT_SRC_WANDERER_H_
#define WANDER_BOT_SRC_WANDERER_H_
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Wanderer{
public:
    // Tunable parameters
    static constexpr double FORWARD_SPEED             = 0.3;
    static constexpr double L_MIN_SCAN_ANGLE          = 0.0   / 180 * M_PI;
    // static constexpr double L_MAX_SCAN_ANGLE          = 20.0  / 180 * M_PI;
    static constexpr double L_MAX_SCAN_ANGLE          = 40.0  / 180 * M_PI;
    // static constexpr double L_MAX_SCAN_ANGLE          = 30.0  / 180 * M_PI;
    static constexpr double L_XTREME_SCAN_ANGLE       = 90.0  / 180 * M_PI;

    static constexpr double R_XTREME_SCAN_ANGLE       = 270.0 / 180 * M_PI;
    // static constexpr double R_MIN_SCAN_ANGLE          = 340.0 / 180 * M_PI;
    static constexpr double R_MIN_SCAN_ANGLE          = 320.0 / 180 * M_PI;
    // static constexpr double R_MIN_SCAN_ANGLE          = 330.0 / 180 * M_PI;
    static constexpr double R_MAX_SCAN_ANGLE          = 360.0 / 180 * M_PI;
    static constexpr float  MIN_DIST_FROM_OBSTACLE    = 0.5;   // Should be smaller than sensor_msgs::LaserScan::range_max
    // static constexpr float  MIN_DIST_FROM_OBSTACLE    = 0.7;   // Should be smaller than sensor_msgs::LaserScan::range_max
    Wanderer();
    void startMoving();

private:
    ros::NodeHandle node;
    ros::Publisher  commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub;   // Subscriber to the robot's laser scan topic
    bool keepMoving;            // Indicates whether the robot should make turning
    bool turnRightFlag;         // Indicates whether the robot should make turning
    bool hasChosen;             // Flag of choosing turn left or right
    bool lockTurn;

    void moveForward();
    void turnLeft();
    void turnRight();
    void stopMove();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

#endif