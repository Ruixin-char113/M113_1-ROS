#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf_listener");

    ros::NodeHandle node;
    ros::service::waitForService("spawn");
        ros::ServiceClient add_turtle =node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    // set turtle2 init_pose
    srv.request.x = 5.5;
    srv.request.y = 5.5;
    add_turtle.call(srv);

    ros::Publisher turtle_vel =node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    
    tf::TransformListener listener;
    ros::Rate rate(10.0);

    while (node.ok()) {
        tf::StampedTransform transform;
        try {
            listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z = 4 * atan2(transform.getOrigin().y(),transform.getOrigin().x());
        vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +pow(transform.getOrigin().y(), 2));
        if(sqrt(pow(transform.getOrigin().x(), 2) +pow(transform.getOrigin().y(), 2)) < 1)
            vel_msg.linear.x = 0;
        turtle_vel.publish(vel_msg);
        rate.sleep();
    }
    return 0;
};
