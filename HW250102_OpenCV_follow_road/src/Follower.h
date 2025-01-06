#include <ros/ros.h>
#include <image_transport/image_transport.h>

class Follower {
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport imageTransport;
    image_transport::Subscriber imageSubscriber;
    ros::Publisher cmdVelPublisher;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
    Follower();
    virtual ~Follower();
};