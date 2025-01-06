
/*
* Follower.cpp
*
*
Created on: Nov 11, 2016
*
Author: viki
*/
#include "Follower.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/highgui/highgui_c.h>
#include <geometry_msgs/Twist.h>

static const std::string OPENCV_WINDOW = "Image window";
bool saw_blue = 0;
bool turn_blue = 0;
int  count_blue = 0;
bool saw_green = 0;

Follower::Follower(): imageTransport(nh) {
    imageSubscriber = imageTransport.subscribe("/camera/rgb/image_raw", 1, 
                                                &Follower::imageCallback, this);
    cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    //cmdVelPublisher = nh.advertise<geometry_msgs:: Twist>("/cmd_vel_mux/input/teleop", 10);
    
    // Create a display window
    // cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_NORMAL);
    cv::resizeWindow(OPENCV_WINDOW, 640, 360);
}

Follower::~Follower() {
    // Close the display window
    cv::destroyWindow(OPENCV_WINDOW);
}

cv::Mat maskRange(cv::Mat mask, int command){

    int width   = mask.cols;
    int height  = mask.rows;
    int search_top    = 3 * height / 4;
    int search_bottom = search_top + 20;

    // Yellow
    if(command == 0){
        search_top    = 3 * height / 4;
        search_bottom = search_top + 20;
    // Blue
    }else if(command == 1){
        search_top    = 3 * height / 4 - 17;
        search_bottom = search_top + 20;
    // Green
    }else if(command == 2){
        search_top    = 3 * height / 4 - 17;
        search_bottom = height - 3;
    }

    // Zero out pixels outside the desired region
    // For some reason, cannot modify the last two rows
    for (int y = 0; y < height - 2; y++) 
    {
        if (y < search_top || y > search_bottom) {
            for (int x=0; x < width; x++) {
                mask.at<cv::Vec3b>(y, x)[0] = 0;
                mask.at<cv::Vec3b>(y, x)[1] = 0; 
                mask.at<cv::Vec3b>(y, x)[2] = 0;
            }
        }
    }

    return mask;
}

void Follower::imageCallback(const sensor_msgs::ImageConstPtr& msg) { 
    // convert the ROS image message to a CvImage
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert input image to HSV
    cv::Mat image = cv_ptr->image;
    cv::Mat hsvImage;
    cv:cvtColor(image, hsvImage, CV_BGR2HSV);

    // Threshold the HSV image, keep only the yellow pixels 
    cv::Scalar lower_yellow(20, 100, 100);
    cv::Scalar upper_yellow(30, 255, 255);
    cv::Mat mask;
    cv::inRange(hsvImage, lower_yellow, upper_yellow, mask);

    // Threshold the HSV image, keep only the blue pixels 
    cv::Scalar lower_blue(110, 100, 100);
    cv::Scalar upper_blue(130, 255, 255);
    cv::Mat mask_blue;
    cv::inRange(hsvImage, lower_blue, upper_blue, mask_blue);

    // Threshold the HSV image, keep only the green pixels 
    cv::Scalar lower_green(50, 100, 100);
    cv::Scalar upper_green(70, 255, 255);
    cv::Mat mask_green;
    cv::inRange(hsvImage, lower_green, upper_green, mask_green);
    
    int width   = mask.cols;
    int height  = mask.rows;
    mask       = maskRange(mask, 0);
    mask_blue  = maskRange(mask_blue , 1);
    mask_green = maskRange(mask_green, 2);
    
    // Use the OpenCV moments() function to calculate the centroid of the blob of the binary image 
    cv::Moments M       = cv::moments(mask);
    cv::Moments M_blue  = cv::moments(mask_blue);
    cv::Moments M_green = cv::moments(mask_green);

    geometry_msgs::Twist cmd;
    int err;
    if (M.m00 > 0) {
        int cx = int(M.m10 / M.m00);
        int cy = int(M.m01 / M.m00);
        cv::circle(image, cv::Point(cx, cy), 20, CV_RGB(255, 0, 0), -1);

        // Move the robot in proportion to the error signal 
        err = cx - width / 2;
        cmd.linear.x = 0.2;
        cmd.angular.z = -(float)err / 500;
    }

    // Blue
    if(M_blue.m00 > 0){
        saw_blue = true;
        // if(count_blue >= 2){
        if(turn_blue){
            int bluex = int(M_blue.m10 / M_blue.m00);
            err = bluex - width / 2;
            err = -err;
            cmd.angular.z = -(float)err / 500;
        }
    }else if(saw_blue){
        saw_blue = false;
        // count_blue++;
        turn_blue = true;
    }

    // Green
    if(M_green.m00 > 0){
        saw_green = true;
        int cx = int(M_green.m10 / M_green.m00);
        err = cx - width / 2;
        cmd.linear.x = 0.2;
        cmd.angular.z = -(float)err / 500;
    }else if(saw_green){
        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }

    cmdVelPublisher.publish(cmd);
    
    // Update the GUI window
    cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(3);
}
