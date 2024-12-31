// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <tf/transform_datatypes.h>

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// int main(int argc, char** argv){
//   ros::init(argc, argv, "send_goals_node");

//   // Get the goal;s x, y and angle from the launch file
//   double x, y, theta;
//   ros::NodeHandle nh;
//   nh.getParam("goal_x", x);
//   nh.getParam("goal_y", y);
//   nh.getParam("goal_theta", theta);

//   // create the action client
//   MoveBaseClient ac("move_base", true);

//   //wait for the action server to come up
//   while(!ac.waitForServer(ros::Duration(5.0))){
//     ROS_INFO("Waiting for the move_base action server to come up");
//   }

//   move_base_msgs::MoveBaseGoal goal;

//   //we'll send a goal to the robot to move 1 meter forward
// //   goal.target_pose.header.frame_id = "base_link";
//   goal.target_pose.header.frame_id = "map";
//   goal.target_pose.header.stamp = ros::Time::now();

// //   goal.target_pose.pose.position.x = 1.0;
// //   goal.target_pose.pose.orientation.w = 1.0;
//     goal.target_pose.pose.position.x = x;
//     goal.target_pose.pose.position.y = y;

//     double radians = theta * (M_PI / 180);
//     tf::Quaternion quaternion;
//     quaternion = tf::createQuzternionFromYaw(radians);

//     geometry_msgs::Quaternion qMsg;
//     tf::quaternionTFToMsg(quaternion, qMsg);

//     goal.target_pose.pose.orientation = qMsg;

// //   ROS_INFO("Sending goal");
//     ROS_INFO("Sending goal x = %f, y = %f, theta = %f", x, y, theta);
//   ac.sendGoal(goal);

//   ac.waitForResult();

//   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//     ROS_INFO("Hooray, the base moved 1 meter forward");
//   else
//     ROS_INFO("The base failed to move forward 1 meter for some reason");

//   return 0;
// }
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <cmath>

using namespace move_base_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const MoveBaseResultConstPtr& result){
  ROS_INFO("You have reached the goal!");
}

void activeCb(){
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const MoveBaseFeedbackConstPtr& feedback){
  float w = feedback->base_position.pose.orientation.w;
  float x = feedback->base_position.pose.orientation.x;
  float y = feedback->base_position.pose.orientation.y;
  float z = feedback->base_position.pose.orientation.z;
  float yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  ROS_INFO("Current robot pose: x = %f, y = %f, theta = %f", 
              feedback->base_position.pose.position.x,
              feedback->base_position.pose.position.y,
              (yaw/3.14)*180
              );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goals_node");

    // Get the goal's x, y and angle from the launch file
    double x, y;
    double theta = 90.0;
    ros::NodeHandle nh;
    nh.getParam("goal_x", x);
    nh.getParam("goal_y", y);
    nh.getParam("goal_theta", theta);

    // create the action client 
    MoveBaseClient ac("move_base", true);

    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");

        // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();   

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

    // Convert the Euler angle to quaternion
    double radians = theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);

    goal.target_pose.pose.orientation = qMsg;

    // Send the goal command
    ROS_INFO("Sending robot to: x = %f, y = %f, theta = %f", x, y, theta);
    // ac.sendGoal(goal);
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    // Wait for the action to return
    // ac.waitForResult();

    // if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_INFO("You have reached the goal!");
    // else
    //     ROS_INFO("The base failed for some reason");

    ros::spin();

    return 0;
}