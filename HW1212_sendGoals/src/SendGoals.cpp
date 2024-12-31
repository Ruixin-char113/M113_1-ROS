#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>

int point_index = 0;
bool keeprun = 1;
std::vector<double> goal_xa, goal_ya, goal_thetaa;
move_base_msgs::MoveBaseGoal goal;

using namespace move_base_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Update or Set Goal
move_base_msgs::MoveBaseGoal setGoal(const std::vector<double> x, 
                                      const std::vector<double> y,
                                      const std::vector<double> theta){
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();   

  // Set x, y
  goal.target_pose.pose.position.x = x[point_index];
  goal.target_pose.pose.position.y = y[point_index];

  // Convert the Euler angle to quaternion
  double radians = theta[point_index] * (M_PI/180);
  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians);

  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quaternion, qMsg);

  goal.target_pose.pose.orientation = qMsg;

  ROS_INFO("Sending robot to: x = %f, y = %f, theta = %f",
               x[point_index], y[point_index], theta[point_index]);
  return goal;
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const MoveBaseResultConstPtr& result){
  ROS_INFO("You have reached the goal!");
}

void activeCb(){
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const MoveBaseFeedbackConstPtr& feedback){
  // Position
  float px = feedback->base_position.pose.position.x;
  float py = feedback->base_position.pose.position.y;

  // Quaternion
  float w = feedback->base_position.pose.orientation.w;
  float x = feedback->base_position.pose.orientation.x;
  float y = feedback->base_position.pose.orientation.y;
  float z = feedback->base_position.pose.orientation.z;
  float yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  yaw = (yaw/3.14)*180;

  ROS_INFO("Current robot pose: x = %f, y = %f, theta = %f", 
              px, py, yaw);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goals_node");

    // Get the goal's x, y and angle from the launch file
    double x, y, theta;
    
    ros::NodeHandle nh;
    nh.getParam("goal_xa", goal_xa);
    nh.getParam("goal_ya", goal_ya);
    nh.getParam("goal_thetaa", goal_thetaa);

    // create the action client 
    MoveBaseClient ac("move_base", true);

    // Wait 60 seconds for the action server to become available
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(60));
    ROS_INFO("Connected to move base server");

    // Keep read point coordinate, angle
    while(keeprun){

      // Send a goal to move_base
      goal = setGoal(goal_xa, goal_ya, goal_thetaa);
      ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

      // Wait for the action to return
      ac.waitForResult();

      // Check "Should create new Goal?"
      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        if(point_index < goal_xa.size()-1){
          keeprun = true;
          point_index++;
        }
        else
          keeprun = false;
      }
      else
        ROS_INFO("The base failed for some reason");
    }

    return 0;
}