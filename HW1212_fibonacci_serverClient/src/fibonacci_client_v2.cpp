#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>
#include <string>

using namespace actionlib_tutorials;
typedef actionlib::SimpleActionClient<FibonacciAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const FibonacciResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Final term of the sequence: %i", result->sequence.back());
  // ROS_INFO("Final term of the sequence: %s", std::to_string(feedback->sequence[feedback->sequence.size()]).c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const FibonacciFeedbackConstPtr& feedback)
{
  std::string buffer;

  int feedbackSize = feedback->sequence.size();
  // ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
  ROS_INFO("Got Feedback of length %d", feedbackSize);

  for(int i = 0; i < feedbackSize; i++){
    buffer += std::to_string(feedback->sequence[i]);
    if(i + 1 != feedbackSize)
      buffer += ", ";
  }

  ROS_INFO(" Sequence : { %s }", buffer.c_str());
}

// =============================================================
//  Check Input Data
// =============================================================

bool isNaturalNumber(char c[]){
  if(c[0] == '0')
    return false;
  
  for(int i = 0; c[i] != 0; i++){
    if(!isdigit(c[i])){
      return false;
    }
  }
  return true;
}

void checkInputdata(const int argc, char** argv){
    try{
        // Check number of parameters
        if(argc != 2){
            throw 10;
        }

        // Check parameters is number
        if(!isNaturalNumber(argv[1])){
            throw 20;
        }
    }
    catch(int exNumber){
        switch(exNumber){
            case 10:
                ROS_ERROR("Please check parameter, only allow \"Number\".");
                break;
            case 20:
                ROS_ERROR("Please check parameter, only allow Natural Number.");
                break;
        }
        ros::shutdown();
    }
}

FibonacciGoal setGoal(const int order){
  FibonacciGoal goal;

  goal.order = order;

  return goal;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci_callback");

  // Create the action client
  Client ac("fibonacci", true);

  // Check parameter
  checkInputdata(argc, argv);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  // Send Goal
  FibonacciGoal goal;
  goal = setGoal(std::stoi(argv[1]));
  // goal.order = 20;

  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin();
  return 0;
}