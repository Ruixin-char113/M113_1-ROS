#include <actionlib/client/simple_action_client.h>
#include "/home/rui/catkin_ws/devel/include/ex8_1_poly_vertice/poly_verticeAction.h"

typedef actionlib::SimpleActionClient<ex8_1_poly_vertice::poly_verticeAction> Client;

// 當action完成後會呼叫次回呼函數一次
void doneCb (const actionlib::SimpleClientGoalState& state,
                const ex8_1_poly_vertice::poly_verticeResultConstPtr& result)
{
    // ROS_INFO("Yay! The dishes are now clean");
    ROS_INFO("Show all vertices.");
    ROS_INFO("Student id: 11363111");
    ros::shutdown();
}

// 當action啟動後會呼叫次回呼函數一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback後呼叫的回呼函數
void feedbackCb(const ex8_1_poly_vertice::poly_verticeFeedbackConstPtr& feedback)
{
    // ROS_INFO(" percent_complete: %f ", feedback->percent_complete);
    ROS_INFO("x: %.2f, y: %.2f", feedback->return_x, feedback->return_y);
}

bool isNumber(char c[]){
    int i = 0;
    bool dotFlag = false;
    
    if(c[i] == '-')
        i = 1;
    // 0 -> '\0'
    for(; c[i] != 0; i++){
        if(!isdigit(c[i])){
            // handle '.'
            if(c[i] == '.' && dotFlag == false){
                dotFlag = true;
                continue;
            }
            else{
                return false;
            }
        }
    }
    return true;
}

void checkInputdata(const int argc, char** argv){
    try{
        // Check number of parameters
        if(argc != 5){
            throw 10;
        }

        // Check parameters is number
        if(!isNumber(argv[1]) || !isNumber(argv[2]) || !isNumber(argv[3]) || !isNumber(argv[4])){
            throw 20;
        }

        // Check Length
        if(std::stof(argv[4]) <= 0){
            throw 30;
        }
    }
    catch(int exNumber){
        switch(exNumber){
            case 10:
                ROS_ERROR("Please check parameters, only allow \"All float: x, y, theta, length\".");
                break;
            case 20:
                ROS_ERROR("Please check parameters, only allow number.");
                break;
            case 30:
                ROS_ERROR("Length have to large than 0.");
                break;
        }
        ros::shutdown();
    }
}

ex8_1_poly_vertice::poly_verticeGoal setGoal(const float x, const float y,
                                                 const float theta, const float length){

    ex8_1_poly_vertice::poly_verticeGoal goal;

    goal.x      = x;
    goal.y      = y;
    goal.theta  = theta;
    goal.length = length;

    return goal;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "poly_vertice_client");

    // 定義一個客戶端
    Client client("poly_vertice", true);

    // Check parameters
    checkInputdata(argc, argv);

    // 等待伺服器端
    ROS_INFO("Waiting for action server to start."); 
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // 建立一個action的goal 
    ex8_1_poly_vertice::poly_verticeGoal goal; 
    goal = setGoal(std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3]), std::stof(argv[4]));
    // goal.dishwasher_id = 1;

    // 傳送action的goal給伺服器端,並且設定回呼函數
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
    ros::spin();
    return 0;
}