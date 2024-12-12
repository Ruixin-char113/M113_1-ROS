#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "/home/rui/catkin_ws/devel/include/ex8_1_poly_vertice/poly_verticeAction.h"
#include <cmath>

typedef actionlib::SimpleActionServer<ex8_1_poly_vertice::poly_verticeAction> Server;

// 收到action的goal後呼叫的回呼函數
void execute(const ex8_1_poly_vertice::poly_verticeGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    ex8_1_poly_vertice::poly_verticeFeedback feedback;
    ROS_INFO("Poly vertice calculater is working x: %.2f y: %.2f, theta: %.2f, length: %.2f.", 
                goal->x, goal->y, goal->theta, goal->length);
    
    float x = goal->x;
    float y = goal->y;
    float theta = goal->theta;
    float length = goal->length;
    float currentTheta = theta;

    for(int vertexNumber=1; vertexNumber<=7; vertexNumber++){
        if(vertexNumber != 1){
            x += length * cos((currentTheta * M_PI) / 180);
            y += length * sin((currentTheta * M_PI) / 180);
            currentTheta += 60;
        }

        feedback.return_x = x;
        feedback.return_y = y;

        as->publishFeedback(feedback); 
        r.sleep();
    }

    // 當action完成後,向客戶端返回結果
    ROS_INFO ("Poly vertice calculater finish working."); 
    as->setSucceeded();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "poly_vertice_server");
    ros::NodeHandle n;

    // 定義一個伺服器
    Server server (n, "poly_vertice", boost::bind(&execute, _1, &server), false);

    // 伺服器開始運行 
    server.start();
    ros::spin(); 
    return 0;
}