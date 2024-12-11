#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "/home/rui/catkin_ws/devel/include/ex8_1_poly_vertice/poly_verticeAction.h"

typedef actionlib::SimpleActionServer<ex8_1_poly_vertice::poly_verticeAction> Server;

// 收到action的goal後呼叫的回呼函數
void execute(const ex8_1_poly_vertice::poly_verticeGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    ex8_1_poly_vertice::poly_verticeFeedback feedback;
    ROS_INFO("Dishwasher %d is working.", goal->dishwasher_id);
    // 假設洗盤子的進度,並且按照1hz的頻率發佈進度feedback

    for(int i=1; i<=10; i++)
    {
        feedback.percent_complete = i* 10;
        as->publishFeedback(feedback); 
        r.sleep();        
    }

    // 當action完成後,向客戶端返回結果
    ROS_INFO ("Dishwasher %d finish working.", goal->dishwasher_id); 
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