#ifndef PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_
#define PLUGINLIB_TUTORIALS__POLYGON_PLUGINS_H_
#include </home/rui/catkin_ws/src/plugin_3111/include/plugin_3111/polygon_base.h>
#include <cmath>  //cmath標頭檔為特殊運算語法，
                //沒有寫的話無法執行pow(,)平方運算語法
namespace polygon_plugins{
    class PolygonHw : public polygon_base::RegularPolygon{
        public:
        PolygonHw(){}

        void initialize(double side_length){
            side_length_ = side_length;
        }

        double area(double side_number){
            double side_height_ = (side_length_ / 2) * tan(((side_number - 2) * 90) * M_PI 
                                    / (side_number * 180));
            return (side_length_ * side_height_ / 2) * side_number;
            // return side_number;
        }
        
        private:
        double side_length_;
    };

    // class Triangle : public polygon_base::RegularPolygon{
    //     public:
    //     Triangle(){}
    //     void initialize(double side_length){
    //         side_length_ = side_length;
    //     }

    //     double area(){
    //         return 0.5 * side_length_ * getHeight();
    //     }
        
    //     double getHeight(){
    //         return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
    //     }
        
    //     private:
    //     double side_length_;
    // };
    
    // class Square : public polygon_base::RegularPolygon{
    //     public:
    //     Square(){}
    //     void initialize(double side_length){
    //         side_length_ = side_length;
    //     }
        
    //     double area(){
    //         return side_length_ * side_length_;
    //     }
        
    //     private:
    //     double side_length_;
    // };
};
#endif
