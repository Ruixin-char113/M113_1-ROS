#include <pluginlib/class_loader.h>
#include </home/rui/catkin_ws/src/plugin_3111/include/plugin_3111/polygon_base.h>
#include <string>

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

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("plugin_3111", "polygon_base::RegularPolygon");
  
try
  {
    // "Please check parameter: Only allow "side_length, side_number""
    if(argc != 3)
        throw 0;

    // "Please check parameter: Only allow "side_length: Possitive number, side_number: Positive Integer""
    if(!isNumber(argv[1]) || !isNumber(argv[2]))
        throw 10;

    double  sideLength = std::stod(argv[1]);
    int     sideNumber = std::stoi(argv[2]);

    // "Please check parameter: Only allow "side_length: Possitive number, side_number: Positive Integer""
    if(sideLength <= 0 || sideNumber <= 0)
        throw 10;
    
    // "Please check parameter: "side_number must >= 3""
    if(sideNumber < 3)
        throw 20;
    
    ROS_INFO("side_length: %.2f, side_number: %d", sideLength, sideNumber);

    boost::shared_ptr<polygon_base::RegularPolygon> polygon = poly_loader.createInstance("polygon_plugins::PolygonHw");
    polygon->initialize(sideLength);
    for(int i=3; i<= sideNumber; i++){
        ROS_INFO("[%d] sides polygon's area: %.2f", i, polygon->area(i));
    }
    ROS_INFO("my id: 11363111");
    // boost::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createInstance("polygon_plugins::Triangle");
    // triangle->initialize(10.0);
    // boost::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createInstance("polygon_plugins::Square");
    // square->initialize(10.0);
    // ROS_INFO("Triangle area: %.2f", triangle->area());
    // ROS_INFO("Square area: %.2f", square->area());
  }

  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  catch(int numException){
    if(numException == 0)
        ROS_ERROR("Please check parameter: Only allow \"side_length, side_number\"");
    else if(numException == 10){
        ROS_ERROR("Please check parameter: Only allow \"side_length: Possitive number, side_number: Positive Integer\"");
    }
    else if(numException == 20){
        ROS_ERROR("Please check parameter: \"side_number must >= 3\"");
    }
  }

  return 0;
}