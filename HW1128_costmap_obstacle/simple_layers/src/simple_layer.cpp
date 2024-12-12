#include "/home/rui/catkin_ws/src/simple_layers/include/simple_layers/simple_layer.h"
#include <pluginlib/class_list_macros.h>
 
PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)
 
using costmap_2d::LETHAL_OBSTACLE;
 
namespace simple_layer_namespace
{
 
    SimpleLayer::SimpleLayer() {}
    
    void SimpleLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        current_ = true;
        
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &SimpleLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }
    
    void SimpleLayer::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                            double* min_y, double* max_x, double* max_y)
    {
        if (!enabled_)
            return;
        
        // mark_x_ = origin_x + cos(origin_yaw);
        // mark_y_ = origin_y + sin(origin_yaw);

        mark_x_ = 1.5;
        mark_y_ = 1.5;

        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
    }
    
    void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_)
            return;
        // unsigned int mx;
        // unsigned int my;
        // if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
        //     master_grid.setCost(mx, my, LETHAL_OBSTACLE);
        // }
        unsigned int mx1;
        unsigned int my1;
        if(master_grid.worldToMap(-0.5, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.4, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.6, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.3, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.7, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.2, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.8, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.1, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }
        if(master_grid.worldToMap(-0.9, 0, mx1, my1)){
            master_grid.setCost(mx1, my1, LETHAL_OBSTACLE);
        }

    }
 
} // end namespace