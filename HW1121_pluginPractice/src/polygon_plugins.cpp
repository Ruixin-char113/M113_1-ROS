#include <pluginlib/class_list_macros.h>
#include </home/rui/catkin_ws/src/plugin_3111/include/plugin_3111/polygon_base.h>
#include </home/rui/catkin_ws/src/plugin_3111/include/plugin_3111/polygon_plugins.h>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::PolygonHw, polygon_base::RegularPolygon)
// PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
// PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)