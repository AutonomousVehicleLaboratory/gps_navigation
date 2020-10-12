#include "ros/ros.h"
#include <gps_navigation/gps_navigation.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "gps_navigation");
  ros::NodeHandle n;
  std::string osm_path = "/home/dfpazr/Documents/CogRob/avl/planning/gps_planner_nv/src/osm_planner/osm_example/ucsd-small.osm";
  gps_navigation::Map osm_map(osm_path);
  
  ros::spin();
  
  return 0;
}
