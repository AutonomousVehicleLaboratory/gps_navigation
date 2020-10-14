#include "ros/ros.h"
#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/graph.h>
#include <vector>
using namespace gps_navigation;
int main(int argc, char **argv){
  ros::init(argc, argv, "gps_navigation");
  ros::NodeHandle n;
  std::string osm_path = "/home/dfpazr/Documents/CogRob/avl/planning/gps_planner_nv/src/osm_planner/osm_example/ucsd-small.osm";
  gps_navigation::Map osm_map(osm_path);
  Node* point1 = new Node;
  point1->osm_id = 698283636;
  point1->lat = 32.8809844;
  point1->lon = -117.2347614;
  Node* point2 = new Node;
  point2->osm_id = 469084101;
  point2->lat = 32.8813332;
  point2->lon = -117.2341074;
  std::vector<Node*> plan = osm_map.ShortestPath(point1, point2);  
  ros::spin();
  
  return 0;
}

