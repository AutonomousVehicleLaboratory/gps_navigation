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
  Node* point1;
  Node* point2;
  // Undefined
  //point1->osm_id = 698283636;
  //point1->lat = 32.8809844;
  //point1->lon = -117.2347614;
  //Node* point2 = new Node;
  //point2->osm_id = 469084101;
  //point2->lat = 32.8813332;
  //point2->lon = -117.2341074;
  // Defined
  //point1->osm_id = 7180450222;
  //point1->lat = 32.8814670;
  //point1->lon = -117.2293529;
  //point2->osm_id = 1176652298;
  //point2->lat = 32.8849806;
  //point2->lon = -117.2300328;
  long long point1_id = 7180450222;
  long long point2_id = 1176652298;
  point1 = osm_map.nodes_.find(point1_id)->second;
  point2 = osm_map.nodes_.find(point2_id)->second;
  std::cout << "1: graph_id: " << point1->graph_id << std::endl;
  std::cout << "1: osm_id: " << point1->osm_id << std::endl;
  std::cout << "1: lat: " << point1->lat<< std::endl;
  std::cout << "1: lon: " << point1->lon<< std::endl;
  std::cout << "2: graph_id: " << point2->graph_id << std::endl;
  std::cout << "2: osm_id: " << point2->osm_id << std::endl;
  std::cout << "2: lat: " << point2->lat<< std::endl;
  std::cout << "2: lon: " << point2->lon<< std::endl;

  std::vector<Node*> plan = osm_map.ShortestPath(point1, point2);  
  ros::spin();
  
  return 0;
}

