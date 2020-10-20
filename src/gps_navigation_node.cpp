#include "ros/ros.h"
#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/graph.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
using namespace gps_navigation;
double get_x(Node* point1, Node* point2){
  double DEG2RAD = M_PI / 180;
  double R = 6371e3;
  double dLon = point2->lon* DEG2RAD - point1->lon* DEG2RAD;
  double latAverage = (point1->lat+ point2->lat) / 2;
  double a = cos(latAverage * DEG2RAD) * cos(latAverage * DEG2RAD) *
             sin(dLon / 2) * sin(dLon / 2);
  double dist = R * 2 * atan2(sqrt(a), sqrt(1 - a));

  return point1->lon < point2->lon ? dist : -dist;
}
double get_y(Node* point1, Node* point2){
  double DEG2RAD = M_PI / 180;
  static double R = 6371e3;
  double dLat = point2->lat * DEG2RAD - point1->lat* DEG2RAD;
  double a = sin(dLat / 2) * sin(dLat / 2);
  double dist = R * 2 * atan2(sqrt(a), sqrt(1 - a));

  return point1->lat < point2->lat ? dist : -dist;
}
visualization_msgs::Marker visualize_path(std::vector<Node*> path){
  visualization_msgs::Marker path_viz;
  path_viz.header.frame_id = "/map";
  path_viz.header.stamp = ros::Time::now();
  path_viz.ns = "points_and_lines";
  path_viz.id = 0;
  path_viz.type = visualization_msgs::Marker::POINTS;
  path_viz.action = visualization_msgs::Marker::ADD;
  path_viz.color.g = 1.0f;
  path_viz.color.a = 1.0;
  path_viz.scale.x = 6.2;
  path_viz.scale.y = 6.2;
   
  Node* ref_start = new Node;
  ref_start->lat = 32.88465;
  ref_start->lon = -117.24244;
  
  //ros::Time current_time = ros::Time::now(); 
  for (unsigned int i=0; i<path.size(); i++){
    geometry_msgs::Point current_pose;
    current_pose.x = get_x(ref_start, path[i]);
    current_pose.y = get_y(ref_start, path[i]);
    current_pose.z = 0.0;
    //current_pose.pose.orientation.x = 1.0;
    //current_pose.pose.orientation.y = 1.0;
    //current_pose.pose.orientation.z = 1.0;
    //current_pose.pose.orientation.w = 1.0;
    
    path_viz.points.push_back(current_pose);
     
  }
  //road_network.header.stamp = current_time;
  return path_viz; 
}
/*
nav_msgs::Path visualize_path(std::vector<Node*> path){
  nav_msgs::Path road_network;
  road_network.header.frame_id = "map";
  geometry_msgs::PoseStamped current_pose;
  
  Node* ref_start = new Node;
  ref_start->lat = 32.88465;
  ref_start->lon = -117.24244;
  
  road_network.poses.clear();
  //ros::Time current_time = ros::Time::now(); 
  for (unsigned int i=0; i<10; i++){
    current_pose.pose.position.x = get_x(ref_start, path[i]);
    current_pose.pose.position.y = get_y(ref_start, path[i]);
    current_pose.pose.position.z = 0.0;
    //current_pose.pose.orientation.x = 1.0;
    //current_pose.pose.orientation.y = 1.0;
    //current_pose.pose.orientation.z = 1.0;
    //current_pose.pose.orientation.w = 1.0;
    current_pose.header.seq = i;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = current_time;
    road_network.poses.push_back(current_pose);
     
  }
  //road_network.header.stamp = current_time;
  return road_network; 
}
*/
nav_msgs::Path visualize_network(std::vector<Way*> ways, ros::Publisher way_pub){
  nav_msgs::Path road_network;
  road_network.header.frame_id = "map";
  geometry_msgs::PoseStamped current_pose;
  
  Node* ref_start = new Node;
  ref_start->lat = 32.88465;
  ref_start->lon = -117.24244;
  
  int counter = 0;
  ros::Time current_time = ros::Time::now();
  for (unsigned int i=0; i<ways.size(); i++){ 
    road_network.poses.clear();
    current_time = ros::Time::now();
    for (unsigned int j=0; j<ways[i]->nodes.size(); j++){
      current_pose.pose.position.x = get_x(ref_start, ways[i]->nodes[j]);
      current_pose.pose.position.y = get_y(ref_start, ways[i]->nodes[j]);
      current_pose.pose.position.z = 0.0;
      //current_pose.pose.orientation.x = 1.0;
      //current_pose.pose.orientation.y = 1.0;
      //current_pose.pose.orientation.z = 1.0;
      //current_pose.pose.orientation.w = 1.0;
      current_pose.header.seq = counter;
      current_pose.header.frame_id = "map";
      current_pose.header.stamp = current_time;
      road_network.poses.push_back(current_pose);
      ++counter; 
    }
    way_pub.publish(road_network);
  }
  //road_network.header.stamp = current_time;
  return road_network; 
}
int main(int argc, char **argv){
  ros::init(argc, argv, "gps_navigation");
  ros::NodeHandle n;
  //ros::Publisher shortest_path = n.advertise<nav_msgs::Path>("/road_network", 1000);
  ros::Publisher shortest_path_viz = n.advertise<visualization_msgs::Marker>("/shortest_path", 1000);
  ros::Publisher road_network_viz = n.advertise<nav_msgs::Path>("/road_network", 1000);
  std::string osm_path = "/home/dfpazr/Documents/CogRob/avl/planning/gps_planner_nv/src/osm_planner/osm_example/ucsd-small.osm";
  gps_navigation::Map osm_map(osm_path);
  Node* point1;
  Node* point2;
  ros::Rate r(30);
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
  //long long point1_id = 1176652302;
  //long long point2_id = 1176652298;
  long long point2_id = 1176652247;
  //point1 = osm_map.nodes_.find(point1_id)->second;
  //point2 = osm_map.nodes_.find(point2_id)->second;

  double lat1 = 32.88465; 
  double lon1 = -117.24244;
  double lat2 = 32.88184;
  double lon2 = -117.23484; 
  Node* point1_shortest = osm_map.FindClosestNode(lat1, lon1); 
  Node* point2_shortest = osm_map.FindClosestNode(lat2, lon2); 
  
  std::vector<Node*> plan = osm_map.ShortestPath(point1_shortest, point2_shortest);  
  //nav_msgs::Path road_networks = visualize_path(plan);
  while(ros::ok()){
    visualization_msgs::Marker shortest_path = visualize_path(plan);
    nav_msgs::Path road_networks = visualize_network(osm_map.ways_, road_network_viz);
    shortest_path_viz.publish(shortest_path);  
    //road_network_viz.publish(road_networks);  
    std::cout << "published path" << std::endl; 

  }
  //ros::spin();
  
  return 0;
}

