#include "ros/ros.h"
#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/graph.h>
#include <gps_navigation/utils.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>

using namespace gps_navigation;
ros::Publisher gps_viz_pub;

void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  Node* ref_start = new Node;
  ref_start->lat = 32.88465;
  ref_start->lon = -117.24244;
    
  Node* gps_pose = new Node;
  gps_pose->lat = msg->latitude;
  gps_pose->lon = msg->longitude;
  
  visualization_msgs::Marker gps_viz;
  gps_viz.header.frame_id = "/map";
  gps_viz.header.stamp = ros::Time::now();
  gps_viz.ns = "points_and_lines";
  gps_viz.id = 0;
  gps_viz.type = visualization_msgs::Marker::POINTS;
  gps_viz.action = visualization_msgs::Marker::ADD;
  gps_viz.color.b = 1.0;
  gps_viz.color.a = 1.0;
  gps_viz.scale.x = 9.2;
  gps_viz.scale.y = 9.2;
  
  geometry_msgs::Point gps_point;
  std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, gps_pose);
  gps_point.x = dx_dy.first;
  gps_point.y = dx_dy.second;
  gps_point.z = 0;

  gps_viz.points.push_back(gps_point);
  gps_viz_pub.publish(gps_viz);
}
visualization_msgs::Marker visualize_waypoints(std::vector<Node*> path){
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
  ref_start->lat = kOsmOriginX;
  ref_start->lon = kOsmOriginY;
  
  //ros::Time current_time = ros::Time::now(); 
  for (unsigned int i=0; i<path.size(); i++){
    geometry_msgs::Point current_pose;
    std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, path[i]);
    current_pose.x = dx_dy.first;
    current_pose.y = dx_dy.second;
    current_pose.z = 0.0;
    
    path_viz.points.push_back(current_pose);
     
  }
  //road_network.header.stamp = current_time;
  return path_viz; 
}

nav_msgs::Path visualize_path(std::vector<Node*> path){
  nav_msgs::Path road_network;
  road_network.header.frame_id = "map";
  geometry_msgs::PoseStamped current_pose;
  
  Node* ref_start = new Node;
  ref_start->lat = kOsmOriginX;
  ref_start->lon = kOsmOriginY;
  
  road_network.poses.clear();
  ros::Time current_time = ros::Time::now(); 
  for (unsigned int i=0; i<path.size(); i++){
    std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, path[i]);
    current_pose.pose.position.x = dx_dy.first;
    current_pose.pose.position.y = dx_dy.second;
    current_pose.pose.position.z = 0.0;
    current_pose.header.seq = i;
    current_pose.header.frame_id = "map";
    current_pose.header.stamp = current_time;
    road_network.poses.push_back(current_pose);
     
  }
  //road_network.header.stamp = current_time;
  return road_network; 
}

nav_msgs::Path visualize_network(std::vector<Way*> ways, ros::Publisher way_pub){
  nav_msgs::Path road_network;
  road_network.header.frame_id = "map";
  geometry_msgs::PoseStamped current_pose;
  
  Node* ref_start = new Node;
  ref_start->lat = kOsmOriginX;
  ref_start->lon = kOsmOriginY;
  
  int counter = 0;
  ros::Time current_time = ros::Time::now();
  for (unsigned int i=0; i<ways.size(); i++){ 
    road_network.poses.clear();
    current_time = ros::Time::now();
    for (unsigned int j=0; j<ways[i]->nodes.size(); j++){
      std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, ways[i]->nodes[j]);
      current_pose.pose.position.x = dx_dy.first;
      current_pose.pose.position.y = dx_dy.second;
      current_pose.pose.position.z = 0.0;
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
  //ros::Publisher shortest_path_viz = n.advertise<visualization_msgs::Marker>("/shortest_path", 1000);
  ros::Publisher shortest_path_viz = n.advertise<nav_msgs::Path>("/shortest_path", 1000);
  ros::Publisher road_network_viz = n.advertise<nav_msgs::Path>("/road_network", 1000);
  gps_viz_pub = n.advertise<visualization_msgs::Marker>("/gps_pose", 1000);
  ros::Subscriber gps_pose = n.subscribe("/lat_lon", 1000, GpsCallback);
  std::string osm_path = "/home/dfpazr/Documents/CogRob/avl/planning/gps_planner_nv/src/osm_planner/osm_example/ucsd-small.osm";
  gps_navigation::Map osm_map(osm_path);
  Node* point1;
  Node* point2;
  ros::Rate r(30);
  

  double lat2 = 32.88465; 
  double lon2 = -117.24244;
  double lat1 = 32.88184;
  double lon1 = -117.23484; 
  Node* point1_shortest = osm_map.FindClosestNode(lat1, lon1); 
  Node* point2_shortest = osm_map.FindClosestNode(lat2, lon2); 
  
  std::vector<Node*> plan = osm_map.ShortestPath(point1_shortest, point2_shortest); 
  while(ros::ok()){
    //visualization_msgs::Marker shortest_path = visualize_path(plan);
    nav_msgs::Path shortest_path = visualize_path(plan);
    nav_msgs::Path road_networks = visualize_network(osm_map.ways_, road_network_viz);
    shortest_path_viz.publish(shortest_path);  
    ros::spinOnce();
  }
  
  return 0;
}

