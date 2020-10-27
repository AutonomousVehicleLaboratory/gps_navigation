#include <gps_navigation/gps_navigation_node.h>

namespace gps_navigation{
  GpsNavigationNode::GpsNavigationNode(){
    shortest_path_viz = n.advertise<nav_msgs::Path>("/shortest_path", 1000);
    road_network_viz = n.advertise<nav_msgs::Path>("/road_network", 1000);
    gps_viz_pub = n.advertise<visualization_msgs::Marker>("/gps_pose", 1000);
    gps_bev_pub = n.advertise<sensor_msgs::Image>("/osm_bev", 1000);
    gps_pose_sub = n.subscribe("/lat_lon", 1000, &GpsNavigationNode::GpsCallback, this);
    imu_sub = n.subscribe("/livox/imu", 1000, &GpsNavigationNode::ImuCallback, this);
    clicked_point = n.subscribe("/move_base_simple/goal", 1000, &GpsNavigationNode::ClickedPointCallback, this);
    
    // Initialize map
    std::string osm_path = "/home/dfpazr/Documents/CogRob/avl/planning/gps_planner_nv/src/osm_planner/osm_example/ucsd-large.osm";
    osm_map = new Map(osm_path);
    osm_bev = new GpsBev(osm_map->ways_, kOsmOriginX, kOsmOriginY, 0.5, 2);
  
  }
  void GpsNavigationNode::ClickedPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    x_dest = msg->pose.position.x; 
    y_dest = msg->pose.position.y;
    has_clicked_point = true; 
  }
  void GpsNavigationNode::GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    gps_avail = true;
    
    new_gps_msg = true;
    lat_pose = msg->latitude;
    lon_pose = msg->longitude;
  
    ref_start->lat = kOsmOriginX;
    ref_start->lon = kOsmOriginY;
      
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
  
  void GpsNavigationNode::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    //road_networks = VisualizeNetwork();
    if(!imu_avail || !gps_avail){
      twist.angular_velocity = msg->angular_velocity;
      twist.linear_acceleration = msg->linear_acceleration;
      twist.header = msg->header;
      imu_avail = true;
      return; 
    }
    //if(gps_avail && imu_avail){
    //  ros::Duration dt = ros::Time::now() - twist.header.stamp;
    //  cv::Mat local_osm_bev = osm_bev->RetrieveLocalBev(lat_pose, lon_pose,
    //                                                   twist.linear_acceleration.x,
    //                                                   twist.linear_acceleration.y,
    //                                                   twist.angular_velocity.z,
    //                                                   dt.toSec(), plan, 200);
    //  sensor_msgs::ImagePtr local_osm_bev_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", local_osm_bev).toImageMsg();
    //  gps_bev_pub.publish(local_osm_bev_msg);
    //}
    if(gps_avail && has_clicked_point){
      point1_shortest = osm_map->FindClosestNode(lat_pose, lon_pose);
      point2_shortest = osm_map->FindClosestNodeRelative(x_dest, y_dest, kOsmOriginX, kOsmOriginY);
      plan = osm_map->ShortestPath(point1_shortest, point2_shortest);
      new_plan = true;
      has_clicked_point = false;
      
      //new_gps_msg = false;
      osm_map->osm_graph.ResetGraph(osm_map->navigation_nodes_);
    }
    
    if(new_plan){
      nav_msgs::Path shortest_path = VisualizePath(plan);
      shortest_path_viz.publish(shortest_path);
    } 
    return; 
  }
  visualization_msgs::Marker GpsNavigationNode::visualize_waypoints(std::vector<Node*> path){
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
     
    Node ref_start;
    ref_start.lat = kOsmOriginX;
    ref_start.lon = kOsmOriginY;
    
    //ros::Time current_time = ros::Time::now(); 
    for (unsigned int i=0; i<path.size(); i++){
      geometry_msgs::Point current_pose;
      std::pair<double, double> dx_dy = RelativeDisplacement(&ref_start, path[i]);
      current_pose.x = dx_dy.first;
      current_pose.y = dx_dy.second;
      current_pose.z = 0.0;
      
      path_viz.points.push_back(current_pose);
       
    }
    //road_network.header.stamp = current_time;
    return path_viz; 
  }
  
  nav_msgs::Path GpsNavigationNode::VisualizePath(std::vector<Node*> path){
    nav_msgs::Path road_network;
    road_network.header.frame_id = "map";
    geometry_msgs::PoseStamped current_pose;
    
    Node ref_start;
    ref_start.lat = kOsmOriginX;
    ref_start.lon = kOsmOriginY;
    
    road_network.poses.clear();
    ros::Time current_time = ros::Time::now(); 
    for (unsigned int i=0; i<path.size(); i++){
      std::pair<double, double> dx_dy = RelativeDisplacement(&ref_start, path[i]);
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
  
  nav_msgs::Path GpsNavigationNode::VisualizeNetwork(){
    nav_msgs::Path road_network;
    road_network.header.frame_id = "map";
    geometry_msgs::PoseStamped current_pose;
    
    Node* ref_start = new Node;
    ref_start->lat = kOsmOriginX;
    ref_start->lon = kOsmOriginY;
    
    int counter = 0;
    ros::Time current_time = ros::Time::now();
    for (unsigned int i=0; i<osm_map->ways_.size(); i++){ 
      road_network.poses.clear();
      current_time = ros::Time::now();
      for (unsigned int j=0; j<osm_map->ways_[i]->nodes.size(); j++){
        std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, osm_map->ways_[i]->nodes[j]);
        current_pose.pose.position.x = dx_dy.first;
        current_pose.pose.position.y = dx_dy.second;
        current_pose.pose.position.z = 0.0;
        current_pose.header.seq = counter;
        current_pose.header.frame_id = "map";
        current_pose.header.stamp = current_time;
        road_network.poses.push_back(current_pose);
        ++counter; 
      }
      road_network_viz.publish(road_network);
    }
    //road_network.header.stamp = current_time;
    return road_network; 
  }
}
int main(int argc, char **argv){
  ros::init(argc, argv, "gps_navigation");
  gps_navigation::GpsNavigationNode gps_nav_node;
  while(ros::ok()){
    static_cast<void>(gps_nav_node.VisualizeNetwork());
    ros::spinOnce(); 

  }

  return 0;
}
