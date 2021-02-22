/**
 * @file gps_navigation_node.cpp
 *
 * @brief ROS node for gps based path planning/local bev generation. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#include <gps_navigation/gps_navigation_node.h>

namespace gps_navigation{
  GpsNavigationNode::GpsNavigationNode(){
    shortest_path_viz = n.advertise<nav_msgs::Path>("/shortest_path", 1000);
    road_network_viz = n.advertise<nav_msgs::Path>("/road_network", 1000);
    node_orientation_viz = n.advertise<visualization_msgs::Marker>("/node_orientations", 1000);
    gps_viz_pub = n.advertise<visualization_msgs::Marker>("/gps_pose", 1000);
    gps_closest_viz_pub = n.advertise<visualization_msgs::Marker>("/gps_oriented_pose", 1000);
    unrouted_bev_pub = n.advertise<sensor_msgs::Image>("/unrouted_osm", 1000);
    routed_bev_pub = n.advertise<sensor_msgs::Image>("/routed_osm", 1000);
    conc_bev_pub = n.advertise<sensor_msgs::Image>("/concat_osm", 1000);
    gps_pose_sub = n.subscribe("/lat_lon", 1000, &GpsNavigationNode::GpsCallback, this);
    imu_sub = n.subscribe("/livox/imu", 1000, &GpsNavigationNode::ImuCallback, this);
    speed_sub = n.subscribe("/pacmod/as_tx/vehicle_speed", 1000, &GpsNavigationNode::SpeedCallback, this);
    clicked_point = n.subscribe("/move_base_simple/goal", 1000, &GpsNavigationNode::ClickedPointCallback, this);
    
    // Initialize map
    std::string osm_path = "/home/avl-robot/DPGN/dpgn_models/osm/ucsd.osm";
    //osm_map = new Map(osm_path);
    gps_navigator = new Navigation(osm_path, kOsmOriginX, kOsmOriginY);
    //osm_bev = new GpsBev(osm_map->ways_, kOsmOriginX, kOsmOriginY, 0.5, 2, 200);
    osm_bev = new GpsBev(gps_navigator->GetMap()->GetWays(), kOsmOriginX, kOsmOriginY, 0.5, 8, 200);
  
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
    twist.angular_velocity = msg->angular_velocity;
    twist.linear_acceleration = msg->linear_acceleration;
    twist.header = msg->header;
    if(!imu_avail || !gps_avail){
      imu_avail = true;
      return; 
    }
    /*
    if(gps_avail && imu_avail){
      ros::Duration dt = ros::Time::now() - twist.header.stamp;
      cv::Mat local_osm_bev = osm_bev->RetrieveLocalBev(lat_pose, lon_pose,
                                                       ego_speed,
                                                       twist.linear_acceleration.x,
                                                       twist.linear_acceleration.y,
                                                       twist.angular_velocity.z,
                                                       dt.toSec(), plan, 200);
      sensor_msgs::ImagePtr local_osm_bev_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", local_osm_bev).toImageMsg();
      gps_bev_pub.publish(local_osm_bev_msg);
    }*/
    if(gps_avail && has_clicked_point){
      // Closest node wrt position of ego vehicle
      gps_navigator->SetStart(lat_pose, lon_pose);
      // Destination point manually set
      gps_navigator->SetTargetRelative(x_dest, y_dest);
      plan = gps_navigator->Plan();
      new_plan = true;
      has_clicked_point = false;
      
      new_gps_msg = false;
      gps_navigator->ResetPlan();
    }

    if(gps_avail && plan.size()){
      ego_state = gps_navigator->UpdateState(lat_pose, lon_pose,  ego_speed, 
                                                                 twist.angular_velocity.z, twist.linear_acceleration.x, ros::Time::now().toSec());

    }
    
    if(new_plan){
      nav_msgs::Path shortest_path = VisualizePath(plan);
      shortest_path_viz.publish(shortest_path);
    } 
    return; 
  }
  void GpsNavigationNode::SpeedCallback(const std_msgs::Float64::ConstPtr& msg){
    ego_speed = msg->data;
  }
  
  void GpsNavigationNode::PublishGpsMap(){
    if(gps_avail && plan.size()){

      if(std::get<0>(ego_state)){
        // Publish oriented ego vehicle
        visualization_msgs::Marker oriented_ego; 
        oriented_ego.action = visualization_msgs::Marker::ADD; 
        oriented_ego = GetMarker(1, gps_ego_counter++, ros::Time::now(), 
                                                           std::get<2>(ego_state), std::get<3>(ego_state), 0.0, std::get<4>(ego_state));
        gps_closest_viz_pub.publish(oriented_ego);
        // Get BEV image and publish too
        std::pair<cv::Mat, cv::Mat> osm_bevs = osm_bev->RetrieveLocalBev(std::get<2>(ego_state), std::get<3>(ego_state), std::get<4>(ego_state),
                                                          std::get<1>(ego_state), plan, 200);
        
        if(!osm_bevs.first.empty()){
          cv::Mat conc_bevs;
          cv::vconcat(osm_bevs.first, osm_bevs.second, conc_bevs);
          sensor_msgs::ImagePtr conc_osm_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", conc_bevs).toImageMsg();
          conc_bev_pub.publish(conc_osm_msg);
        }
        sensor_msgs::ImagePtr unrouted_osm_bev_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", osm_bevs.first).toImageMsg();
        sensor_msgs::ImagePtr routed_osm_bev_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", osm_bevs.second).toImageMsg();

        unrouted_bev_pub.publish(unrouted_osm_bev_msg);
        routed_bev_pub.publish(routed_osm_bev_msg);
        
      }
    }
  }
  visualization_msgs::Marker GpsNavigationNode::GetMarker(int marker_type, long id, ros::Time ts, double x, double y, double z, double yaw){
    visualization_msgs::Marker marker;
    marker.id = id;
    marker.header.frame_id = "map";
    marker.header.seq = id;
    marker.header.stamp = ts;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Quaternion node_q;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    if(marker_type == 0){
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.type = visualization_msgs::Marker::SPHERE;
    }
    else if(marker_type == 1){
      marker.scale.x = 1.0;
      marker.scale.y = 0.5;
      marker.scale.z = 0.5;
      marker.color.a = 1.0;
      marker.color.r = 0.74;
      marker.color.g = 0.2;
      marker.color.b = 0.92;
      node_q.setRPY(0.0, 0.0, yaw);  
      marker.pose.orientation.x = node_q[0];
      marker.pose.orientation.y = node_q[1];
      marker.pose.orientation.z = node_q[2];
      marker.pose.orientation.w = node_q[3];
      marker.type = visualization_msgs::Marker::ARROW;
      
    }
    return marker;
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
    //visualization_msgs::MarkerArray node_directions;
    visualization_msgs::Marker current_direction;
    current_direction.action = visualization_msgs::Marker::ADD;
    

    road_network.header.frame_id = "map";
    geometry_msgs::PoseStamped current_pose;
    
    Node* ref_start = new Node;
    ref_start->lat = kOsmOriginX;
    ref_start->lon = kOsmOriginY;
    
    int counter = 0;
    ros::Time current_time = ros::Time::now();
    for (unsigned int i=0; i<gps_navigator->GetMap()->GetWays().size(); i++){ 
      road_network.poses.clear();
      current_time = ros::Time::now();
      for (unsigned int j=0; j<gps_navigator->GetMap()->GetWays()[i]->nodes.size(); j++){
        std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, gps_navigator->GetMap()->GetWays()[i]->nodes[j]);
        // Accumulate orientation markers
        // Estimate quaternion representation
        if((gps_navigator->GetMap()->GetWays()[i]->nodes[j]->dx_dy.first != 0) && 
           (gps_navigator->GetMap()->GetWays()[i]->nodes[j]->dx_dy.second != 0) &&
           (gps_navigator->GetMap()->GetWays()[i]->one_way)){
          double yaw = atan2(gps_navigator->GetMap()->GetWays()[i]->nodes[j]->dx_dy.second, 
                             gps_navigator->GetMap()->GetWays()[i]->nodes[j]->dx_dy.first);
          current_direction = GetMarker(1, counter, current_time, dx_dy.first, dx_dy.second, 0.0, yaw);
        }
        else{
          current_direction = GetMarker(0, counter, current_time, dx_dy.first, dx_dy.second, 0.0, 0);
        }
        //node_directions.markers.push_back(current_direction);
        
        // Create road network segment
        current_pose.pose.position.x = dx_dy.first;
        current_pose.pose.position.y = dx_dy.second;
        current_pose.pose.position.z = 0.0;
        current_pose.header.seq = counter;
        current_pose.header.frame_id = "map";
        current_pose.header.stamp = current_time;
        road_network.poses.push_back(current_pose);

        node_orientation_viz.publish(current_direction);
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
    gps_nav_node.PublishGpsMap();
    ros::spinOnce(); 

  }

  return 0;
}
