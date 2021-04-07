/**
 * @file gps_navigation_node.cpp
 *
 * @brief ROS node for gps based path planning/local bev generation. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */
//test current code
#include <gps_navigation/gps_navigation_node.h>
#include <chrono>

namespace gps_navigation{
  GpsNavigationNode::GpsNavigationNode(){
    kd_nearest_viz_pub = n.advertise<visualization_msgs::Marker>("/kd_nearest", 1000);
    shortest_path_viz = n.advertise<nav_msgs::Path>("/shortest_path", 1000);
    road_network_viz = n.advertise<nav_msgs::Path>("/road_network", 1000);
    traversed_path_viz = n.advertise<nav_msgs::Path>("/traversed_path", 1000);
    planned_path_viz = n.advertise<nav_msgs::Path>("/planned_path", 100);
    footpaths_viz = n.advertise<nav_msgs::Path>("/foot_paths", 1000);
    node_orientation_viz = n.advertise<visualization_msgs::Marker>("/node_orientations", 1000);
    gps_viz_pub = n.advertise<visualization_msgs::Marker>("/gps_pose", 1000);
    gps_closest_viz_pub = n.advertise<visualization_msgs::Marker>("/gps_oriented_pose", 1000);
    unrouted_bev_pub = n.advertise<sensor_msgs::Image>("/unrouted_osm", 1000);
    routed_bev_pub = n.advertise<sensor_msgs::Image>("/routed_osm", 1000);
    conc_bev_pub = n.advertise<sensor_msgs::Image>("/concat_osm", 1000);

    // Graph Based Visualization
    g_stops_pub = n.advertise<visualization_msgs::MarkerArray>("/stop_signs", 10);
    g_crossings_pub = n.advertise<visualization_msgs::MarkerArray>("/crossings", 10);
    g_signals_pub = n.advertise<visualization_msgs::MarkerArray>("/traffic_signals", 10);
    g_constructions_pub = n.advertise<visualization_msgs::Marker>("/constructions", 10);


    gps_pose_sub = n.subscribe("/lat_lon", 1000, &GpsNavigationNode::GpsCallback, this);
    imu_sub = n.subscribe("/livox/imu", 1000, &GpsNavigationNode::ImuCallback, this);
    speed_sub = n.subscribe("/pacmod/as_tx/vehicle_speed", 1000, &GpsNavigationNode::SpeedCallback, this);
    clicked_point = n.subscribe("/move_base_simple/goal", 1000, &GpsNavigationNode::ClickedPointCallback, this);
    
    // Initialize map
    //std::string osm_path = "/mnt/avl_shared/user-files/dpaz/DPGN/ucsd.osm";
    std::string osm_path;
    double radius;
    if(n.getParam("/gps_navigation/osm_path", osm_path)){
      std::cout<<"The open street map file's path is set to be: "<<osm_path<<"\n";
    }
    if(n.getParam("/gps_navigation/radius", radius)){
      std::cout<<"The search radius for construction is set to be: "<<radius<<"\n";
    }
    if(n.getParam("/gps_navigation/replanThreshold", replanThreshold)){
      std::cout<<"The replan threshold is set to be: "<<replanThreshold<<"\n";
    }


    //osm_map = new Map(osm_path);
    gps_navigator = new Navigation(osm_path, kOsmOriginX, kOsmOriginY, radius);
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
    
    visualization_msgs::Marker kd_nearest_viz;
    kd_nearest_viz.header.frame_id = "/map";
    kd_nearest_viz.header.stamp = ros::Time::now();
    kd_nearest_viz.ns = "points_and_lines";
    kd_nearest_viz.id = 0;
    kd_nearest_viz.type = visualization_msgs::Marker::POINTS;
    kd_nearest_viz.action = visualization_msgs::Marker::ADD;
    kd_nearest_viz.color.b = 1.0;
    kd_nearest_viz.color.a = 1.0;
    kd_nearest_viz.scale.x = 9.2;
    kd_nearest_viz.scale.y = 9.2;
    //auto start = std::chrono::high_resolution_clock::now();
    Node* kd_nearest = gps_navigator->GetMap()->nn_graph_.KDNearest(gps_pose);
    //auto end = std::chrono::high_resolution_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //std::cout << "Time taken in microseconds: " << duration.count() << std::endl;
    dx_dy = RelativeDisplacement(ref_start, kd_nearest);
    
    gps_viz.points.push_back(gps_point);
    gps_point.x = dx_dy.first;
    gps_point.y = dx_dy.second;
    kd_nearest_viz.points.push_back(gps_point);
    kd_nearest_viz_pub.publish(kd_nearest_viz);
    
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
      std::cout<<"Planned New trajectory with plan size: "<<plan.size()<<std::endl;
      new_plan = true;
      has_clicked_point = false;
      
      new_gps_msg = false;
      gps_navigator->ResetPlan();
    }

    if(gps_avail && plan.size()){
      // TODO: 
      //ego_state = gps_navigator->UpdateState(lat_pose, lon_pose,  ego_speed, 
      //                                                           twist.angular_velocity.z, twist.linear_acceleration.x, ros::Time::now().toSec());
      double dist = INFINITY;
      auto start_node = new Node();
      start_node->lat = lat_pose;
      start_node->lon = lon_pose;
      for(auto node: plan){
        dist = std::min(dist, GreatCircleDistance(node, start_node));
      }
      delete start_node;
      if(dist>replanThreshold){
        has_clicked_point = true;
      }
      gps_navigator->GenerateSTGraph(lat_pose, lon_pose, ego_speed, ros::Time::now().toSec());
    }
    
    if(new_plan){
      nav_msgs::Path shortest_path = VisualizePath(plan);
      shortest_path_viz.publish(shortest_path);
    } 
    return; 
  }
  void GpsNavigationNode::SpeedCallback(const std_msgs::Float64::ConstPtr& msg){
    ego_speed = msg->data;

    if(gps_avail && plan.size()){
      gps_navigator->GenerateSTGraph(lat_pose, lon_pose, ego_speed, ros::Time::now().toSec());
    }
    
  }
  
  void GpsNavigationNode::PublishGpsMap(){
    if(gps_avail && plan.size()){

      // For graph generation method, visualize markers/paths
      std::vector<Node*> stops = gps_navigator->GetMap()->GetStops(); 
      std::vector<Node*> crossings = gps_navigator->GetMap()->GetCrossings(); 
      std::vector<Node*> signals = gps_navigator->GetMap()->GetTrafficSignals(); 
      std::vector<Way*> footpaths = gps_navigator->GetMap()->GetFootPaths(); 
      std::vector<Node*> traversed_path = gps_navigator->GetTraversedNodes();
      std::vector<Node*> planned_path = gps_navigator->GetPlannedNodes();
      std::vector<Node*> constructions = gps_navigator->GetMap()->GetConstrustions();
      //std::vector<Way*> footpaths = gps_navigator->GetMap()->footpaths_; 

      // Visualize stops, crossings and signals
      visualization_msgs::MarkerArray stop_markers = VisMarkersFromNodes(stops, 0, 1.0, 1.0, 0.0); 
      visualization_msgs::MarkerArray crossing_markers= VisMarkersFromNodes(crossings, 0, 0.0, 1.0, 1.0); 
      visualization_msgs::MarkerArray signal_markers= VisMarkersFromNodes(signals, 0, 1.0, 0.5, 0.0); 
      visualization_msgs::Marker construction_markers = VisMarkersFromNodes2(constructions, 0, 1.0, 0.5, 0.0);


      g_stops_pub.publish(stop_markers);
      g_crossings_pub.publish(crossing_markers);
      g_signals_pub.publish(signal_markers);
      g_constructions_pub.publish(construction_markers);

      // Visualize footpaths
      VisualizeFootPaths(footpaths);

      // Visualize traversed path
      nav_msgs::Path trav_path_msg = VisualizePath(traversed_path);
      traversed_path_viz.publish(trav_path_msg);

      // Visualize planned path
      nav_msgs::Path plan_path_msg = VisualizePath(planned_path);
      planned_path_viz.publish(plan_path_msg);

      // For OSM bev
      // TODO:
      /*
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
      */
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

  visualization_msgs::MarkerArray GpsNavigationNode::VisMarkersFromNodes(std::vector<Node*> nodes, int marker_type, float color_r, float color_g, float color_b){
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    
    int id = 0;
    ros::Time current_t = ros::Time::now();
    for(auto node: nodes){
      marker.id = id;
      marker.header.frame_id = "map";
      marker.header.seq = id;
      marker.header.stamp = current_t;
      marker.action = visualization_msgs::Marker::ADD;
      tf2::Quaternion node_q;
      double yaw = atan2(node->dx_dy.second, node->dx_dy.first);
      std::pair<double, double> x_y = RelativeDisplacement(ref_start, node);

      marker.pose.position.x = x_y.first;
      marker.pose.position.y = x_y.second;
      marker.pose.position.z = 0.0;
      if(marker_type == 0){
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = color_r;
        marker.color.g = color_g;
        marker.color.b = color_b;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
      }
      //else if(marker_type == 1){
      //  marker.scale.x = 1.0;
      //  marker.scale.y = 0.5;
      //  marker.scale.z = 0.5;
      //  marker.color.a = 1.0;
      //  marker.color.r = 0.74;
      //  marker.color.g = 0.2;
      //  marker.color.b = 0.92;
      //  node_q.setRPY(0.0, 0.0, yaw);  
      //  marker.pose.orientation.x = node_q[0];
      //  marker.pose.orientation.y = node_q[1];
      //  marker.pose.orientation.z = node_q[2];
      //  marker.pose.orientation.w = node_q[3];
      //  marker.type = visualization_msgs::Marker::ARROW;
      //  
      //}
      markers.markers.push_back(marker);

      ++id;
    }
    return markers;
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

   visualization_msgs::Marker GpsNavigationNode::VisMarkersFromNodes2(std::vector<Node*> nodes, int marker_type, float color_r, float color_g, float color_b){
    visualization_msgs::Marker marker;
    ros::Time current_t = ros::Time::now();
    // std::cout<<"Visualize node size: "<<nodes.size()<<std::endl;
    if(nodes.size()){
      int id = 0;
      marker.id = id;
      marker.ns = "construction_pub_ns";
      marker.header.frame_id = "map";
      marker.header.seq = id;
      marker.header.stamp = current_t;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 2.0;
      marker.color.a = 1.0;
      marker.color.r = color_r;
      marker.color.g = color_g;
      marker.color.b = color_b;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
    }
    marker.type = visualization_msgs::Marker::LINE_LIST;
    for(auto node1: nodes){
      if(nodes.size()){
        //std::cout<<nodes.size()<<"   "<<(node1->constructionEdges==nullptr)<<std::endl;
      }
      if(node1->constructionEdges==nullptr)
        continue;
      auto children = node1->constructionEdges->nodes;
      for(auto node2: children){
        if(std::find(nodes.begin(), nodes.end(), node2)==nodes.end()) 
          continue;
        std::pair<double, double> x_y1 = RelativeDisplacement(ref_start, node1);
        std::pair<double, double> x_y2 = RelativeDisplacement(ref_start, node2);
        geometry_msgs::Point p1, p2;
        p1.x = x_y1.first;
        p1.y = x_y1.second;
        p1.z = 0.0;
        p2.x = x_y2.first;
        p2.y = x_y2.second;
        p2.z = 0.0;
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }
    return marker;
  }
 
  void GpsNavigationNode::VisualizeFootPaths(std::vector<Way*> ways){
    nav_msgs::Path paths;
    visualization_msgs::Marker current_direction;
    current_direction.action = visualization_msgs::Marker::ADD;
    

    paths.header.frame_id = "map";
    geometry_msgs::PoseStamped current_pose;
    
    Node* ref_start = new Node;
    ref_start->lat = kOsmOriginX;
    ref_start->lon = kOsmOriginY;
    
    int counter = 0;
    ros::Time current_time = ros::Time::now();
    for (unsigned int i=0; i<ways.size(); i++){ 
      paths.poses.clear();
      current_time = ros::Time::now();
      for (unsigned int j=0; j<ways[i]->nodes.size(); j++){
        std::pair<double, double> dx_dy = RelativeDisplacement(ref_start, ways[i]->nodes[j]);
        // Accumulate orientation markers
        // Estimate quaternion representation
        //node_directions.markers.push_back(current_direction);
        
        // Create road network segment
        current_pose.pose.position.x = dx_dy.first;
        current_pose.pose.position.y = dx_dy.second;
        current_pose.pose.position.z = 0.0;
        current_pose.header.seq = counter;
        current_pose.header.frame_id = "map";
        current_pose.header.stamp = current_time;
        paths.poses.push_back(current_pose);

        ++counter; 
      }
      footpaths_viz.publish(paths);
    }
    //road_network.header.stamp = current_time;
  
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
