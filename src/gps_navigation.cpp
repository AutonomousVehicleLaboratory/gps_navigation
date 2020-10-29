#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/utils.h>

namespace gps_navigation{
  Map::Map(){}
  Map::Map(std::string map_path){
    // Read and Parse OSM XML format
    xml_map_ = TiXmlDocument(map_path);
    osm_status_ = xml_map_.LoadFile();
    
    // Extract information about map's bounds, and sequences of nodes/ways
    osm_map_ = xml_map_.FirstChildElement();
    osm_bounds_ = osm_map_->FirstChild("bounds");
    osm_nodes_ = osm_map_->FirstChild("node");
    osm_ways_ = osm_map_->FirstChild("way");
  
    // Parse map
    ParseMap();    
    std::cout << " Parsed map" << std::endl;
    // Define OSM graph
    osm_graph_ = OsmGraph();
    osm_graph_.Generate(ways_, navigation_nodes_);
    
  }

  std::vector<Node*> Map::ExtractNodes(int start_node_id, int end_node_id){
    auto start_node = nodes_.find(start_node_id);
    auto end_node = nodes_.find(end_node_id);
    double dist = GreatCircleDistance(start_node->second, end_node->second);
    int count_new_nodes = dist / 2.0;
    
    std::vector<Node*> new_nodes;
    
    for (int i = 0; i < count_new_nodes; i++) {
      Node* new_node = new Node;
      new_node->lat = ((count_new_nodes - i) * start_node->second->lat + (i + 1) * end_node->second->lat) / (count_new_nodes + 1);
      new_node->lon = ((count_new_nodes - i) * start_node->second->lon + (i + 1) * end_node->second->lon) / (count_new_nodes + 1);
      new_nodes.push_back(new_node);    
      
    }
    return new_nodes;
      
  } 
  void Map::ParseMap(){
    TiXmlHandle node_handle = TiXmlHandle(osm_nodes_);
    TiXmlHandle way_handle = TiXmlHandle(osm_ways_);

    // Extract all nodes: unordered_map<int, Node*> (id: osm_id)
    // Extract ways of interest (vector<Way*>) with their corresponding nodes (id: osm_id and graph_id)
    //      unordered_map<int, Node*> and interpolate nodes 
    //
    int current_graph_id = 0;
    int node_id;
    double lat, lon;

    // Extract all of the node information 
    TiXmlElement *node_element = node_handle.Element(); 
    for (node_element; node_element; node_element= node_element->NextSiblingElement("node")) {
      node_element->Attribute("id", &node_id);
      node_element->Attribute("lat", &lat);
      node_element->Attribute("lon", &lon);

      auto node_check = nodes_.find(node_id);
      if(node_check == nodes_.end()){
        Node* new_node = new Node;
        new_node->osm_id = node_id; 
        new_node->lat = lat; 
        new_node->lon = lon; 
        nodes_.insert({node_id, new_node});
      } 
    }

    // Extract all of the way information
    int way_id;
    TiXmlElement *tag;
    TiXmlElement *nd;
    TiXmlElement *way_element = way_handle.Element();
    for (way_element; way_element; way_element = way_element->NextSiblingElement("way")) {
      way_element->Attribute("id", &way_id); 
      tag = way_element->FirstChildElement("tag");
      nd = way_element->FirstChildElement("nd");
      bool is_oneway = false;
      bool way_valid = false;
      std::string k, v;
 
      // For current way, check if relevant for driving scenarios
      // Find its corresponding nodes
      // Interpolate nodes and assign new ids (graph_id)
      for (tag; tag; tag = tag->NextSiblingElement("tag")) {
        // A tag includes a key "k" and a value "v"
        k = tag->Attribute("k");
        v = tag->Attribute("v");
        if( k == "oneway" && v == "yes"){
          is_oneway = true;
          //std::cout << "found a oneway road" << std::endl;
        }
        if( (k == "highway" && v == "unclassified") ||
            (k == "highway" && v == "service") ||
            (k == "highway" && v == "tertiary") ||
            (k == "highway" && v == "residential")) {
          //std::cout << "found way: " << way_id << std::endl;
          way_valid = true;
           
          }
          //break;  
      }
      if(way_valid){ 
        // Create new way struct
        Way* new_way = new Way;
        new_way->way_id = way_id;
        new_way->one_way = is_oneway;
        ways_.push_back(new_way);

        // iterate through nodes in way
        // interpolate node pairs and assign unique graph_id
        // intersert to way's way and push into vector<Way*>
        int start_node_id = 0;
        int end_node_id = 0;
        nd->Attribute("ref", &start_node_id);
        auto start_node = nodes_.find(start_node_id)->second;
        auto end_node = nodes_.find(start_node_id)->second;
        // Check if node already exists in navigation_nodes_
        auto check = navigation_nodes_.find(start_node->graph_id);
        if(check == navigation_nodes_.end()){
          start_node->graph_id = current_graph_id;
          navigation_nodes_.insert({current_graph_id, start_node});
          ++current_graph_id;
        }
        new_way->nodes.push_back(start_node); 

        while(nd->NextSiblingElement("nd")) {
          nd->NextSiblingElement("nd")->Attribute("ref", &end_node_id); 
          end_node = nodes_.find(end_node_id)->second;
 
          // Interpolate nodes
          std::vector<Node*> interpolated_nodes = ExtractNodes(start_node_id, end_node_id);
          // Tag all nodes with new graph_id and insert into ways and nodes 
          for(unsigned int i=0; i<interpolated_nodes.size(); i++){
            navigation_nodes_.insert({current_graph_id, interpolated_nodes[i]});
            interpolated_nodes[i]->graph_id = current_graph_id;
            new_way->nodes.push_back(interpolated_nodes[i]); 
            ++current_graph_id;
          }
          // Check to see if end node already exists in navigation_nodes_
          check = navigation_nodes_.find(end_node->graph_id);
          if(check == navigation_nodes_.end()){
            end_node->graph_id = current_graph_id;
            navigation_nodes_.insert({current_graph_id, end_node});  
            ++current_graph_id;
          }
          new_way->nodes.push_back(end_node);
          nd = nd->NextSiblingElement("nd");
          start_node_id = end_node_id;   
        }
      }
    } 
  }
  Node* Map::FindClosestNode(double lat, double lon){
    Node point;
    point.lat = lat;
    point.lon = lon;
    auto nodes_start = navigation_nodes_.begin();
    auto nodes_end = navigation_nodes_.end();

    Node* shortest_node = NULL;
    double shortest_distance = INFINITY;
    double curr_distance = INFINITY;
    while(nodes_start != nodes_end){
      curr_distance = GreatCircleDistance(&point, nodes_start->second);
      if(curr_distance < shortest_distance ){
        shortest_distance = curr_distance;
        shortest_node = nodes_start->second; 
      }
      ++nodes_start;
    }
    //std::cout << "Node (" << shortest_node->graph_id << ")distance: " << shortest_distance << std::endl;
    
    return shortest_node;
  }
  Node* Map::FindClosestNodeRelative(double x, double y, double origin_lat, double origin_lon){
    Node point;
    point.lat = origin_lat;
    point.lon = origin_lon;
    auto nodes_start = navigation_nodes_.begin();
    auto nodes_end = navigation_nodes_.end();

    Node* shortest_node = NULL;
    double shortest_distance = INFINITY;
    double curr_distance = INFINITY;
    std::pair<double, double> curr_distance_vec;
    while(nodes_start != nodes_end){
      curr_distance_vec = RelativeDisplacement(&point, nodes_start->second);
      curr_distance = sqrt(pow(curr_distance_vec.first - x, 2) + pow(curr_distance_vec.second -y, 2)); 
      if(curr_distance < shortest_distance ){
        shortest_distance = curr_distance;
        shortest_node = nodes_start->second; 
      }
      ++nodes_start;
    }
    //std::cout << "Relative Node (" << shortest_node->graph_id << ")distance: " << shortest_distance << std::endl;
    
    return shortest_node;

  }

  std::vector<Node*> Map::ShortestPath(Node* point1, Node* point2){
    //std::cout << "Starting Dijkstra" << std::endl;
    std::vector<Node*> traj;
    std::stack<Node*> traj_stack = osm_graph_.Dijkstra(point1, point2);
    while(!traj_stack.empty()){
      traj.push_back(traj_stack.top());
      traj_stack.pop();
    }
    return traj;
  } std::vector<Way*> Map::GetWays(){
    return ways_;
  }
  void Map::ResetPlan(){
    osm_graph_.ResetGraph(navigation_nodes_);
  }

  Navigation::Navigation(){};
  Navigation::Navigation(std::string map_path, double lat_origin, double lon_origin){
    lat_origin_ = lat_origin;
    lon_origin_ = lon_origin;
    ref_origin_ = new Node;
    ref_origin_->lat = lat_origin; 
    ref_origin_->lon = lon_origin; 
    osm_map_ = new Map(map_path);
    //state_.pose = new Node;
  }

  Map* Navigation::GetMap(){
    return osm_map_;
  }
  void Navigation::SetStart(double lat, double lon){
    state_.pose.lat = lat;
    state_.pose.lon = lon;
    start_node_ = osm_map_->FindClosestNode(lat, lon);
    std::pair<double, double> x_y = RelativeDisplacement(ref_origin_, start_node_);
    //std::pair<double, double> x_y = RelativeDisplacement(ref_origin_, state_.pose);
    state_.x_ego = x_y.first;
    state_.y_ego = x_y.second;
    next_node_index_ = 0;
    nearest_gps_node_ = start_node_;
  }
  void Navigation::SetTarget(double lat, double lon){
    end_node_ = osm_map_->FindClosestNode(lat, lon);
  }
  void Navigation::SetTargetRelative(double x_dest, double y_dest){
    end_node_ = osm_map_->FindClosestNodeRelative(x_dest, y_dest, lat_origin_, lon_origin_);
  }
  void Navigation::ResetPlan(){
    osm_map_->ResetPlan();
  } 
  std::vector<Node*> Navigation::Plan(){
    current_plan_ = osm_map_->ShortestPath(start_node_, end_node_);
    next_node_index_ = 0;
    return current_plan_;
  }
  std::pair<int, Node*> Navigation::FindClosestPlannedNode(){
    double closest_distance = INFINITY;
    double current_distance = INFINITY;
    std::pair<int, Node*> closest_node;
    
    for(unsigned int i=0; i<current_plan_.size(); i++){
      std::pair<double, double> dx_dy = RelativeDisplacement(&state_.pose, current_plan_[i]);
      current_distance = sqrt(dx_dy.first*dx_dy.first + dx_dy.second*dx_dy.second);
      if(current_distance < closest_distance){
        closest_distance = current_distance;
        closest_node.first = i;
        closest_node.second = current_plan_[i];
      }
    }
    return closest_node; 
  }
  std::tuple<bool, long, double, double, double> Navigation::UpdateState(double lat, double lon, double v, double w_z, double a_x, double t){
    std::tuple<bool, long, double, double, double> current_state{false, 0, 0.0, 0.0, -1.0};
    //std::tuple<bool, double, double, double> current_state};
    if(t_prev_ == -1.0){
      t_prev_ = t;
      state_.a_x = a_x;
      state_.w_z = w_z;
      state_.v = v;
      // Set initial position/orientation of ego vehicle
      if(current_plan_.size() > 1){
        std::pair<double, double> dx_dy = RelativeDisplacement(current_plan_[next_node_index_], 
                                                               current_plan_[next_node_index_+1]);
        //double dx_dy_norm = sqrt(dx_dy.first*dx_dy.first + dx_dy.second*dx_dy.second);
        state_.yaw_ego = atan2(start_node_->dx_dy.second, start_node_->dx_dy.first);
        state_.prev_yaw_ego = state_.yaw_ego;
        state_.pose.lat = lat;
        state_.pose.lon = lon;
        // Node and position of planned node closest to ego vehicle
        state_.pose = *current_plan_[next_node_index_];
        std::pair<double, double> x_y_node = RelativeDisplacement(ref_origin_, &state_.pose);
        state_.x_ego = x_y_node.first;
        state_.y_ego = x_y_node.second;
        //std::pair<int, Node*> planned_node = FindClosestPlannedNode();
      }

      return current_state; 
    }
      
    // If a new gps measurement is received
    if(lat != state_.pose.lat){
      state_.pose.lat = lat;
      state_.pose.lon = lon;
      // Node and position of planned node closest to ego vehicle
      std::pair<double, double> x_y_ego = RelativeDisplacement(ref_origin_, &state_.pose);

      // Define condition to set position/orientation using gps or imu
      // If closest node to ego is x meters past its threshold, update x_ego, y_ego and angles, else set flag for imu
      double dist_diff = sqrt((x_y_ego.first - state_.x_ego)*(x_y_ego.first - state_.x_ego) + 
                              (x_y_ego.second - state_.y_ego)*(x_y_ego.second - state_.y_ego));
      
      // If the new gps location is too far from previously expected 
      if(dist_diff >= 2.0){
        // If this node does not contain orientation information
        // TODO: or use IMU
        if((start_node_->dx_dy.first == 0) && (start_node_->dx_dy.second == 0)){
          return current_state;
        }
        // Set position based on planned node closest to ego vehicle
        std::pair<int, Node*> planned_node = FindClosestPlannedNode();
        if(planned_node.first == current_plan_.size()-1){
          return current_state;
        }
        if(planned_node.second == NULL){
          std::cout << "null" << std::endl;
        } 
        std::pair<double, double> x_y_node = RelativeDisplacement(ref_origin_, planned_node.second);
        next_node_index_ = planned_node.first;
        start_node_ = planned_node.second;
        state_.x_ego = x_y_node.first;
        state_.y_ego = x_y_node.second;
        state_.prev_yaw_ego = state_.yaw_ego;
        // Set orientation
        std::cout << "i: " << next_node_index_ << " len: " << current_plan_.size() << std::endl;
        std::pair<double, double> dx_dy = RelativeDisplacement(current_plan_[next_node_index_], 
                                                               current_plan_[next_node_index_+1]);
        double dx_dy_norm = sqrt(dx_dy.first*dx_dy.first + dx_dy.second*dx_dy.second);
        // TODO: use imu for drastic changes in orientation
        state_.prev_yaw_ego = state_.yaw_ego;
        state_.yaw_ego = atan2(start_node_->dx_dy.second, start_node_->dx_dy.first);
        current_state = std::make_tuple(true, next_node_index_, state_.x_ego, state_.y_ego, state_.yaw_ego); 
        return current_state;
      }

    }
    //else{
    //  // TODO: Use IMU to update position 
    //  
    //}
     
  
  } 
  
   
  
}
