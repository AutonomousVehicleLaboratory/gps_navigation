/**
 * @file gps_navigation.cpp
 *
 * @brief Core classes for handling graph search and ego vehicle states. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

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

    // Defines node types
    
  
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

  void Map::ParseNode(TiXmlElement *node_element){

    int node_id;
    double lat, lon;

    std::string k, v;
    TiXmlElement *node_tag;
      
    if(!node_element) return;    

    // Node ID and coordinates
    node_element->Attribute("id", &node_id);
    node_element->Attribute("lat", &lat);
    node_element->Attribute("lon", &lon);

    // Avoid inserting duplicated nodes 
    auto node_check = nodes_.find(node_id);
    if(node_check != nodes_.end()) return ;

    // Create new node instance
    Node* new_node = new Node;
    new_node->osm_id = node_id; 
    new_node->lat = lat; 
    new_node->lon = lon; 


    // Iterate through node's tags to identify special types 
    node_tag = node_element->FirstChildElement("tag");;
    std::vector<std::pair<std::string, std::string>> attributes;

    // Determine key node type
    NodeType node_type;
    bool is_key_type = false;
    for (; node_tag; node_tag = node_tag->NextSiblingElement("tag")) {
      k = node_tag->Attribute("k");
      v = node_tag->Attribute("v");
      
      // accumulate all attributes
      attributes.push_back(std::make_pair(k, v));

      // check for stops, traffic signals and crossings and track them 
      // these are node attributes only
      if(k == "highway"){
        if(v == "stop"){
          node_type = NodeType::kStopSign; 
          is_key_type = true;
          //stop_nodes_.insert({node_id, new_node});
        }
        else if(v == "traffic_signals"){
          node_type = NodeType::kTrafficSignal; 
          is_key_type = true;
          //traffic_signal_nodes_.insert({node_id, new_node});
        }
        else if(v == "crossing"){
          node_type = NodeType::kCrossing; 
          is_key_type = true;
          //crossing_nodes_.insert({node_id, new_node});
        }
      }
    }
    
    // Only add attributes for key types
    if(is_key_type){
      new_node->attributes = attributes;
      new_node->key_attribute = node_type;
    }

    // keep a track of all nodes for now
    nodes_.insert({node_id, new_node});

  }
  void Map::ParseWay(TiXmlElement *way_element){
    if(!way_element) return;

    int way_id;
    TiXmlElement *tag;
    TiXmlElement *nd;

    way_element->Attribute("id", &way_id); 
    tag = way_element->FirstChildElement("tag");
    nd = way_element->FirstChildElement("nd");
    bool is_oneway = false;
    bool way_valid = false;
    std::string k, v;
 
    // For current way, check if any of its tags are relevant for driving scenarios
    // these are way attributes only
    for (; tag; tag = tag->NextSiblingElement("tag")) {
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
      // TODO: for footpaths, tag nodes associated with a footpath way
      // TODO: for construction, find closest nodes do it and tag nodes associated with construction
      
    }

    // Find road way's corresponding nodes
    // Interpolate nodes and assign new ids (graph_id)
    if(way_valid){ 
      // Create new way struct
      Way* new_way = new Way;
      new_way->way_id = way_id;
      new_way->one_way = is_oneway;
      //new_way->key_attribute = WayType::kRoad;
      ways_.push_back(new_way);

      // iterate through nodes in way
      // interpolate node pairs and assign unique graph_id
      // intersert to way's way and push into vector<Way*>
      int start_node_id = 0;
      int end_node_id = 0;

      // nd points at first node of current road way
      nd->Attribute("ref", &start_node_id);

      auto start_node = nodes_.find(start_node_id)->second;
      auto end_node = nodes_.find(start_node_id)->second;

      // Check if start node already exists in navigation_nodes_ to avoid duplicates in graph
      auto check = navigation_nodes_.find(start_node->graph_id);
      if(check == navigation_nodes_.end()){
        start_node->graph_id = current_graph_id_;
        navigation_nodes_.insert({current_graph_id_, start_node});
        ++current_graph_id_;
      }

      // Add start node to new way
      new_way->nodes.push_back(start_node); 

      while(nd->NextSiblingElement("nd")) {

        // Interpolate nodes
        nd->NextSiblingElement("nd")->Attribute("ref", &end_node_id); 
        end_node = nodes_.find(end_node_id)->second;
        std::vector<Node*> interpolated_nodes = ExtractNodes(start_node_id, end_node_id);

        // Tag all nodes with new graph_id and insert into ways and nodes 
        for(unsigned int i=0; i<interpolated_nodes.size(); i++){
          navigation_nodes_.insert({current_graph_id_, interpolated_nodes[i]});
          interpolated_nodes[i]->graph_id = current_graph_id_;
          new_way->nodes.push_back(interpolated_nodes[i]); 
          ++current_graph_id_;
        }
        // Check if end node already exists in navigation_nodes_ to avoid duplicates in graph
        check = navigation_nodes_.find(end_node->graph_id);
        if(check == navigation_nodes_.end()){
          end_node->graph_id = current_graph_id_;
          navigation_nodes_.insert({current_graph_id_, end_node});  
          ++current_graph_id_;
        }

        // Insert end node to new way
        new_way->nodes.push_back(end_node);
        nd = nd->NextSiblingElement("nd");
        start_node_id = end_node_id;   
      }
    }
    return;
  }
  void Map::ParseMap(){
    TiXmlHandle node_handle = TiXmlHandle(osm_nodes_);
    TiXmlHandle way_handle = TiXmlHandle(osm_ways_);

    // Extract all nodes: unordered_map<int, Node*> (id: osm_id)
    // Extract ways of interest (vector<Way*>) with their corresponding nodes (id: osm_id and graph_id)
    //      unordered_map<int, Node*> and interpolate nodes 
    //

    // Extract all of the node information 
    // TODO: track nodes of type crossings, stopsigns and traffic signs
    
    TiXmlElement *node_element = node_handle.Element(); 

    for (; node_element; node_element= node_element->NextSiblingElement("node")) {
      // Parse node and insert into crossing_nodes, stop_nodes_, traffic_sign_nodes_ or nodes_ 
      ParseNode(node_element); 
    }

    // Extract all of the way information
    TiXmlElement *way_element = way_handle.Element();
    for (; way_element; way_element = way_element->NextSiblingElement("way")) {
      ParseWay(way_element);
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
  } 

  void Map::FindRoadFeatures(Node* point, int k){
    osm_graph_.BFS(point, k);
  }

  std::vector<Node*> Map::GetStops(){
    return osm_graph_.RetrieveStops();
  }

  std::vector<Node*> Map::GetCrossings(){
    return osm_graph_.RetrieveCrossings();
  }

  std::vector<Node*> Map::GetTrafficSignals(){
    return osm_graph_.RetrieveTrafficSignals();
  }

  std::vector<Way*> Map::GetWays(){
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
  std::pair<unsigned int, Node*> Navigation::FindClosestPlannedNode(){
    double closest_distance = INFINITY;
    double current_distance = INFINITY;
    std::pair<unsigned int, Node*> closest_node;
    
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
  bool Navigation::CheckNextAngles(unsigned long k, double thresh){
    std::pair<double, double> dx_dy1;
    std::pair<double, double> dx_dy2;
    double prev_angle = 0;
    double current_angle = 0;
    double yaw_diff = 0;
    dx_dy1 = RelativeDisplacement(ref_origin_, current_plan_[next_node_index_]);
    dx_dy2 = RelativeDisplacement(ref_origin_, current_plan_[next_node_index_+1]);
    prev_angle = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                           (dx_dy1.first - dx_dy2.first));

    for(unsigned int i=next_node_index_+1; i<std::min(next_node_index_+k, current_plan_.size()-1); i++){
        dx_dy1 = RelativeDisplacement(ref_origin_, current_plan_[i]);
        dx_dy2 = RelativeDisplacement(ref_origin_, current_plan_[i+1]);
        current_angle = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                                (dx_dy1.first - dx_dy2.first));
      
        yaw_diff = fabs(fmod(M_PI + current_angle, 2*M_PI)-
                       fmod(M_PI + prev_angle, 2*M_PI)); 

        prev_angle = current_angle;
        if(yaw_diff >= thresh){
          return true;
        }
    }
    return false;
  }

  void Navigation::GenerateSTGraph(double lat, double lon, double v, double t){
    
    std::vector<Node*> st_graph;
    Node* veh_pose;
    // [pose | traversed | planned |
    //  crossings | stops | trafficsignals | footpaths | construction | road_network |]
    // Insert pose, traversed and planned nodes

    // Get crossings, stops, traffic signals, footpaths, construction
    // Initialize states

    //if(t_prev_ == -1){
    //  g_state_.pose.lat = lat;
    //  g_state_.pose.lon = lon;
    //  t_prev = t;

    //  return st_graph;
    //}
    // If a new gps measurement is received
    if((lat != g_state_.pose.lat) && use_gps_){
      g_state_.pose.lat = lat;
      g_state_.pose.lon = lon;

      // Node and position of planned node closest to ego vehicle
      std::pair<double, double> x_y_ego = RelativeDisplacement(ref_origin_, &g_state_.pose);

      // Condition to set position/orientation using gps
      double dist_diff = sqrt((x_y_ego.first - g_state_.x_ego)*(x_y_ego.first - g_state_.x_ego) + 
                              (x_y_ego.second - g_state_.y_ego)*(x_y_ego.second - g_state_.y_ego));
      
      // If the new GPS location is too far from pose estimated by odometry 
      if(dist_diff >= 2.0 || t_prev_ == -1){
        // If this node does not contain orientation information
        if((start_node_->dx_dy.first == 0) && (start_node_->dx_dy.second == 0)){
          return;
        }
        // Set position based on planned node closest to ego vehicle
        std::pair<unsigned int, Node*> planned_node = FindClosestPlannedNode();
        next_node_index_ = planned_node.first;
        start_node_ = planned_node.second;

        // Set orientation
        std::pair<double, double> dx_dy1 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_]);
        std::pair<double, double> dx_dy2 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+1]);
        std::pair<double, double> dx_dy3 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+2]);
        g_state_.current_yaw = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                               (dx_dy1.first - dx_dy2.first));
        g_state_.next_yaw = M_PI + atan2((dx_dy2.second - dx_dy3.second),
                               (dx_dy2.first - dx_dy3.first));
        t_prev_ = t;
        g_state_.x_ego = dx_dy1.first;
        g_state_.y_ego = dx_dy1.second;
        x_next_ = dx_dy2.first;
        y_next_ = dx_dy2.second;

        //veh_pose = current_plan_[next_node_index_];
        //current_state = std::make_tuple(true, next_node_index_, g_state_.x_ego, g_state_.y_ego, g_state_.current_yaw); 
        //return current_state;
      }

    }
    else{
      // Use odometry to update position
      double dt = t - t_prev_;
      double dd = 2*v*dt;
      t_prev_ = t;
      
      //g_state_.yaw_ego += w_z * dt;
      double dx = dd*cos(g_state_.current_yaw);
      double dy = dd*sin(g_state_.current_yaw);
      double predicted_x = g_state_.x_ego + dx;
      double predicted_y = g_state_.y_ego + dy;
      double dist_to_next = sqrt(pow(predicted_x - x_next_, 2) + pow(predicted_y - y_next_, 2));
      
      
      // Update vehicle state with respect to planned path
      // If distance to next predicted state is less than a threshold, update
      if(dist_to_next < 2.0){
        std::pair<double, double> dx_dy1 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_]);
        std::pair<double, double> dx_dy2 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+1]);
        std::pair<double, double> dx_dy3 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+2]);
        g_state_.current_yaw = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                               (dx_dy1.first - dx_dy2.first));
        g_state_.next_yaw = M_PI + atan2((dx_dy2.second - dx_dy3.second),
                               (dx_dy2.first - dx_dy3.first));

        // Update starting pose
        g_state_.x_ego = x_next_;
        g_state_.y_ego = y_next_;
        // Update distance based on remaining distance
        dd -= dist_to_next;
        // Update orientation based on next waypoint
        next_node_index_ += 1;
        x_next_ = dx_dy2.first; 
        y_next_ = dx_dy2.second; 

      }
      
       
      //veh_pose = current_plan_[next_node_index_];

      g_state_.x_ego += dd*cos(g_state_.current_yaw);
      g_state_.y_ego += dd*sin(g_state_.current_yaw);
      //current_state = std::make_tuple(true, next_node_index_, g_state_.x_ego, g_state_.y_ego, g_state_.sim_yaw); 
       
      
    }

    // TODO: Extract graph
    veh_pose = current_plan_[next_node_index_];
    GetMap()->FindRoadFeatures(veh_pose, 20);

  }

  std::tuple<bool, long, double, double, double> Navigation::UpdateState(double lat, double lon, double v, double w_z, double a_x, double t){
    std::tuple<bool, long, double, double, double> current_state{false, 0, 0.0, 0.0, -1.0};

    // Initialize states
    if(t_prev_ == -1.0){
      t_prev_ = t;
      state_.a_x = a_x;
      state_.w_z = w_z;
      state_.v = v;
      // Set initial position/orientation of ego vehicle
      if(current_plan_.size() > 2){
        std::pair<double, double> dx_dy1 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_]);
        std::pair<double, double> dx_dy2 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+1]);
        std::pair<double, double> dx_dy3 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+2]);
        state_.current_yaw = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                               (dx_dy1.first - dx_dy2.first));
        state_.next_yaw = M_PI + atan2((dx_dy2.second - dx_dy3.second),
                               (dx_dy2.first - dx_dy3.first));
        //state_.current_yaw = fmod(state_.current_yaw, 2*M_PI);
        //state_.next_yaw = fmod(state_.next_yaw, 2*M_PI);
        state_.sim_yaw = state_.current_yaw;
        state_.pose.lat = lat;
        state_.pose.lon = lon;
        // Node and position of planned node closest to ego vehicle
        state_.pose = *current_plan_[next_node_index_];
        std::pair<double, double> x_y_node = RelativeDisplacement(ref_origin_, &state_.pose);
        state_.x_ego = x_y_node.first;
        state_.y_ego = x_y_node.second;
        x_next_ = dx_dy2.first;
        y_next_ = dx_dy2.second;
      }

      return current_state; 
    }
      
    // If a new gps measurement is received
    if((lat != state_.pose.lat) && use_gps_){
      state_.pose.lat = lat;
      state_.pose.lon = lon;
      // Node and position of planned node closest to ego vehicle
      std::pair<double, double> x_y_ego = RelativeDisplacement(ref_origin_, &state_.pose);

      // Condition to set position/orientation using gps
      double dist_diff = sqrt((x_y_ego.first - state_.x_ego)*(x_y_ego.first - state_.x_ego) + 
                              (x_y_ego.second - state_.y_ego)*(x_y_ego.second - state_.y_ego));
      
      // If the new GPS location is too far from pose estimated by IMU
      if(dist_diff >= 2.0){
        // If this node does not contain orientation information
        if((start_node_->dx_dy.first == 0) && (start_node_->dx_dy.second == 0)){
          return current_state;
        }
        // Set position based on planned node closest to ego vehicle
        std::pair<unsigned int, Node*> planned_node = FindClosestPlannedNode();
        if(planned_node.first == current_plan_.size()-2){
          return current_state;
        }
        //std::pair<double, double> x_y_node = RelativeDisplacement(ref_origin_, planned_node.second);
        next_node_index_ = planned_node.first;
        start_node_ = planned_node.second;

        // Set orientation
        //std::cout << "i: " << next_node_index_ << " len: " << current_plan_.size() << std::endl;
        std::pair<double, double> dx_dy1 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_]);
        std::pair<double, double> dx_dy2 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+1]);
        std::pair<double, double> dx_dy3 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+2]);
        state_.current_yaw = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                               (dx_dy1.first - dx_dy2.first));
        state_.next_yaw = M_PI + atan2((dx_dy2.second - dx_dy3.second),
                               (dx_dy2.first - dx_dy3.first));
        //state_.current_yaw = fmod(state_.current_yaw, 2*M_PI);
        //state_.next_yaw = fmod(state_.next_yaw, 2*M_PI);
        state_.sim_yaw = state_.current_yaw;
        t_prev_ = t;
        state_.x_ego = dx_dy1.first;
        state_.y_ego = dx_dy1.second;
        x_next_ = dx_dy2.first; 
        y_next_ = dx_dy2.second; 
        current_state = std::make_tuple(true, next_node_index_, state_.x_ego, state_.y_ego, state_.current_yaw); 
        return current_state;
      }

    }
    else{
      // TODO: Use IMU to update position
      double dt = t - t_prev_;
      double dd = 2*v*dt;
      t_prev_ = t;
      
      //state_.yaw_ego += w_z * dt;
      double dx = dd*cos(state_.current_yaw);
      double dy = dd*sin(state_.current_yaw);
      double predicted_x = state_.x_ego + dx;
      double predicted_y = state_.y_ego + dy;
      double dist_to_next = sqrt(pow(predicted_x - x_next_, 2) + pow(predicted_y - y_next_, 2));
      
      
      // Update vehicle state with respect to planned path
      // If distance to next predicted state is less than a threshold, update
      if(dist_to_next < 2.0){
        //use_gps_ = false;
        //if(abs(state_.next_yaw - state_.current_yaw) >= 0.08){
        //  state_.sim_yaw += w_z * dt;
        //}
        std::pair<double, double> dx_dy1 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_]);
        std::pair<double, double> dx_dy2 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+1]);
        std::pair<double, double> dx_dy3 = RelativeDisplacement(ref_origin_, 
                                                               current_plan_[next_node_index_+2]);
        state_.current_yaw = M_PI + atan2((dx_dy1.second - dx_dy2.second),
                               (dx_dy1.first - dx_dy2.first));
        state_.next_yaw = M_PI + atan2((dx_dy2.second - dx_dy3.second),
                               (dx_dy2.first - dx_dy3.first));
        //state_.current_yaw = fmod(state_.current_yaw, 2*M_PI);
        //state_.next_yaw = fmod(state_.next_yaw, 2*M_PI);
        //state_.sim_yaw = state_.current_yaw;
        // Update starting pose
        state_.x_ego = x_next_;
        state_.y_ego = y_next_;
        // Update distance based on remaining distance
        dd -= dist_to_next;
        // Update orientation based on next waypoint
        next_node_index_ += 1;
        x_next_ = dx_dy2.first; 
        y_next_ = dx_dy2.second; 
      //  }
      }
      
      // If angle between to adjacent nodes is large, simulate rotation
      double yaw_diff = fabs(fmod(M_PI + state_.current_yaw, 2*M_PI)-
                            fmod(M_PI + state_.next_yaw, 2*M_PI)); 
      std::cout << "New Diff: "<< yaw_diff << std::endl;
      bool simulate = CheckNextAngles(8, 0.1);
      //if(fabs(state_.next_yaw - state_.current_yaw) > 0.1){
      if(simulate){
        //std::cout << "Normal DIFF: "<< abs(state_.next_yaw - state_.current_yaw) << std::endl;
        //std::cout << "current: "<< state_.current_yaw << " next: " << state_.next_yaw << std::endl;
        use_sim_yaw_ = true;
        use_gps_ = false; 
        sim_distance_ = 0;
      }
      
      // Once a simulation is triggered, the vehicle will use the imu to estimate 
      // orientation until a distance threshold is met 
      if(use_sim_yaw_){
        sim_distance_ += 2*v*dt;
        state_.sim_yaw += w_z*dt;
        if(sim_distance_ >= 20.0){
          use_sim_yaw_ = false;
          use_gps_ = true;
          state_.sim_yaw = state_.current_yaw;
        }
      } 
      else{
        state_.sim_yaw = state_.current_yaw;
      }
       
      state_.x_ego += dd*cos(state_.current_yaw);
      state_.y_ego += dd*sin(state_.current_yaw);
      current_state = std::make_tuple(true, next_node_index_, state_.x_ego, state_.y_ego, state_.sim_yaw); 
       
      
    }
     
    return current_state; 
  } 
  
   
  
}
