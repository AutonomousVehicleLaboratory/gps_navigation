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
    //TODO: verify
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
        if( k == "highway" && v == "unclassified" ||
            k == "highway" && v == "service" ||
            k == "highway" && v == "tertiary" ||
            k == "highway" && v == "residential") {
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
    //TODO
    //std::cout << "Starting Dijkstra" << std::endl;
    std::vector<Node*> traj;
    std::stack<Node*> traj_stack = osm_graph_.Dijkstra(point1, point2);
    while(!traj_stack.empty()){
      //if(traj_stack.top()->osm_id != -1){ 
      //  std::cout << "node: " << traj_stack.top()->graph_id << " | " << traj_stack.top()->graph_id << std::endl;
      //}
      traj.push_back(traj_stack.top());
      traj_stack.pop();
    }
    return traj;
  }
  std::vector<Way*> Map::GetWays(){
    return ways_;
  }
  //std::vector<Node*> Map::GetNavigationNodes(){
  //  return navigation_nodes_;
  //}
  void Map::ResetPlan(){
    osm_graph_.ResetGraph(navigation_nodes_);
  }

  Navigation::Navigation(){};
  Navigation::Navigation(std::string map_path, double origin_x, double origin_y){
    osm_map_ = new Map(map_path);
  }

  Map* Navigation::GetMap(){
    return osm_map_;
  }
  
}
