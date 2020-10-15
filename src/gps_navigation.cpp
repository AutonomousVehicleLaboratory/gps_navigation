#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/utils.h>

namespace gps_navigation{
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
    osm_graph = OsmGraph();
    osm_graph.Generate(ways_, navigation_nodes_);
    
  }
  /*
  double Map::GreatCircleDistance(Node* point1, Node* point2){
    //TODO: verify
    //std::cout << "P1: " << point1->lat << " ; " << point1->lon << std::endl; 
    //std::cout << "P2: " << point2->lat << " ; " << point2->lon << std::endl; 
    double DEG2RAD = M_PI / 180;
    double R = 6371e3;
    double dLat = point2->lat*DEG2RAD - point1->lat*DEG2RAD;
    double dLon = point2->lon*DEG2RAD - point1->lon*DEG2RAD;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(point1->lat* DEG2RAD) * cos(point2->lat* DEG2RAD) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R*c; 
  }*/

  std::vector<Node*> Map::ExtractNodes(int start_node_id, int end_node_id){
    //TODO: verify
    auto start_node = nodes_.find(start_node_id);
    auto end_node = nodes_.find(end_node_id);
    double dist = GreatCircleDistance(start_node->second, end_node->second);
    int count_new_nodes = dist / 2.0;
    
    std::vector<Node*> new_nodes;
    
    //std::cout << "Count new nodes size: " << count_new_nodes << std::endl;
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
 
      // For current way, check if relevant for driving scenarios
      // Find its corresponding nodes
      // Interpolate nodes and assign new ids (graph_id)
      for (tag; tag; tag = tag->NextSiblingElement("tag")) {
        // A tag includes a key "k" and a value "v"
        std::string k(tag->Attribute("k"));
        std::string v(tag->Attribute("v"));
        if( k == "highway" && v == "unclassified" ||
            k == "highway" && v == "service" ||
            k == "highway" && v == "tertiary" ||
            k == "highway" && v == "residential") {
          //std::cout << "found way: " << way_id << std::endl;
          // Create new way struct
          Way* new_way = new Way;
          new_way->way_id = way_id;
          ways_.push_back(new_way);

          //TODO: iterate through nodes in way
          //TODO: interpolate node pairs and assign unique graph_id
          //TODO: intersert to way's way and push into vector<Way*>
          int start_node_id = 0;
          int end_node_id = 0;
          nd->Attribute("ref", &start_node_id);
          auto start_node = nodes_.find(start_node_id)->second;
          auto end_node = nodes_.find(start_node_id)->second;
          // Check if node already exists in navigation_nodes_
          auto check = navigation_nodes_.find(start_node->graph_id);
          if(way_id == 838919370){
            std::cout << "adding node: " << start_node_id << std::endl;
            std::cout << "graph_id: " << nodes_.find(start_node_id)->second->graph_id<< std::endl; 
            std::cout << "osm_id: " << nodes_.find(start_node_id)->second->osm_id<< std::endl;
            std::cout << "-----" << std::endl; 
          }
          if(check == navigation_nodes_.end()){
            start_node->graph_id = current_graph_id;
            navigation_nodes_.insert({current_graph_id, start_node});
            //std::cout << "verifying: " << current_graph_id << " ; " << navigation_nodes_.find(current_graph_id)->second->graph_id << std::endl; 
            //std::cout << "verifying2: " << nodes_.find(start_node->osm_id)->second->graph_id<< std::endl; 
            ++current_graph_id;
            //std::cout << "current_graph_id: " << current_graph_id << std::endl;
            //std::cout << "current_graph_id: " << current_graph_id << std::endl;
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
              //if(way_id == 838919370){
              //  std::cout << "graph_id(1) at node: " << end_node->graph_id << std::endl;
              //}
              navigation_nodes_.insert({current_graph_id, end_node});  
              ++current_graph_id;
            }
            //if(way_id == 838919370){
            //  std::cout << "end node graph_id: " << nodes_.find(end_node->osm_id)->second->graph_id<< std::endl; 
            //  std::cout << "graph_id(2) at node: " << end_node->graph_id << std::endl;
            //  std::cout << "end node osm_id: " << nodes_.find(end_node_id)->second->osm_id<< std::endl;
            //  std::cout << "-----" << std::endl; 
            //}
            new_way->nodes.push_back(end_node);
            nd = nd->NextSiblingElement("nd");
            start_node_id = end_node_id;   
          }
          break;  
        }
      } 

    } 
  }
  std::vector<Node*> Map::ShortestPath(Node* point1, Node* point2){
    //TODO
    std::cout << "Starting Dijkstra" << std::endl;
    std::vector<Node*> traj;
    std::stack<Node*> traj_stack = osm_graph.Dijkstra(point1, point2);
    //for(unsigned int i=0; i<traj_stack.size(); i++){
    while(!traj_stack.empty()){
      if(traj_stack.top()->osm_id != -1){ 
        std::cout << "Node: " << traj_stack.top()->osm_id << " | " << traj_stack.top()->graph_id << std::endl;
      }
      traj.push_back(traj_stack.top());
      traj_stack.pop();
    }
    return traj;
  }
}
