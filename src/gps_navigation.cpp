#include <gps_navigation/gps_navigation.h>

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

    // Define OSM graph
    OsmGraph osm_graph = OsmGraph();
    
  }
  double Map::GreatCircleDistance(Node* point1, Node* point2){
    //TODO: verify
    double DEG2RAD = M_PI / 180;
    double R = 6371e3;
    double dLat = point2->lat*DEG2RAD - point1->lon*DEG2RAD;
    double dLon = point2->lon*DEG2RAD - point1->lon*DEG2RAD;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(point1->lat* DEG2RAD) * cos(point2->lat* DEG2RAD) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R*c; 
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
    static unsigned int current_graph_id = 0;
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
          //TODO: iterate through nodes in way
          //TODO: interpolate node pairs and assign unique graph_id
          //TODO: intersert to way's way and push into vector<Way*>
          int start_node_id = 0;
          int end_node_id = 0;
          while(nd->NextSiblingElement("nd")) {
            nd->Attribute("ref", &start_node_id);
            nd->NextSiblingElement("nd")->Attribute("ref", &end_node_id); 
            nd = nd->NextSiblingElement("nd");
            
            // Interpolate nodes
            // Tag all nodes with new graph_id and insert into ways and nodes 
            std::vector<Node*> interpolated_nodes = ExtractNodes(start_node_id, end_node_id); 
             
          }  
        }
      } 

    } 

  }
}
