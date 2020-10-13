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
    
  }
  
  void Map::GenerateGraph(){
    TiXmlHandle node_handle = TiXmlHandle(osm_nodes_);
    TiXmlHandle way_handle = TiXmlHandle(osm_ways_);

    // Extract all nodes and interpolate (define 2 ids: osm_id; graph_id)
    // Extract ways and associate ways with nodes (interpolate nodes)
    //      - every new node has an incremental id assigned to it
    // 

  }
}
