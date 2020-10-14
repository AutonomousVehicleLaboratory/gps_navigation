#ifndef GPSMAP
#define GPSMAP
#include <sstream>
#include <tinyxml.h>
#include <vector>
#include <stack>
#include <gps_navigation/graph.h>
#include <tf/transform_datatypes.h>
//#include <math.h>
#define MAX_THRESH 8.0

namespace gps_navigation{
  class Map{
    public:
      Map(std::string map_path);
      TiXmlDocument xml_map_;
      TiXmlNode *osm_map_;
      TiXmlNode *osm_bounds_;
      TiXmlNode *osm_nodes_;
      TiXmlNode *osm_ways_;
      bool osm_status_;
      std::vector<Way*> ways_;
      std::unordered_map<int, Node*> nodes_;
      std::unordered_map<int, Node*> navigation_nodes_;
      
 
      /* Initializes nodes/ways*/
      void ParseMap();
      std::vector<Node*> ShortestPath(Node* point1, Node* point2);
      
    private:
      OsmGraph osm_graph;
      double GreatCircleDistance(Node* point1, Node* point2); 
      std::vector<Node*> ExtractNodes(int start_node_id, int end_node_id);
  };
}
#endif 
