#ifndef GPSMAP
#define GPSMAP
#include <sstream>
#include <tinyxml.h>
#include <vector>
#include <stack>
#include <math.h>
#include <gps_navigation/graph.h>
#include <tf/transform_datatypes.h>
//#include <math.h>
#define MAX_THRESH 8.0

namespace gps_navigation{
  class Map{
    public:
      Map();
      Map(std::string map_path);
 
      /* Initializes nodes/ways*/
      void ParseMap();
      Node* FindClosestNode(double lat, double lon);
      Node* FindClosestNodeRelative(double x, double y, double origin_x, double origin_y);
      std::vector<Node*> ShortestPath(Node* point1, Node* point2);
      std::vector<Way*> GetWays();
      //std::vector<Node*> GetNavigationNodes();
      void ResetPlan();
      
    private:
      TiXmlDocument xml_map_;
      TiXmlNode *osm_map_;
      TiXmlNode *osm_bounds_;
      TiXmlNode *osm_nodes_;
      TiXmlNode *osm_ways_;
      bool osm_status_;
      std::vector<Way*> ways_;
      std::unordered_map<int, Node*> nodes_;
      std::unordered_map<int, Node*> navigation_nodes_;
      OsmGraph osm_graph_;

      std::vector<Node*> ExtractNodes(int start_node_id, int end_node_id);
  };
  
  class Navigation{
    public:
      Navigation();

    
  }; 
}
#endif 
