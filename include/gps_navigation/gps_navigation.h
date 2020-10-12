#ifndef GPSMAP
#define GPSMAP
#include <sstream>
#include <tinyxml.h>
#define MAX_THRESH 8.0

namespace gps_navigation{
  class Map{
    public:
      Map(std::string map_path);
      void GenerateGraph();
      TiXmlDocument xml_map_;
      TiXmlNode *osm_map_;
      TiXmlNode *osm_bounds_;
      TiXmlNode *osm_nodes_;
      TiXmlNode *osm_ways_;
      bool osm_status_;
      
      /* Graph functions */ 
  };
}
#endif 
