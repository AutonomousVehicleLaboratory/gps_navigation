#ifndef GPSMAP
#define GPSMAP
#include <sstream>
#include <tinyxml.h>
#define MAX_THRESH 8.0

namespace GpsNavigation{
  class Map{
    public:
      Map(std::string map_path);
      TiXmlDocument osm_map;
      bool osm_status;
      
      /* Graph functions */ 
  };
}
#endif 
