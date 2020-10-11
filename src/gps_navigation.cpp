#include <gps_navigation/gps_navigation.h>

namespace GpsNavigation{
  Map::Map(std::string map_path){
    osm_map = TiXmlDocument(map_path);
    osm_status = osm_map.LoadFile();
    
  }
}
