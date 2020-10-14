#ifndef OSM_GRAPH
#define OSM_GRAPH
#include <vector>
#include <unordered_map>

namespace gps_navigation{
  struct Node;
  struct Edge;

  struct Node{
    int osm_id;
    int graph_id;
    float lat;
    float lon;
    Edge* edges;

  };
  
  struct Edge{
    std::vector<Node*> nodes; 
  };

  struct Way{
    std::vector<Node*> nodes;
  };

  class OsmGraph{
    public:
      OsmGraph();
      void Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> nodes);
      
      // Hashtables that represent graph
      //      node_table: maps a node to Node* that saves information about its edges
      std::unordered_map<int, Node*> node_table;

  };
}
#endif
