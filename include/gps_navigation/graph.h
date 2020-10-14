#ifndef OSM_GRAPH
#define OSM_GRAPH
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <tf/transform_datatypes.h>
namespace gps_navigation{
  struct Node;
  struct Edge;

  struct Node{
    long osm_id;
    long graph_id = -1;
    float lat;
    float lon;
    Edge* edges;

    /* For graph search */
    double dist = 1000000000; // TODO: distance from source: init to INFTY
    Node* prev_node = NULL; //node that last updated dist
    bool visited = false;
  };
  
  struct Edge{
    std::vector<Node*> nodes;
    std::vector<double> distances; 
  };

  struct Way{
    std::vector<Node*> nodes;
  };

  class NodeComp {
    public:
        bool operator()(Node*& left, Node*& right) const
        {
            return left->dist > right->dist;
        }
    };
  class OsmGraph{
    public:
      OsmGraph();
      double GreatCircleDistance(Node* point1, Node* point2);
      void Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> nodes);
      std::stack<Node*> Dijkstra(Node* point1, Node* point2);

      // Hashtables that represent graph
      //      node_table: maps a node to Node* that saves information about its edges
      std::unordered_map<int, Node*> node_table;

  };
}
#endif
