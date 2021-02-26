/**
 * @file graph.h
 *
 * @brief Graph search header file for using Dijkstra. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#ifndef OSM_GRAPH
#define OSM_GRAPH
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
//#include <math.h>
#include <tf/transform_datatypes.h>
namespace gps_navigation{
  
  enum class NodeType { kStopSign, kTrafficSignal, kCrossing, kRoadElement, kOther }; 
  enum class WayType { kRoad, kFootPath, kConstruction }; 
  
  struct Node;
  struct Edge;

  struct Node{
    long osm_id = -1;
    long graph_id = -1;
    double lat;
    double lon;
    Edge* edges = NULL;

    // Orientation
    std::pair<double, double> dx_dy = std::make_pair(0.0, 0.0);

    // For graph search 
    double dist = INFINITY; 
    Node* prev_node = NULL; //node that last updated dist
    bool visited = false;

    // Node attributes
    NodeType key_attribute;
    std::vector<std::pair<std::string, std::string>> attributes;
  };
  
  struct Edge{
    std::vector<Node*> nodes;
    std::vector<double> distances; 
  };

  struct Way{
    std::vector<Node*> nodes;
    int way_id = -1;
    bool one_way = false;

    // Way Attributes
    WayType key_attribute;
    std::vector<std::pair<std::string, std::string>> attributes;
    
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
      void Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> node_table);
      void ResetGraph(std::unordered_map<int, Node*> node_table);
      std::stack<Node*> Dijkstra(Node* point1, Node* point2);


  };
}
#endif
