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
  
  enum class NodeType { kStopSign, kTrafficSignal, kCrossing, kRoad, kOther }; 
  enum class WayType { kPlanned, kTraversed, kFootPath, kConstruction }; 
  
  struct Node;
  struct Edge;
  struct Way;

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

    // For traversal
    bool explored = false;

    // Node attributes
    NodeType key_attribute;
    std::vector<std::pair<std::string, std::string>> attributes;

    // Other Associated Ways
    std::vector<Way*> associated_ways;
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
      // Elements within a radius k
      std::vector<Node*> stopsigns_;
      std::vector<Node*> crossings_;
      std::vector<Node*> traffic_signals_; 
      std::vector<Node*> foot_paths_;
      std::vector<Node*> roads_;
      std::vector<Node*> construction_;

      std::vector<Node*> explored_;

      void Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> node_table);
      void ResetGraph(std::unordered_map<int, Node*> node_table);
      std::stack<Node*> Dijkstra(Node* point1, Node* point2);

      // Extracts k nearest neighbors given a point using BFS 
      // [pose | traversed | planned ]
      void FindRoadFeatures(Node* point, int k);

      std::vector<Node*> RetrieveStops();
      std::vector<Node*> RetrieveCrossings();
      std::vector<Node*> RetrieveTrafficSignals();

  };
}
#endif
