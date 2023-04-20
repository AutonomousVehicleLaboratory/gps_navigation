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
  
  enum class NodeType { kOther, kTrafficSignal, kStopSign, kCrossing, kRoad }; 
  enum class WayType { kOther, kRoad, kPlanned, kTraversed, kFootPath, kConstruction }; 
  
  struct Node;
  struct Edge;
  struct Way;
  // struct NewNodeType{
  //   vector<NodeType> types;
  //   void operator =(NodeType aType){
  //     types.push_back(aType);
  //   }
  //   void operator =(NewNodeType aType){
  //     types = aType.types;
  //   }
  //   // some other member utility functions
  //   bool find(NodeType aType){
  //     return std::find(types.begin(), types.end(), aType) != types.end();
  //   }
  // }

  struct Node{
    long osm_id = -1;
    long graph_id = -1;
    double lat;
    double lon;

    // Edges based on traversability/connectivity 
    Edge* edges = NULL;

    // Edges for constructions
    Edge* constructionEdges= NULL;

    // Edges to nearby elements 
    Edge* implicit_edges = NULL;
    

    // Orientation
    std::pair<double, double> dx_dy = std::make_pair(0.0, 0.0);

    // For graph search 
    double dist = INFINITY; 
    Node* prev_node = NULL; //node that last updated dist
    bool visited = false;

    // For traversal
    bool explored = false;

    // Node attributes
    NodeType key_attribute = NodeType::kOther;
    std::vector<std::pair<std::string, std::string>> attributes;

    // Associated footpaths for the current node
    std::vector<Way*> associated_ways;
    std::vector<Node*> associated_construction_Nodes;
  public:
    bool isConnectedToConstructionNodes();
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
      std::vector<Node*> nearby_stopsigns_;
      std::vector<Node*> nearby_crossings_;
      std::vector<Node*> nearby_traffic_signals_; 
      std::vector<Way*> nearby_footpaths_;
      std::vector<Node*> nearby_construction_;

      std::vector<Node*> explored_;

      // Generates connections for main roads
      void Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> node_table);
  
      // Generates connections between ways and existing node (road) elements (nearest connection assumed)
      void ConnectWays(std::vector<Way*> elements, std::unordered_map<int, Node*> node_table);

      void ConnectConstructionWays(std::vector<Way*>& constructions, std::unordered_map<int, Node*>& node_table, double radius);

      void BuildConstructionEdges(std::vector<Way*> ways);
      void ConnectConstructionEdge(Node* start_node, Node* end_node, double weight);


      std::vector<Node*> FindNodesWithinRadius(Node* aNode, std::unordered_map<int, Node*>& node_table, double radius);

      // Generates connections between ways and existing node (road) elements (by closest distance) 
      void ConnectImplicitWays(std::vector<Way*> elements, std::unordered_map<int, Node*> node_table);
  

      void ResetGraph(std::unordered_map<int, Node*> node_table);
      std::stack<Node*> Dijkstra(Node* point1, Node* point2);

      // Extracts k nearest neighbors given a point using BFS 
      // [pose | traversed | planned ]
      void BFS(Node* point, int k);

      std::vector<Node*> RetrieveStops();
      std::vector<Node*> RetrieveCrossings();
      std::vector<Node*> RetrieveTrafficSignals();
      std::vector<Way*> RetrieveFootPaths();
      std::vector<Node*> RetrieveConstructions();
      std::vector<Node*> RetrieveExplored();

    protected:
      void ConnectEdge(std::pair<int, Node*> startNodePair, std::pair<int, Node*> endNodePair, double weight);
      void ConnectImplicitEdge(std::pair<int, Node*> startNodePair, std::pair<int, Node*> endNodePair, double weight);


  };
}
#endif
