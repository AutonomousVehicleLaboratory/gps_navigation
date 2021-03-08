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
  };
  
  struct Edge{
    std::vector<Node*> nodes;
    std::vector<double> distances; 
  };

  struct Way{
    std::vector<Node*> nodes;
    int way_id = -1;
    bool one_way = false;
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
  // Node structure for the KD Tree. 
  // Split value is just the value it has as the median for this layer
  // lat ind and lon ind are the position it would have been in an array 
  // sorted by lat primary and lon secondary or vice versa.
  class KDNode {
    public:
      KDNode(Node * osm_node) {
        this->osm_node = osm_node;
        this->left = NULL;
        this->right = NULL;
      }
      Node * osm_node;
      KDNode * left;
      KDNode * right;
      double split;
      unsigned int lat_ind; // Latitude tuple
      unsigned int lon_ind; // Longitude tuple
  };
  // Comparator functions (compares the ind variables or just direct latitude and longitude variables)
  bool lesserKDNodeLat(KDNode* n1, KDNode* n2);
  bool lesserKDNodeLon(KDNode* n1, KDNode* n2);
  bool ltLatInd(KDNode* n1, KDNode* n2);
  bool ltLonInd(KDNode* n1, KDNode* n2);
  // Obtains the median of unsorted and returns it.
  KDNode* linearMedian(std::vector<KDNode*> unsorted, bool lat_level);
  // KD Tree structure
  // root is the root of the tree, best/best dist is for NN search
  class NNGraph {
    public:
      NNGraph();
      KDNode* root = NULL;
      KDNode* best = NULL;
      double best_dist = INFINITY;
      // Creates a tree
      void Generate(std::unordered_map<int, Node*> node_table);
      // Unused for now, unimplemented as maps won't change during a run
      void Insert(Node* osm_node);
      // Recursive function used by Generate to help create the tree
      KDNode* Partition(std::vector<KDNode*> children, bool lat_level);
      // Recursive function called by KDNearest to help get the nearest neighbor
      void NearestNeighbor(KDNode* root, KDNode* ego_location, bool lat_level);
      // Outward facing function for nearest neighbor (likely the node will directly call this and generate)
      Node* KDNearest (Node* ego_location);
  };
}
#endif
