#include <gps_navigation/graph.h>

namespace gps_navigation{

  OsmGraph::OsmGraph(){
  }

  double OsmGraph::GreatCircleDistance(Node* point1, Node* point2){
    //TODO: verify
    double DEG2RAD = M_PI / 180;
    double R = 6371e3;
    double dLat = point2->lat*DEG2RAD - point1->lat*DEG2RAD;
    double dLon = point2->lon*DEG2RAD - point1->lon*DEG2RAD;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(point1->lat* DEG2RAD) * cos(point2->lat* DEG2RAD) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R*c;
  }  
  void OsmGraph::Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> nodes){
    static int edge_counter = 0;
    for (unsigned int i=0; i<ways.size(); i++){
      for (unsigned int j=0; j<ways[i]->nodes.size()-1; j++){
        int start_node_id = ways[i]->nodes[j]->graph_id;
        int end_node_id = ways[i]->nodes[j+1]->graph_id;
        auto start_node = node_table.find(start_node_id);
        auto end_node = node_table.find(end_node_id);
        double dist = GreatCircleDistance(start_node->second, end_node->second);
        
        // If node dne, add it and push end node to its vector of nodes 
        if(start_node == node_table.end()){
          node_table.insert({ start_node_id, ways[i]->nodes[j]});
          start_node = node_table.find(start_node_id);
          start_node->second->edges->nodes.push_back(ways[i]->nodes[j+1]);
          start_node->second->edges->distances.push_back(dist);
          continue;
        }
        // Sanity check in case start_node and end_node connection exists
        bool connection_exists = false;
        for(unsigned int k=0;k<start_node->second->edges->nodes.size(); k++){
          if(start_node->second->edges->nodes[k]->graph_id == end_node_id){
            connection_exists = true;
            break;
          } 
        }
        // If edge connection between start_node and end_node dne, create it 
        if(!connection_exists){
          start_node->second->edges->nodes.push_back(end_node->second);
          start_node->second->edges->distances.push_back(dist);
        } 

      }
    }  
  }
  std::vector<Node*> OsmGraph::Dijkstra(Node* point1, Node* point2){
    std::vector<Node*> shortest_path;
    // To keep track of the total distance
    int length = 0;
    Node* current_node;
    std::priority_queue<Node*, std::vector<Node*>, NodeComp> pq;
 
    return shortest_path;
  }
}
