#include <gps_navigation/utils.h>
#include <gps_navigation/graph.h>

namespace gps_navigation{

  OsmGraph::OsmGraph(){
  }
  /*
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
  }*/  
  void OsmGraph::Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> node_table){
    static int edge_counter = 0;
    for (unsigned int i=0; i<ways.size(); i++){
      for (unsigned int j=0; j<ways[i]->nodes.size()-1; j++){
        int start_node_id = ways[i]->nodes[j]->graph_id;
        int end_node_id = ways[i]->nodes[j+1]->graph_id;
        auto start_node = node_table.find(start_node_id);
        auto end_node = node_table.find(end_node_id);
        double dist = GreatCircleDistance(start_node->second, end_node->second);
        
        if(start_node->second == NULL || end_node->second == NULL){
          std::cout << "NOT FOUND-----------------" << std::endl;
        }
        // If node dne, add it and push end node to its vector of nodes 
        //if(start_node == node_table.end()){
        //  //TODO: remove this
        //  std::cout << "This should not execute" << std::endl;
        //  node_table.insert({ start_node_id, ways[i]->nodes[j]});
        //  start_node = node_table.find(start_node_id);
        //  start_node->second->edges->nodes.push_back(ways[i]->nodes[j+1]);
        //  start_node->second->edges->distances.push_back(dist);
        //  continue;
        //}
        // Sanity check in case start_node and end_node connection exists
        bool node_exists = false;
        if(start_node->second->edges != NULL){
          //if(start_node->second->edges
          for(unsigned int k=0;k<start_node->second->edges->nodes.size(); k++){
            if(start_node->second->edges->nodes[k]->graph_id == end_node_id){
              node_exists = true;
              break;
            } 
          }
        } 
        // If edge connection between start_node and end_node dne, create it 
        if(!node_exists){
          if(start_node->second->edges == NULL){
            start_node->second->edges = new Edge;
          }
          start_node->second->edges->nodes.push_back(end_node->second);
          start_node->second->edges->distances.push_back(dist);
        } 
        //if(ways[i]->way_id == 101920218 && start_node->second->osm_id != -1){
        if(start_node->second->osm_id == 1176652302){
          std::cout << "#########" << std::endl;
          std::cout << "node: " << start_node->second->osm_id << std::endl;
          if(start_node->second->edges == NULL){
            std::cout << " -> -1" << std::endl;
          }else{
            std::cout << " -> " << start_node->second->edges->nodes.size()<< std::endl;
          }
          
        }

      }
    }  
  }
  std::stack<Node*> OsmGraph::Dijkstra(Node* point1, Node* point2){
    std::stack<Node*> shortest_path;
    // To keep track of the total distance
    int current_dist = 0;
    Node* current_node;
    std::priority_queue<Node*, std::vector<Node*>, NodeComp> pq;
    
    // Find shortest path using Dijkstra's algorithm from point1 to point2 
    point1->dist = 0;
    pq.push(point1);
    while(!pq.empty()){
      //std::cout << "PQ size: " << pq.size() << std::endl;
      current_node = pq.top();
      pq.pop();
      current_dist = current_node->dist;
      
      // Check terminating condition: destination reached
      if(current_node->osm_id == point2->osm_id){
        
        std::cout << "Found shortest path to osm_id: " << current_node->osm_id << std::endl;
        // Traverse from point2 backwards using prev_node pointer
        int p=0;
        while(current_node != NULL){
          //if(current_node->osm_id != -1){
          //  std::cout << "Path (" << p << "): " << current_node->osm_id << std::endl;
          //  ++p;
          //}
          shortest_path.push(current_node);
          current_node = current_node->prev_node;
        }
        return shortest_path; 
      }
      
      // If we have not visited node popped yet
      if(!current_node->visited){
        // Mark node as visited
        current_node->visited = true;
        // Iterate through neighbors
        //std::cout << "1: " << current_node->osm_id << std::endl;
        auto adj_node_start = current_node->edges->nodes.begin(); 
        auto adj_node_end = current_node->edges->nodes.end();
        auto adj_weight_start = current_node->edges->distances.begin();
        auto adj_weight_end = current_node->edges->distances.end();
        
        double c;
        //std::cout << "2" << std::endl;
        while(adj_node_start != adj_node_end){
          c = current_dist + (*adj_weight_start);
          if(c < (*adj_node_start)->dist){
            // Set prev_node
            (*adj_node_start)->prev_node = current_node;
            // Update distance
            (*adj_node_start)->dist = c;
            pq.push(*adj_node_start);
          } 
          ++adj_node_start;
          ++adj_weight_start;
        }      

      }
       
    } 
    //return shortest_path;
  }
}
