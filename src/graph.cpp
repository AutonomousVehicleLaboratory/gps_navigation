#include <gps_navigation/utils.h>
#include <gps_navigation/graph.h>

namespace gps_navigation{

  OsmGraph::OsmGraph(){
  }

  void OsmGraph::Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> node_table){
    static int edge_counter = 0;
    for (unsigned int i=0; i<ways.size(); i++){
      for (unsigned int j=0; j<ways[i]->nodes.size()-1; j++){
        int start_node_id = ways[i]->nodes[j]->graph_id;
        int end_node_id = ways[i]->nodes[j+1]->graph_id;
        auto start_node = node_table.find(start_node_id);
        auto end_node = node_table.find(end_node_id);
        // Calculate the distance between nodes
        double dist = GreatCircleDistance(start_node->second, end_node->second);
        // Estimate dx, dy for second node 
        std::pair<double, double> dx_dy = RelativeDisplacement(start_node->second, end_node->second);
        double dx_dy_norm = sqrt(dx_dy.first*dx_dy.first + dx_dy.second*dx_dy.second);
        dx_dy.first /= dx_dy_norm; 
        dx_dy.second /= dx_dy_norm;
        end_node->second->dx_dy = dx_dy; 
        
        if(start_node->second == NULL || end_node->second == NULL){
          std::cout << "NOT FOUND-----------------" << std::endl;
        }
        // Perform start_node->end_node edge connection 
        bool node_exists = false;
        if(start_node->second->edges != NULL){
          for(unsigned int k=0;k<start_node->second->edges->nodes.size(); k++){
            if(start_node->second->edges->nodes[k]->graph_id == end_node_id){
              node_exists = true;
              break;
            } 
          }
        }
        if(!node_exists){
          if(start_node->second->edges == NULL){
            start_node->second->edges = new Edge;
          }
          start_node->second->edges->nodes.push_back(end_node->second);
          start_node->second->edges->distances.push_back(dist);
        }
        // Perform end_node->start_node edge connection if applicable 
        if(!ways[i]->one_way){
          node_exists = false;
          if(end_node->second->edges != NULL){
            for(unsigned int k=0;k<end_node->second->edges->nodes.size(); k++){
              if(end_node->second->edges->nodes[k]->graph_id == start_node_id){
                node_exists = true;
                break;
              } 
            }
          }
          if(!node_exists){
            if(end_node->second->edges == NULL){
              end_node->second->edges = new Edge;
            }
            end_node->second->edges->nodes.push_back(start_node->second);
            end_node->second->edges->distances.push_back(dist);
          }
        }
        
      }
    }  
  }
  void OsmGraph::ResetGraph(std::unordered_map<int, Node*> node_table){
    auto node_start = node_table.begin();  
    auto node_end = node_table.end();
    while(node_start != node_end){
       node_start->second->prev_node = NULL;
       node_start->second->visited = false;
       node_start->second->dist = INFINITY;
      
      ++node_start;
    } 
  }
  std::stack<Node*> OsmGraph::Dijkstra(Node* point1, Node* point2){
    std::stack<Node*> shortest_path;
    // To keep track of the total distance
    int current_dist = 0;
    Node* current_node;
    std::priority_queue<Node*, std::vector<Node*>, NodeComp> pq;
    //std::cout << "Node1 graph_id: " << point1->graph_id << std::endl;
    //std::cout << "Node2 graph_id: " << point2->graph_id << std::endl;
    
    // Find shortest path using Dijkstra's algorithm from point1 to point2 
    point1->dist = 0;
    pq.push(point1);
    while(!pq.empty()){
      //std::cout << "PQ size: " << pq.size() << std::endl;
      current_node = pq.top();
      pq.pop();
      current_dist = current_node->dist;
      
      // Check terminating condition: destination reached
      if(current_node->graph_id == point2->graph_id){
        
        //std::cout << "Found shortest path to osm_id: " << current_node->graph_id << std::endl;
        // Traverse from point2 backwards using prev_node pointer
        int p=0;
        while(current_node != NULL){
          //std::cout << "iter" << std::endl;
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
        //std::cout << "1"<< std::endl;
        current_node->visited = true;
        // Iterate through neighbors
        //std::cout << "1: " << current_node->osm_id << std::endl;
        if(current_node->edges == NULL){
          continue;
        } 
        auto adj_node_start = current_node->edges->nodes.begin(); 
        auto adj_node_end = current_node->edges->nodes.end();
        auto adj_weight_start = current_node->edges->distances.begin();
        auto adj_weight_end = current_node->edges->distances.end();
        //std::cout << "2"<< std::endl;
        
        double c;
        while(adj_node_start != adj_node_end){
          if(!(*adj_node_start)->visited){

            c = current_dist + (*adj_weight_start);
            if(c < (*adj_node_start)->dist){
              // Set prev_node
              (*adj_node_start)->prev_node = current_node;
              // Update distance
              (*adj_node_start)->dist = c;
              pq.push(*adj_node_start);
            } 
          }
          ++adj_node_start;
          ++adj_weight_start;
        }      

      }
       
    }
    std::cout << "Graph disconnected"<< std::endl; 
    //return shortest_path;
  }
}
