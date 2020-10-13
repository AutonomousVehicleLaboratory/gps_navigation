#include <gps_navigation/graph.h>

namespace gps_navigation{

  OsmGraph::OsmGraph(){
  }
  
  void OsmGraph::Generate(std::vector<Way*> ways, std::vector<Node*> nodes){
    static int edge_counter = 0;
    for (unsigned int i=0; i<ways.size(); i++){
      for (unsigned int j=0; j<ways[i]->nodes.size()-1; j++){
        int start_node_id = ways[i]->nodes[j]->graph_id;
        int end_node_id = ways[i]->nodes[j+1]->graph_id;
        auto start_node = node_table.find(start_node_id);
        auto end_node = node_table.find(end_node_id);
        
        // If node dne, add it and push end node to its vector of nodes 
        if(start_node == node_table.end()){
          node_table.insert({ start_node_id, ways[i]->nodes[j]});
          start_node = node_table.find(start_node_id);
          start_node->second->edges->nodes.push_back(ways[i]->nodes[j+1]);
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
        } 

      }
    }  
  }
}
