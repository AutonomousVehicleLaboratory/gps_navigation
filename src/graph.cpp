/**
 * @file graph.cpp
 *
 * @brief Graph search implementation using Dijkstra. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */
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
  bool lesserKDNodeLat(KDNode* n1, KDNode* n2) { 
    if(n1->osm_node->lat == n2->osm_node->lat) {
      return n1->osm_node->lon < n2->osm_node->lon;
    }
    return n1->osm_node->lat < n2->osm_node->lat;
  }
  bool lesserKDNodeLon(KDNode* n1, KDNode* n2) {
    if(n1->osm_node->lon == n2->osm_node->lon) {
      return n1->osm_node->lat < n2->osm_node->lat;
    }
    return n1->osm_node->lon < n2->osm_node->lon;
  }
  
  // No ties for indices
  bool ltLatInd(KDNode* n1, KDNode* n2) {
    return n1->lat_ind < n2->lat_ind;
  }
  bool ltLonInd(KDNode* n1, KDNode* n2) {
    return n1->lon_ind < n2->lon_ind;
  }
  // Finds the median of an unsorted list of KDNode indexed tuples based on the level to be searched recursively
  // Linear median of medians algorithm
  KDNode* linearMedian(std::vector<KDNode*> unsorted, bool lat_level) {
    //std::cout << unsorted.size() << std::endl;
    if(unsorted.size() == 1) { // Base case, we are at the end of this recursion
      return unsorted[0];
    }
    if(!lat_level) { // Longitude case, we can just return the middle of the array since the input array is guaranteed to be sorted by longitude
      return unsorted[unsorted.size()>>1];
    }
    std::vector<KDNode*> intermediate;
    
    // Split it into length segments of 5, find their medians manually, recurse 
    for(unsigned int i = 0; i < unsorted.size()/5; i++) { // Could leave a segment at the end of length < 5
      //if(lat_level) {
        std::sort(unsorted.begin()+(5*i), unsorted.begin()+(5*i+5), ltLatInd);
        intermediate.push_back(unsorted[5*i+2]);
      /*} else {
        std::sort(unsorted.begin()+(5*i), unsorted.begin()+(5*i+5), ltLonInd);  
        intermediate.push_back(unsorted[5*i+2]);
      }*/
    }
    // Deal with the remaining segment of < 5 here and return that median as well to intermediate
    if(5*intermediate.size() < unsorted.size()) {
      unsigned int lastseglength = unsorted.size() - (5*intermediate.size());
      //if(lat_level) {
        std::sort(unsorted.begin()+intermediate.size(), unsorted.end(), ltLatInd);
      /*} else {
        std::sort(unsorted.begin()+intermediate.size(), unsorted.end(), ltLonInd);
      }*/
      intermediate.push_back(unsorted[5*intermediate.size()+lastseglength>>1]);
    }
    KDNode* result = linearMedian(intermediate, lat_level);
    //std::cout << result->lat_ind << std::endl;
    return result;
  }
  // Contstructor, doesn't actually do much besides instantiate
  NNGraph::NNGraph(){
  }
  // Unused, not necessary for now
  void NNGraph::Insert(Node* osm_node) {
    // Make top level lat, second level lon
  }
  
  // O(knlogn) algorithm to build KD tree (k sorts, nlogn building algorithm)
  KDNode* NNGraph::Partition(std::vector<KDNode*> children, bool lat_level) {
    // Empty children list
    if(!children.size()) {
      return NULL;
    }
    // Given a list of nodes, find the midpoint of children, recurse into left, right and return midpoint for assignment
    KDNode* median = linearMedian(children, lat_level);
    //std::cout << children.size() << std::endl;
    // TODO: Build left and right children arrays
    std::vector<KDNode*> lt;
    std::vector<KDNode*> gt;
    for(unsigned int i = 0; i < children.size(); i++) {
      if(median == children[i]) { // Exclude median from child arrays
        continue;
      }
      if(lat_level) { // Latitude level
        if(ltLatInd(median, children[i])) { // Add to right side
          gt.push_back(children[i]);
        } else { // Add to left side
          lt.push_back(children[i]);
          //std::cout << children[i]->lat_ind << std::endl;
        }
      } else { // Longitude level
        if(ltLonInd(median, children[i])) { // Add to right side
          gt.push_back(children[i]);
        } else { // Add to left side
          lt.push_back(children[i]);
        }
      }
    }
    // Recurse given our median node
    median->left = Partition(lt, !lat_level);
    median->right = Partition(gt, !lat_level);
    median->split = lat_level ? median->osm_node->lat : median->osm_node->lon;
    return median;
  }
  void NNGraph::Generate(std::unordered_map<int, Node*> node_table) {
    // Utilize OSM Graph implementation (Wrap nodes in a K-D structure before adding), iterate through table
    std::vector<KDNode*> node_array;
    
    for (auto it : node_table) {
      //std::cout << it.second << std::endl;
      node_array.push_back(new KDNode(it.second));
    }
    std::cout << " Created KD Node Array" << std::endl;
    // Populate lat and lon integer indices to make sorting easier
    std::sort(node_array.begin(), node_array.end(), lesserKDNodeLat);
    //std::cout << node_array.size() << std::endl;
    for (unsigned int i = 0; i < node_array.size(); i++) {
      node_array[i]->lat_ind = i; // Find issue here (lat/lon_inds look way off, skewing KD development)
    }
    std::sort(node_array.begin(), node_array.end(), lesserKDNodeLon);
    for (unsigned int i = 0; i < node_array.size(); i++) {
      node_array[i]->lon_ind = i;
    }
    // Make it as balanced as possible
    // Potential point of improvement: make the lon median calculations O(1) vs O(n) since the passed list is sorted by lon
    this->root = Partition(node_array, true);
    
  }
  
  // Recursive, returns nearest neighbor in the tree (start with lat_level = true)
  // O(2^d + log(n)) where d is depth of tree
  void NNGraph::NearestNeighbor(KDNode* root, KDNode* ego_location, bool lat_level) {
    // Base case, no such node on this part of the tree exists
    if(!root) {
      return;
    }
    // Base case, reached leaf
    if(!root->left && !root->right) {
      //std::cout << " Reached leaf" << std::endl;
      double dist = GreatCircleDistance(root->osm_node, ego_location->osm_node);
      if(dist < this->best_dist) {
        this->best_dist = dist;
        this->best = root;
      }
      return; 
    }
    //std::cout << " Reached the Nearest Neighbor Algorithm" << std::endl;
    if(lat_level) { // Latitude split
      if(lesserKDNodeLat(ego_location, root)) { // Left
        NearestNeighbor(root->left, ego_location, !lat_level);
        if(ego_location->osm_node->lat + this->best_dist >= root->split) { // Right tree could be better
          NearestNeighbor(root->right, ego_location, !lat_level);
        }
      } else { // Right
        NearestNeighbor(root->right, ego_location, !lat_level);
        if(ego_location->osm_node->lat - this->best_dist <= root->split) { // Left tree could be better
          NearestNeighbor(root->left, ego_location, !lat_level);
        }    
      }
    } else { // Longitude split
      if(lesserKDNodeLon(ego_location, root)) { // Left
        NearestNeighbor(root->left, ego_location, !lat_level);
        if(ego_location->osm_node->lon + this->best_dist >= root->split) { // Right tree could be better
          NearestNeighbor(root->right, ego_location, !lat_level);
        }
      } else { // Right
        NearestNeighbor(root->right, ego_location, !lat_level);
        if(ego_location->osm_node->lon - this->best_dist <= root->split) { // Left tree could be better
          NearestNeighbor(root->left, ego_location, !lat_level);
        }    
      }
    }
    double dist = GreatCircleDistance(root->osm_node, ego_location->osm_node);
    if(dist < this->best_dist) { // Parent node is better case
      this->best_dist = dist;
      this->best = root;
    }
  }
  // ROS Node facing function: Calls given the query gps pose
  Node* NNGraph::KDNearest(Node* ego_location) {
    this->best_dist = INFINITY;
    this->best = NULL;
    this->NearestNeighbor(this->root, new KDNode(ego_location), true);
    return this->best->osm_node; 
  }
  
}
