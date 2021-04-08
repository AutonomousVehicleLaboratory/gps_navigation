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
  bool Node::isConnectedToConstructionNodes(){
    return associated_construction_Nodes.size()>0;
  }

  void OsmGraph::Generate(std::vector<Way*> ways, std::unordered_map<int, Node*> node_table){
    //static int edge_counter = 0;
    for (unsigned int i=0; i<ways.size(); i++){
      for (unsigned int j=0; j<ways[i]->nodes.size()-1; j++){
        int start_node_id = ways[i]->nodes[j]->graph_id;
        int end_node_id = ways[i]->nodes[j+1]->graph_id;
        auto start_node = node_table.find(start_node_id)->second;
        auto end_node = node_table.find(end_node_id)->second;
        // Calculate the distance between nodes
        double dist = GreatCircleDistance(start_node, end_node);
        // Estimate dx, dy for second node 
        std::pair<double, double> dx_dy = RelativeDisplacement(start_node, end_node);
        double dx_dy_norm = sqrt(dx_dy.first*dx_dy.first + dx_dy.second*dx_dy.second);
        dx_dy.first /= dx_dy_norm; 
        dx_dy.second /= dx_dy_norm;
        end_node->dx_dy = dx_dy; 
        
        if(start_node == NULL || end_node == NULL){
          std::cout << "NOT FOUND-----------------" << std::endl;
        }
        // Connect start_node->end_node if it doesn't exist before
        ConnectEdge(std::make_pair(start_node_id, start_node), std::make_pair(end_node_id, end_node), dist);
        // Perform end_node->start_node edge connection if applicable 
        if(!ways[i]->one_way){
          ConnectEdge(std::make_pair(end_node_id, end_node), std::make_pair(start_node_id, start_node), dist);
        }
        else{
          // TODO: verify
          // Perform an implicit end_node->start_node edge connection (used for finding road elements) 
          ConnectImplicitEdge(std::make_pair(end_node_id, end_node), std::make_pair(start_node_id, start_node), dist);
        }
        
      }
    }  
  }
  void OsmGraph::ConnectEdge(std::pair<int, Node*> startNodePair, std::pair<int, Node*> endNodePair, double weight){
    //auto start_node_id = startNodePair.first;
    auto start_node = startNodePair.second;
    auto end_node_id = endNodePair.first;
    auto end_node = endNodePair.second;

    bool node_exists = false;
    if(start_node->edges != NULL){
      auto totalEdges = start_node->edges->nodes.size();
      for(unsigned int k=0;k<totalEdges; k++){
        if(start_node->edges->nodes[k]->graph_id == end_node_id){
          node_exists = true;
          break;
        } 
      }
    }
    if(!node_exists){
      if(start_node->edges == NULL){
        start_node->edges = new Edge;
      }
      start_node->edges->nodes.push_back(end_node);
      start_node->edges->distances.push_back(weight);
    }
  }

  void OsmGraph::BuildConstructionEdges(std::vector<Way*> ways){
    //static int edge_counter = 0;
    for (unsigned int i=0; i<ways.size(); i++){
      for (unsigned int j=0; j<ways[i]->nodes.size()-1; j++){
        auto start_node= ways[i]->nodes[j];
        auto end_node = ways[i]->nodes[j+1];
        // Calculate the distance between nodes
        double dist = GreatCircleDistance(start_node, end_node);
        // Estimate dx, dy for second node 
        std::pair<double, double> dx_dy = RelativeDisplacement(start_node, end_node);
        double dx_dy_norm = sqrt(dx_dy.first*dx_dy.first + dx_dy.second*dx_dy.second);
        dx_dy.first /= dx_dy_norm; 
        dx_dy.second /= dx_dy_norm;
        end_node->dx_dy = dx_dy; 
        
        if(start_node == NULL || end_node == NULL){
          std::cout << "NOT FOUND-----------------" << std::endl;
        }
        ConnectConstructionEdge(start_node, end_node, dist);
        ConnectConstructionEdge(end_node, start_node, dist);
      }
    }  
  }
  void OsmGraph::ConnectConstructionEdge(Node* start_node, Node* end_node, double weight){
    bool node_exists = false;
    if(start_node->constructionEdges != NULL){
      auto totalEdges = start_node->constructionEdges->nodes.size();
      for(unsigned int k=0;k<totalEdges; k++){
        if(start_node->constructionEdges->nodes[k] == end_node){
          node_exists = true;
          break;
        } 
      }
    }
    if(!node_exists){
      if(start_node->constructionEdges == NULL){
        start_node->constructionEdges = new Edge;
      }
      std::cout<<"Build an ConnectConstructionEdge"<<std::endl;
      start_node->constructionEdges->nodes.push_back(end_node);
      start_node->constructionEdges->distances.push_back(weight);
    }
  }


  void OsmGraph::ConnectImplicitEdge(std::pair<int, Node*> startNodePair, std::pair<int, Node*> endNodePair, double weight){
    //auto start_node_id = startNodePair.first;
    auto start_node = startNodePair.second;
    auto end_node_id = endNodePair.first;
    auto end_node = endNodePair.second;
    bool node_exists = false;
    if(start_node->implicit_edges != NULL){
      auto totalEdges = start_node->implicit_edges->nodes.size();
      for(unsigned int k=0;k<totalEdges; k++){
        if(start_node->implicit_edges->nodes[k]->graph_id == end_node_id){
          node_exists = true;
          break;
        } 
      }
    }
    if(!node_exists){
      if(start_node->implicit_edges == NULL){
        start_node->implicit_edges = new Edge;
      }
      start_node->implicit_edges->nodes.push_back(end_node);
      start_node->implicit_edges->distances.push_back(weight);
    }
  }

  void OsmGraph::ConnectWays(std::vector<Way*> elements, std::unordered_map<int, Node*> node_table){
    for(auto way: elements){
      for(auto node: way->nodes){
        // graph_id are generated for interpolated nodes but also assigned to
        // original osm nodes 
        
        auto check = node_table.find(node->graph_id);  
        if(check != node_table.end()){
          check->second->associated_ways.push_back(way);
        }
      }
    }

    return;  
  }
  void OsmGraph::ConnectConstructionWays(std::vector<Way*>& constructions, std::unordered_map<int, Node*>& node_table, double radius){
    // for every navigation nodes, find the set of construction nodes associated with it
    double dist;
    std::cout<<"Total construction Nodes: "<<constructions.size()<<"\n";
    std::unordered_map<int, Node*> cnode_table;
    for(auto constructionWay: constructions){
      for(auto constructionNode: constructionWay->nodes){
        
        //   for(auto nodePair: node_table){
        //     auto theNode = nodePair.second;
        //     dist = GreatCircleDistance(theNode, constructionNode);
        //     if(dist<radius){
        //       theNode->associated_construction_Nodes.push_back(constructionNode);
        //       //std::cout<<"Distance: "<<dist<<std::endl;
        //     }

        //   }
        cnode_table.insert(std::make_pair(0, constructionNode));
      }
    }
    // Generates a NNGraph for the construction nodes only
    // Also will iterate through node table and populate all the construction associated nodes
    auto CNodeGraph = NNGraph();
    CNodeGraph.Generate(cnode_table);
    for (auto nodePair: node_table) {
        auto theNode = nodePair.second;
        CNodeGraph.WithinRadius(theNode, radius);
    }

  }

  std::vector<Node*> OsmGraph::FindNodesWithinRadius(Node* aNode, std::unordered_map<int, Node*>& node_table, double radius){

    return {};
  }


  void ConnectImplicitWays(std::vector<Way*> elements, std::unordered_map<int, Node*> node_table){

    for(auto way: elements){
      //TODO
      for(auto node: way->nodes){
        // graph_id are generated for interpolated nodes but also assigned to
        // original osm nodes 
        //TODO: find node within node_table that is closest to node
        //check->second->associated_ways.push_back(way);
      }
    }

    return;  

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
    
    // Find shortest path using Dijkstra's algorithm from point1 to point2 
    point1->dist = 0;
    pq.push(point1);
    while(!pq.empty()){
      current_node = pq.top();
      pq.pop();
      current_dist = current_node->dist;
      
      // Check terminating condition: destination reached
      if(current_node->graph_id == point2->graph_id){
        
        // Traverse from point2 backwards using prev_node pointer
        //int p=0;
        while(current_node != NULL){

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
        if(current_node->edges == NULL){
          continue;
        } 
        auto adj_node_start = current_node->edges->nodes.begin(); 
        auto adj_node_end = current_node->edges->nodes.end();
        auto adj_weight_start = current_node->edges->distances.begin();
        //auto adj_weight_end = current_node->edges->distances.end();
        
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
    return shortest_path;
  }
  void OsmGraph::BFS(Node* point, int k){

    // Given a node point, find the k nearest neighbors 
    std::queue<Node*> search_q; 
    
    if(!point) return;

    
    Node* curr_node;

    // Counter to keep track of radius
    int curr_k = 0;

    // clean explored nodes
    if(!explored_.empty()){
      for(auto node: explored_){
        node->explored = false;
      }
      nearby_stopsigns_ = {};
      nearby_crossings_ = {};
      nearby_traffic_signals_ = {};
      nearby_footpaths_ = {};
      nearby_construction_ = {};
    }
    std::map<NodeType, std::string> logMaps = \
          {{NodeType::kStopSign, "kStopSign"},{NodeType::kCrossing, "kCrossing"},{NodeType::kTrafficSignal, "kTrafficSignal"}, {NodeType::kOther, "kOther"}, {NodeType::kRoad, "kRoad"}};

    search_q.push(point);

    while(curr_k != k){
      
      std::queue<Node*> curr_q; 

      while(!search_q.empty()){
        curr_node = search_q.front();
        search_q.pop();
        if(curr_node->explored) continue;
        //std::cout<<"current Node Type: "<<logMaps[curr_node->key_attribute]<<"\n";
      
        // Mark as explored to avoid cycles
        curr_node->explored = true;
        explored_.push_back(curr_node);

        // separate node types by:
        // nearby_stopsigns_
        if(curr_node->key_attribute == NodeType::kStopSign){
          nearby_stopsigns_.push_back(curr_node);
        }
        // nearby_crossings_ 
        if(curr_node->key_attribute == NodeType::kCrossing){
          nearby_crossings_.push_back(curr_node);
        }
        
        // nearby_traffic_signals_
        if(curr_node->key_attribute == NodeType::kTrafficSignal){
          nearby_traffic_signals_.push_back(curr_node);
        }

        //if(curr_node->associated_ways.size() > 0) std::cout  << "FOUND WAY\n";

        // nearby_footpaths_
        for(auto curr_way: curr_node->associated_ways){
          // nearby_footpaths_
          if(curr_way->key_attribute == WayType::kFootPath){
            nearby_footpaths_.push_back(curr_way);
          }

          // TODO: search for nearby_construction_
        }
        for(auto theConstructionNode: curr_node->associated_construction_Nodes){
          if(nearby_construction_.size()==0 || std::find(nearby_construction_.begin(), nearby_construction_.end(), theConstructionNode)==nearby_construction_.end()){
            // std::cout<<"Constructions: "<<curr_node->lat<<" "<<curr_node->lon<<std::endl;
            nearby_construction_.push_back(theConstructionNode);
          }
        }
        // if(nearby_construction_.size()!=0){
        //   std::cout<<"total size: "<<nearby_construction_.size()<<std::endl;
        // }

        // Find neighbors of type Node
        if(curr_node->edges){
          for(auto neighbor: curr_node->edges->nodes){
            if(neighbor->explored) continue;

            curr_q.push(neighbor);          
          } 
        }

        if(curr_node->implicit_edges){
          for(auto neighbor: curr_node->implicit_edges->nodes){
            if(neighbor->explored) continue;

            curr_q.push(neighbor);          
          } 
        }
      }
      search_q = curr_q;
      curr_k += 1;
    }
    
    return;


  }
  std::vector<Node*> OsmGraph::RetrieveStops(){
    return nearby_stopsigns_;
  }

  std::vector<Node*> OsmGraph::RetrieveCrossings(){
    return nearby_crossings_;
  }

  std::vector<Node*> OsmGraph::RetrieveTrafficSignals(){
    return nearby_traffic_signals_;
  }

  std::vector<Way*> OsmGraph::RetrieveFootPaths(){
    return nearby_footpaths_;
  } 
  std::vector<Node*> OsmGraph::RetrieveConstructions(){
    return nearby_construction_;
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
  // Recursive, returns nearest neighbor in the tree (start with lat_level = true)
  // O(2^d + log(n)) where d is depth of tree
  void NNGraph::RadiusSearch(KDNode* root, KDNode* ego_location, double radius, bool lat_level) {
    // Base case, no such node on this part of the tree exists
    if(!root) {
      return;
    }
    // Base case, reached leaf
    if(!root->left && !root->right) {
      //std::cout << " Reached leaf" << std::endl;
      double dist = GreatCircleDistance(root->osm_node, ego_location->osm_node);
      if(dist < radius) {
        ego_location->osm_node->associated_construction_Nodes.push_back(root->osm_node);
      }
      return; 
    }
    //std::cout << " Reached the Nearest Neighbor Algorithm" << std::endl;
    if(lat_level) { // Latitude split
      if(lesserKDNodeLat(ego_location, root)) { // Left
        RadiusSearch(root->left, ego_location, radius, !lat_level);
        if(ego_location->osm_node->lat + radius >= root->split) { // Right tree contains points within threshold
          RadiusSearch(root->right, ego_location, radius, !lat_level);
        }
      } else { // Right
        RadiusSearch(root->right, ego_location, radius, !lat_level);
        if(ego_location->osm_node->lat - radius <= root->split) { // Left tree contains points within threshold
          RadiusSearch(root->left, ego_location, radius, !lat_level);
        }    
      }
    } else { // Longitude split
      if(lesserKDNodeLon(ego_location, root)) { // Left
        RadiusSearch(root->left, ego_location, radius, !lat_level);
        if(ego_location->osm_node->lon + radius >= root->split) { // Right tree contains points within threshold
          RadiusSearch(root->right, ego_location, radius, !lat_level);
        }
      } else { // Right
        RadiusSearch(root->right, ego_location, radius, !lat_level);
        if(ego_location->osm_node->lon - radius <= root->split) { // Left tree contains points within threshold
          RadiusSearch(root->left, ego_location, radius, !lat_level);
        }    
      }
    }
    double dist = GreatCircleDistance(root->osm_node, ego_location->osm_node);
    if(dist < radius) { // Parent node is within threshold case
      ego_location->osm_node->associated_construction_Nodes.push_back(root->osm_node);
    }
  }
  // Radius search for a given KD tree
  void NNGraph::WithinRadius(Node* query, double radius) {
    // Populate the query associated_construction_nodes with nodes within the specified radius
    this->RadiusSearch(this->root, new KDNode(query), radius, true);

  }
  // ROS Node facing function: Calls given the query gps pose
  Node* NNGraph::KDNearest(Node* ego_location) {
    this->best_dist = INFINITY;
    this->best = NULL;
    this->NearestNeighbor(this->root, new KDNode(ego_location), true);
    return this->best->osm_node; 
  }
  
}
