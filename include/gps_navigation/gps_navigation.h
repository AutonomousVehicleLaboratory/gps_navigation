/**
 * @file gps_navigation.h
 *
 * @brief Header file for handling graph search and ego vehicle states. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#ifndef GPSMAP
#define GPSMAP
#include <sstream>
#include <tinyxml.h>
#include <vector>
#include <stack>
#include <algorithm>
//#include <math.h>
#include <gps_navigation/graph.h>
#include <tf/transform_datatypes.h>
//#include <math.h>
#define MAX_THRESH 8.0

namespace gps_navigation{
  struct EgoState;
  struct EgoState{
    // Measurements from IMU
    double a_x, a_y, w_z;
    
    // Position estimated from GPS
    Node pose;
    
    // Closest node on graph
    Node matched_pose;

    // Position estimated by IMU+GPS
    //double x_ego, y_ego, yaw_ego, prev_yaw_ego;
    double x_ego, y_ego, next_yaw, current_yaw, sim_yaw;

    // Velocities integrated from Odom/IMU
    double v, v_x, v_y;
    bool pose_init = false;
    bool bearing_init = false;
    bool gps_is_valid = false;
  };
  class Map{
    public:
      Map();
      Map(std::string map_path, double radius);

      // For assigning new node IDs after interpolation
      int current_graph_id_ = 0;

      // Parses relevant nodes 
      void ParseNode(TiXmlElement *node_element);

      // Parses relevant ways 
      void ParseWay(TiXmlElement *way_element);
 
      // Initializes nodes/ways
      void ParseMap();
      // Find the closest node within the graph to the given lat/lon coordinate 
      Node* FindClosestNode(double lat, double lon);

      // Find the closest node within the graph to the given (x, y) relative coordinate wrt
      // a lat/lon coordinate (origin_x, origin_y) 
      Node* FindClosestNodeRelative(double x, double y, double origin_x, double origin_y);

      // Estimate shortest path between point1 and point2
      std::vector<Node*> ShortestPath(Node* point1, Node* point2);

      // Wrapper for OsmGraph road feature search 
      void FindRoadFeatures(Node* point, int k);

      // Wrapper for returning OsmGraph stopsigns 
      std::vector<Node*> GetStops();

      // Wrapper for returning OsmGraph crossings 
      std::vector<Node*> GetCrossings();

      // Wrapper for returning OsmGraph traffic signals  
      std::vector<Node*> GetTrafficSignals();

      // Wrapper for returning OsmGraph footpaths 
      std::vector<Way*> GetFootPaths();

      std::vector<Node*> GetConstrustions();
      

      // Return ways 
      std::vector<Way*> GetWays();

      void ResetPlan();
      
    private:
      TiXmlDocument xml_map_;
      TiXmlNode *osm_map_;
      TiXmlNode *osm_bounds_;
      TiXmlNode *osm_nodes_;
      TiXmlNode *osm_ways_;
      bool osm_status_;

      // Stores road networks: k=highway; v=unclassified/service/tertiary/residential
      // (ways are composed of interpolated nodes)
      std::vector<Way*> ways_;
      

      // Stores all ways of type 'kFootPath' 
      std::vector<Way*> footpaths_;

      // Stores all ways of type 'kConstruction' 
      std::vector<Way*> construction_;

      //std::unordered_map<int, Node*> traffic_signal_nodes_;

      // Stores all remaining and unique node definitions
      std::unordered_map<int, Node*> nodes_;

      // Stores interpolated nodes that are only part of road networks
      std::unordered_map<int, Node*> navigation_nodes_;
      OsmGraph osm_graph_;

      std::vector<Node*> ExtractNodes(int start_node_id, int end_node_id);
  };
  
  class Navigation{
    public:
      Navigation();
      Navigation(std::string map_path, double x_origin, double y_origin, double radius);

      // Getter for Map object//
      Map* GetMap();

      // For planning, set origin */
      void SetStart(double lat, double lon);

      // For planning, set destination as (lat,lon)
      void SetTarget(double lat, double lon);

      // For planning, set destination as relative (x,y) coordinate
      // assuming a relative displacement wrt (x_origin,y_origin)
      void SetTargetRelative(double x_dest, double y_dest);

      // Prepares graph for another iteration of graph search*/
      void ResetPlan();

      // Estimate shortest path and return trajectory*/
      std::vector<Node*> Plan();
      
      // Find node within planned trajectory closest to vehicle pose 
      std::pair<unsigned int, Node*> FindClosestPlannedNode();
      
      // 
      // Updates state of ego vehicle using GNSS and odom only
      // returns a graph around vehicle
      //std::vector<Node*> GenerateSTGraph(double lat, double lon, double v, double t);
      void GenerateSTGraph(double lat, double lon, double v, double t);

      // Updates state of ego vehicle as reported by GNSS, odom and IMU 
      std::tuple<bool, long, double, double, double> UpdateState(double lat, double lon, double v, double w_z, double a_x, double t);

      // Returns only traversed nodes, last node corresponds to ego pose
      std::vector<Node*> GetTraversedNodes(); 

      // Returns only planned nodes
      std::vector<Node*> GetPlannedNodes(); 

      


      
    private:
      Map* osm_map_;
      EgoState state_;
      EgoState g_state_;
      Node* start_node_;
      Node* end_node_;
      //Node* current_node_;
      Node* nearest_gps_node_;
      Node* ref_origin_;
      std::vector<Node*> current_plan_;
      double lat_origin_;
      double lon_origin_;
      double t_prev_ = -1.0;
      bool new_gps_ = false; 
      bool use_sim_yaw_ = false; 
      double sim_distance_ = false; 

      // Index of node that comes after 
      unsigned long next_node_index_;
      double x_next_, y_next_;
      bool use_gps_ = true;
      
      // If any 2 nodes between current_plan_[current_index]->current_plan_[current_index+k]
      // generate an angle >= thresh, return true 
      bool CheckNextAngles(unsigned long k, double thresh);
    
  }; 
}
#endif 
