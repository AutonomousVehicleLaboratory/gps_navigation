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

    // Position estimated by IMU+GPS
    //double x_ego, y_ego, yaw_ego, prev_yaw_ego;
    double x_ego, y_ego, next_yaw, current_yaw, sim_yaw;

    // Velocities integrated from IMU
    double v, v_x, v_y;
    bool pose_init = false;
    bool bearing_init = false;
    bool gps_is_valid = false;
  };
  class Map{
    public:
      Map();
      Map(std::string map_path);
 
      /* Initializes nodes/ways*/
      void ParseMap();
      Node* FindClosestNode(double lat, double lon);
      Node* FindClosestNodeRelative(double x, double y, double origin_x, double origin_y);
      std::vector<Node*> ShortestPath(Node* point1, Node* point2);
      std::vector<Way*> GetWays();
      //std::vector<Node*> GetNavigationNodes();
      void ResetPlan();
      NNGraph nn_graph_;
      
    private:
      TiXmlDocument xml_map_;
      TiXmlNode *osm_map_;
      TiXmlNode *osm_bounds_;
      TiXmlNode *osm_nodes_;
      TiXmlNode *osm_ways_;
      bool osm_status_;
      std::vector<Way*> ways_;
      std::unordered_map<int, Node*> nodes_;
      std::unordered_map<int, Node*> navigation_nodes_;
      OsmGraph osm_graph_;

      std::vector<Node*> ExtractNodes(int start_node_id, int end_node_id);
  };
  
  class Navigation{
    public:
      Navigation();
      Navigation(std::string map_path, double x_origin, double y_origin);
      Map* GetMap();
      void SetStart(double lat, double lon);
      void SetTarget(double lat, double lon);
      void SetTargetRelative(double x_dest, double y_dest);
      void ResetPlan();
      std::vector<Node*> Plan();
      std::pair<int, Node*> FindClosestPlannedNode();
      std::tuple<bool, long, double, double, double> UpdateState(double lat, double lon, double v, double w_z, double a_x, double t);

    private:
      Map* osm_map_;
      EgoState state_;
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
