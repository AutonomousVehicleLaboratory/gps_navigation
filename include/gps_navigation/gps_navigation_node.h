/**
 * @file gps_navigation_node.h
 *
 * @brief ROS node header file for gps based path planning/local bev generation. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#ifndef GPS_NAV_NODE
#define GPS_NAV_NODE
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/graph.h>
#include <gps_navigation/utils.h>
#include <gps_navigation/render_bev.h>
#include <gps_navigation/SpatioTemporalGraph.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>


namespace gps_navigation{
  class GpsNavigationNode{
    public:
      GpsNavigationNode();
      void ClickedPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
      void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
      void SpeedCallback(const pacmod_msgs::VehicleSpeedRpt::ConstPtr& msg);
      void PublishGpsMap();

      SpatioTemporalGraph ParseGraph(std::vector<Node*> stops,
                                     std::vector<Node*> crossings,
                                     std::vector<Node*> signals,
                                     std::vector<Way*> footpaths,
                                     std::vector<Node*> traversed_path,
                                     std::vector<Node*> planned_path,
                                     std::vector<Node*> local_network);

      visualization_msgs::Marker GetMarker(int marker_type, long id, ros::Time time, double x, double y, double z, double yaw);
      
      visualization_msgs::MarkerArray VisMarkersFromNodes(std::vector<Node*> nodes, int marker_type, float color_r, float color_g, float color_b);
      
      visualization_msgs::Marker VisMarkersFromNodes2(std::vector<Node*> nodes, int marker_type, float color_r, float color_g, float color_b);

      
      nav_msgs::Path VisualizePath(std::vector<Node*> path);
      void VisualizeFootPaths(std::vector<Way*> ways);
      nav_msgs::Path VisualizeNetwork();

      
      // Publishers   
      ros::NodeHandle n;
      ros::Publisher shortest_path_viz;
      ros::Publisher road_network_viz;
      ros::Publisher traversed_path_viz;
      ros::Publisher planned_path_viz;
      ros::Publisher footpaths_viz;
      ros::Publisher node_orientation_viz;
      ros::Publisher gps_viz_pub;
      ros::Publisher gps_closest_viz_pub;
      ros::Publisher unrouted_bev_pub;
      ros::Publisher routed_bev_pub;
      ros::Publisher conc_bev_pub;

      ros::Publisher graph_pub;

      // Graph Based visualization
      ros::Publisher g_stops_pub;
      ros::Publisher g_crossings_pub;
      ros::Publisher g_signals_pub;
      ros::Publisher g_constructions_pub;
      ros::Publisher g_network_pub;
      
  
      // Subscribers
      ros::Subscriber gps_pose_sub;
      ros::Subscriber speed_sub;
      ros::Subscriber imu_sub;
      ros::Subscriber clicked_point;
  
      // OSM Map
      //Map* osm_map;
      Navigation* gps_navigator;
      GpsBev* osm_bev;
      nav_msgs::Path road_networks;
      cv::Mat local_osm_bev;
      Node* point1_shortest = new Node;
      Node* point2_shortest = new Node;
      std::vector<Node*> plan;
       
   
      // Variables    
      Node* ref_start = new Node;
      Node* gps_pose = new Node;
      int gps_ego_counter = 0;
      bool gps_avail = false;
      bool imu_avail = false;
      bool new_plan = false;
      bool new_gps_msg = false;
      bool has_clicked_point = false;
      double lat_pose = 0.0;
      double lon_pose = 0.0;
      double x_dest = 0.0;
      double y_dest = 0.0;
      sensor_msgs::Imu twist;
      double ego_speed = 0.0;
      double replan_threshold = 0.0;
      bool dt_avail = false;
      ros::Duration dt;
      ros::Time current_t;
      double bearing;
      int bfs_horizon;

      // Updated upon IMU callback
      std::tuple<bool, long, double, double, double> ego_state;

  
  };
}
#endif
