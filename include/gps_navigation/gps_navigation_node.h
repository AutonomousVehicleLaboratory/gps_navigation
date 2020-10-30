#ifndef GPS_NAV_NODE
#define GPS_NAV_NODE
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <gps_navigation/gps_navigation.h>
#include <gps_navigation/graph.h>
#include <gps_navigation/utils.h>
#include <gps_navigation/render_bev.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>


namespace gps_navigation{
  class GpsNavigationNode{
    public:
      GpsNavigationNode();
      void ClickedPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
      void ImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
      void SpeedCallback(const std_msgs::Float64::ConstPtr& msg);
      visualization_msgs::Marker GetMarker(int marker_type, long id, ros::Time time, double x, double y, double z, double yaw);
      nav_msgs::Path VisualizePath(std::vector<Node*> path);
      nav_msgs::Path VisualizeNetwork();
      
      // Publishers and Subscribers   
      ros::NodeHandle n;
      ros::Publisher shortest_path_viz;
      ros::Publisher road_network_viz;
      ros::Publisher node_orientation_viz;
      ros::Publisher gps_viz_pub;
      ros::Publisher gps_closest_viz_pub;
      ros::Publisher gps_bev_pub;
  
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
      bool dt_avail = false;
      ros::Duration dt;
      ros::Time current_t;
      double bearing;


  
  };
}
#endif
