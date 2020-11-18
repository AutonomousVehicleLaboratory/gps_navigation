/**
 * @file render_bev.h
 *
 * @brief Header file for handling local bird's eye view maps using Open Street map data. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#include <gps_navigation/graph.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <gps_navigation/gps_navigation.h>

namespace gps_navigation{
  /*
  struct EgoState;
  struct EgoState{
    // Measurements from IMU
    double a_x, a_y, w_z;
    
    // Position/Bearing estimated from GPS
    double x,y, bearing, prev_bearing;
    Node* pose;
    
    // Velocities integrated from IMU
    double v_x, v_y;
    bool pose_init = false;
    bool bearing_init = false;
    bool gps_is_valid = false;
  }; 
  */
  class GpsBev{
    public:
      
      cv::Mat osm_map_;
      cv::Mat default_map_;
      Node* ref_start_;
      Node* pose_;
      
      EgoState* ego_;      

      double min_x_;
      double min_y_;
      double max_x_;
      double max_y_;
      double total_x_;
      double total_y_;
      double x_origin_;
      double y_origin_;
      double map_res_;
      unsigned int pix_x_;
      unsigned int pix_y_;
      int road_thickness_;
      std::vector<Node*> prev_plan_;
      int prev_index_;
      GpsBev();
      GpsBev(std::vector<Way*> road_network, double origin_lat, double origin_lon, double res, int road_thickness, int local_bev_dim);
      //double GetBearing(double lat, double lon);
      std::pair<cv::Mat, cv::Mat> RetrieveLocalBev(double x, double y, double yaw, int next_node, std::vector<Node*> plan, int region); 
  };


}
