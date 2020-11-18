/**
 * @file render_bev.cpp
 *
 * @brief Class for handling local bird's eye view maps using Open Street map data. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#include <gps_navigation/render_bev.h>
#include <gps_navigation/graph.h>
#include <gps_navigation/utils.h>

namespace gps_navigation{
  GpsBev::GpsBev(){};
  GpsBev::GpsBev(std::vector<Way*> road_network, double origin_lat, double origin_lon, double res, int road_thickness, int local_bev_dim){
 
    map_res_ = res;
    road_thickness_ = road_thickness;
    min_x_ = INFINITY;
    min_y_ = INFINITY;
    max_x_ = 0;
    max_y_ = 0;
    ref_start_ = new Node;
    ego_ = new EgoState;
    //ego_.pose = new Node;
    ref_start_->lat = origin_lat;
    ref_start_->lon = origin_lon;
    prev_index_ = 0;

    // Initialize default map
    cv::Mat default_map_(local_bev_dim, local_bev_dim, CV_8UC3, cv::Scalar(0,0,0));

    // Estimate dimensions of image
    for(unsigned int i=0; i<road_network.size(); i++){
      for(unsigned int j=0; j<road_network[i]->nodes.size(); j++){
        std::pair<double, double> dx_dy = RelativeDisplacement(ref_start_, road_network[i]->nodes[j]); 
        min_x_ = dx_dy.first < min_x_ ? dx_dy.first : min_x_; 
        max_x_ = dx_dy.first > max_x_ ? dx_dy.first : max_x_;
        min_y_ = dx_dy.second < min_y_ ? dx_dy.second : min_y_; 
        max_y_ = dx_dy.second > max_y_ ? dx_dy.second : max_y_;
      }
    }
    
    // Construct bev image
    total_x_ = abs(max_x_ - min_x_);
    total_y_ = abs(max_y_ - min_y_);
    x_origin_ = abs(min_x_);
    y_origin_ = abs(min_y_);
    pix_x_ = (total_x_ / map_res_);
    pix_y_ = (total_y_ / map_res_);
    osm_map_ = cv::Mat(pix_x_+1, pix_y_+1, CV_8UC3, cv::Scalar(0,0,0));
    
    // Populate roads 
    for(unsigned int i=0; i<road_network.size(); i++){
      for(unsigned int j=0; j<road_network[i]->nodes.size()-1; j++){
        std::pair<double, double> seg_start = RelativeDisplacement(ref_start_, road_network[i]->nodes[j]);
        std::pair<double, double> seg_end = RelativeDisplacement(ref_start_, road_network[i]->nodes[j+1]);

        unsigned int u_start = (unsigned int)((seg_start.first  + x_origin_) / map_res_); 
        unsigned int v_start = (unsigned int)((seg_start.second + y_origin_) / map_res_);
        unsigned int u_end = (unsigned int)((seg_end.first  + x_origin_) / map_res_); 
        unsigned int v_end = (unsigned int)((seg_end.second + y_origin_) / map_res_);
        
        cv::Point path_start(v_start, u_start), path_end(v_end, u_end);
        cv::Scalar color_line(255,255,255);
        cv::line(osm_map_, path_start, path_end, color_line, road_thickness_);
         
      }
    }
    
  }
  //double GpsBev::GetBearing(double lat, double lon){
  //  double y = sin(lon - ego_->pose.lon)*cos(lat);
  //  double x = cos(ego_->pose.lat)*sin(lat) -
  //             sin(ego_->pose.lat)*cos(lat)*cos(lon - ego_->pose.lon);
  //  double theta = atan2(y, x);
  //  return (int)(theta*180/M_PI + 360) % 360;
  //  //return theta*180/M_PI;
  //  //double y = sin(lon - ref_start_->lon)*cos(lat);
  //  //double x = cos(ref_start_->lat)*sin(lat) -
  //  //           sin(ref_start_->lat)*cos(lat)*cos(lon - ref_start_->lon);
  //  //double theta = atan2(y, x);
  //  //return (int)(theta*180/M_PI + 360) % 360;
  //}
  
  std::pair<cv::Mat, cv::Mat> GpsBev::RetrieveLocalBev(double x, double y, double yaw, int next_node, std::vector<Node*> plan, int region){

    unsigned int u_pose = (unsigned int)((x  + x_origin_) / map_res_); 
    unsigned int v_pose = (unsigned int)((y + y_origin_) / map_res_);
    //cv::Mat local_bev = osm_map_(cv::Range(u_pose-region, u_pose+region), cv::Range(v_pose-region, v_pose+region));
    //cv::Mat local_bev = osm_map_(cv::Range(v_pose-(region/2), u_pose-(region/2)), cv::Range(region, region));
    
    std::pair<cv::Mat, cv::Mat> osm_bevs;
    cv::Mat rot_unrouted_bev;
    cv::Mat rot_routed_bev;

    if(plan.size()){
      // Paint out previous plan
      if(prev_plan_.size() != 0){
        for(unsigned int i=prev_index_; i<prev_plan_.size()-1; i++){
          std::pair<double, double> seg_start = RelativeDisplacement(ref_start_, prev_plan_[i]);
          std::pair<double, double> seg_end = RelativeDisplacement(ref_start_, prev_plan_[i+1]);

          unsigned int u_start = (unsigned int)((seg_start.first  + x_origin_) / map_res_); 
          unsigned int v_start = (unsigned int)((seg_start.second + y_origin_) / map_res_);
          unsigned int u_end = (unsigned int)((seg_end.first  + x_origin_) / map_res_); 
          unsigned int v_end = (unsigned int)((seg_end.second + y_origin_) / map_res_);
          
          cv::Point path_start(v_start, u_start), path_end(v_end, u_end);
          cv::Scalar color_line(255,255,255);
          cv::line(osm_map_, path_start, path_end, color_line, road_thickness_);
           
        }
      } 
      // Extract unrouted bev 
      cv::Rect roi(v_pose-(region/2), u_pose-(region/2), region, region);
      cv::Mat unrouted_bev(osm_map_, roi);
      // Rotate unrouted bev
      cv::Point2f rot_point(unrouted_bev.cols/2.0, unrouted_bev.rows/2.0);
      cv::Mat r_so3 = cv::getRotationMatrix2D(rot_point, 180-(yaw*180/M_PI), 1.0);
      //cv::Mat r_so3 = cv::getRotationMatrix2D(rot_point, (-1.0*yaw*180/M_PI), 1.0);
      cv::warpAffine(unrouted_bev, rot_unrouted_bev, r_so3, cv::Size(unrouted_bev.cols, unrouted_bev.rows)); 
      
      // Paint new plan 
      for(unsigned int i=next_node; i<plan.size()-1; i++){
        std::pair<double, double> seg_start = RelativeDisplacement(ref_start_, plan[i]);
        std::pair<double, double> seg_end = RelativeDisplacement(ref_start_, plan[i+1]);

        unsigned int u_start = (unsigned int)((seg_start.first  + x_origin_) / map_res_); 
        unsigned int v_start = (unsigned int)((seg_start.second + y_origin_) / map_res_);
        unsigned int u_end = (unsigned int)((seg_end.first  + x_origin_) / map_res_); 
        unsigned int v_end = (unsigned int)((seg_end.second + y_origin_) / map_res_);
        
        cv::Point path_start(v_start, u_start), path_end(v_end, u_end);
        cv::Scalar color_line(0,255,0);
        cv::line(osm_map_, path_start, path_end, color_line, road_thickness_);
         
      }
      prev_index_ = next_node;
      prev_plan_ = plan;

      // Extract routed bev 
      cv::Mat routed_bev(osm_map_, roi);
      // Rotate routed bev
      cv::warpAffine(routed_bev, rot_routed_bev, r_so3, cv::Size(routed_bev.cols, routed_bev.rows)); 

    }
    osm_bevs.first = rot_unrouted_bev;
    osm_bevs.second = rot_routed_bev;
    return osm_bevs; 
  }

}
