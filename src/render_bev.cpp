#include <gps_navigation/render_bev.h>
#include <gps_navigation/graph.h>
#include <gps_navigation/utils.h>

namespace gps_navigation{
  GpsBev::GpsBev(std::vector<Way*> road_network, double origin_lat, double origin_lon, double res, int road_thickness){
 
    map_res_ = res;
    road_thickness_ = road_thickness;
    min_x_ = INFINITY;
    min_y_ = INFINITY;
    max_x_ = 0;
    max_y_ = 0;
    ref_start_ = new Node;
    pose_ = new Node;
    ref_start_->lat = origin_lat;
    ref_start_->lon = origin_lon;

    

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
  cv::Mat GpsBev::RetrieveLocalBev(double lat, double lon, std::vector<Node*> plan, int region){
    pose_->lat = lat;
    pose_->lon = lon;
    std::pair<double, double> ego_pose = RelativeDisplacement(ref_start_, pose_);
    
    unsigned int u_pose = (unsigned int)((ego_pose.first  + x_origin_) / map_res_); 
    unsigned int v_pose = (unsigned int)((ego_pose.second + y_origin_) / map_res_);
    //cv::Mat local_bev = osm_map_(cv::Range(u_pose-region, u_pose+region), cv::Range(v_pose-region, v_pose+region));
    //cv::Mat local_bev = osm_map_(cv::Range(v_pose-(region/2), u_pose-(region/2)), cv::Range(region, region));
    
    if(plan.size()){
      // Paint over previous plan
      if(prev_plan_.size() != 0){
        for(unsigned int i=0; i<prev_plan_.size()-1; i++){
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
      
      // Paint new plan 
      std::cout << "painting new plan" << std::endl;
      for(unsigned int i=0; i<plan.size()-1; i++){
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
      prev_plan_ = plan;

    }
    cv::Rect roi(v_pose-(region/2), u_pose-(region/2), region, region);
    cv::Mat local_bev(osm_map_, roi);

    return local_bev; 
  } 

}
