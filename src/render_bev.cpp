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
    ego_->pose = new Node;
    ref_start_->lat = origin_lat;
    ref_start_->lon = origin_lon;

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
  double GpsBev::GetBearing(double lat, double lon){
    double y = sin(lon - ego_->pose->lon)*cos(lat);
    double x = cos(ego_->pose->lat)*sin(lat) -
               sin(ego_->pose->lat)*cos(lat)*cos(lon - ego_->pose->lon);
    double theta = atan2(y, x);
    return (int)(theta*180/M_PI + 360) % 360;
    //return theta*180/M_PI;
    //double y = sin(lon - ref_start_->lon)*cos(lat);
    //double x = cos(ref_start_->lat)*sin(lat) -
    //           sin(ref_start_->lat)*cos(lat)*cos(lon - ref_start_->lon);
    //double theta = atan2(y, x);
    //return (int)(theta*180/M_PI + 360) % 360;
  }
  cv::Mat GpsBev::RetrieveLocalBev(double lat, double lon, double v, double a_x, double a_y, double w_z, double dt, std::vector<Node*> plan, int region){
    ego_->a_x = a_x; 
    ego_->a_y = a_y;
    ego_->w_z = w_z;
    //ego_->v_x = a_x * dt; 
    //ego_->v_y = a_y * dt;
    if(!ego_->pose_init || !ego_->bearing_init){
      double current_bearing = GetBearing(lat, lon);
      std::cout << "Bearing: " << current_bearing << std::endl;
      if(abs(current_bearing - ego_->prev_bearing) < 5.0){
        ego_->bearing_init= true; 
      }
      ego_->pose->lat = lat;
      ego_->pose->lon = lon;
      ego_->pose_init = true; 
      ego_->bearing = current_bearing;
      return default_map_; 
    }
    //if(ego_->pose_init && ego_->bearing_init){
    //    std::cout << "init success" << std::endl;
    //}
    // If there is a new gps measurement 
    if(lat != ego_->pose->lat){
      double current_bearing = GetBearing(lat, lon);
      std::cout << "Bearing: " << current_bearing << std::endl;
      std::pair<double, double> ego_pose = RelativeDisplacement(ref_start_, ego_->pose);
      // Update bearing as long as it does not drastically fluctuate
      if(abs(current_bearing - ego_->prev_bearing) < 5.0){
        //std::cout << "---Good Bearing: " << current_bearing << std::endl;
        ego_->gps_is_valid = true;
        ego_->bearing = current_bearing;
      }
      else{
        ego_->gps_is_valid = false;
      } 
      ego_->prev_bearing = current_bearing;
      ego_->x = ego_pose.first;
      ego_->y = ego_pose.second;
    } 
    if(!ego_->gps_is_valid || (lat == ego_->pose->lat)){
      //std::cout << "a_x: "<< a_x << " a_y: " << a_y << std::endl;
      //std::cout << "w_z: "<< w_z << std::endl;
      //std::cout << "dt: "<< dt << std::endl;
      // use IMU to update position
      ego_->v_y = v*sin(ego_->w_z * dt);
      ego_->v_x = v*cos(ego_->w_z * dt);
      //std::cout << "vx: "<< ego_->v_x << " vy: " << ego_->v_y << std::endl;
      double dx =  ego_->v_x * dt + 0.5*a_x*dt*dt; 
      double dy = ego_->v_y * dt + 0.5*a_y*dt*dt; 
      ego_->x += ego_->v_x * dt + 0.5*a_x*dt*dt; 
      ego_->y += ego_->v_y * dt + 0.5*a_y*dt*dt;
      // TODO: v_x and v_y are not correct 
      //std::cout << "dx: "<< dx << " dy: " << dy << std::endl;
      //std::cout << "v_x: "<< ego_->v_x << " v_y: " << ego_->v_y << std::endl;
      
    }
    ego_->pose->lat = lat;
    ego_->pose->lon = lon;
    unsigned int u_pose = (unsigned int)((ego_->x  + x_origin_) / map_res_); 
    unsigned int v_pose = (unsigned int)((ego_->y + y_origin_) / map_res_);
    //cv::Mat local_bev = osm_map_(cv::Range(u_pose-region, u_pose+region), cv::Range(v_pose-region, v_pose+region));
    //cv::Mat local_bev = osm_map_(cv::Range(v_pose-(region/2), u_pose-(region/2)), cv::Range(region, region));
    
    if(plan.size()){
      // Paint out previous plan
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
