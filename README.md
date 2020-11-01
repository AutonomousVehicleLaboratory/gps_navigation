# gps_navigation
This ROS node inludes C++ implementations for extracting OpenStreetMaps(OSM), perform graph search using latitude/longitude or 2D relative map coordinates and renders ego-centric local bird's eye view representations for the planned trajectory using the state of the vehicle.

  * *gps_navigation_node.cpp*: ROS node takes in as input 2D navigation goal set by RViz, ego vehicle gps location and computes shortest path between these two points in they exist.
  * *gps_navitation.cpp*: Core functionality for graph search, local bird'eye view map generation and estimating state of the vehicle.
  
  * *graph.cpp*: Handles graph search and OSM parsing.
  
  * *render_bev.cpp*: Generates local bird's eye view map given the state of the ego vehicle and a planned trajectory.
  
  * *utils.cpp*: Methods for measuring relative distance using lat/lon and the relative map frame.
  * *nmea_to_navstat.py*: Converts NMEA sentences into longitude and latitude.
