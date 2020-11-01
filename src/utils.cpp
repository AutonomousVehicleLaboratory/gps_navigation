/**
 * @file utils.cpp
 *
 * @brief Latitude and Longitude measurements and conversions. 
 *
 * @author David Paz 
 * Contact: dpazruiz@ucsd.edu 
 *
 */

#include <gps_navigation/utils.h>

double gps_navigation::GreatCircleDistance(Node* point1, Node* point2){
  double d_lat = point2->lat*kDegRadsConversion - point1->lat*kDegRadsConversion;
  double d_lon = point2->lon*kDegRadsConversion - point1->lon*kDegRadsConversion;
  double a = sin(d_lat / 2) * sin(d_lat / 2) +
             cos(point1->lat* kDegRadsConversion) * cos(point2->lat* kDegRadsConversion) *
             sin(d_lon / 2) * sin(d_lon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return kREarth*c;
}

std::pair<double, double> gps_navigation::RelativeDisplacement(Node* point1, Node* point2){
  std::pair<double, double> dx_dy;
  double d_lat = point2->lat*kDegRadsConversion - point1->lat*kDegRadsConversion;
  double d_lon = point2->lon*kDegRadsConversion - point1->lon*kDegRadsConversion;
  double lat_average = (point1->lat+ point2->lat) / 2;
  double a_x = cos(lat_average * kDegRadsConversion) * cos(lat_average * kDegRadsConversion) *
             sin(d_lon / 2) * sin(d_lon / 2);
  double dx = kREarth * 2 * atan2(sqrt(a_x), sqrt(1 - a_x));
  
  double a_y = sin(d_lat / 2) * sin(d_lat / 2);
  double d_y = kREarth * 2 * atan2(sqrt(a_y), sqrt(1 - a_y)); 
  dx_dy.first = point1->lon < point2->lon ? dx: -dx; 
  dx_dy.second = point1->lat < point2->lat ? d_y: -d_y;
  return dx_dy; 
}
