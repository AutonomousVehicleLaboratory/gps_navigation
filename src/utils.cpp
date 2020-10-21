#include <gps_navigation/utils.h>

double gps_navigation::GreatCircleDistance(Node* point1, Node* point2){
  //TODO: verify
  //std::cout << "P1: " << point1->lat << " ; " << point1->lon << std::endl; 
  //std::cout << "P2: " << point2->lat << " ; " << point2->lon << std::endl; 
  double DEG2RAD = M_PI / 180;
  double R = 6371e3;
  double dLat = point2->lat*DEG2RAD - point1->lat*DEG2RAD;
  double dLon = point2->lon*DEG2RAD - point1->lon*DEG2RAD;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(point1->lat* DEG2RAD) * cos(point2->lat* DEG2RAD) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R*c;
}
