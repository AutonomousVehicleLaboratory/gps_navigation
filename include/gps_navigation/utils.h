#ifndef UTILS 
#define UTILS 
#include <vector>
#include <tf/transform_datatypes.h>
#include <gps_navigation/graph.h>

namespace gps_navigation{
  const double kDegRadsConversion = M_PI/180;
  const double kOsmOriginX = 32.88465;
  const double kOsmOriginY = -117.24244;  
  const double kREarth = 6371e3;
  double GreatCircleDistance(Node* point1, Node* point2);
  std::pair<double, double> RelativeDisplacement(Node* point1, Node* point2);
}
#endif
