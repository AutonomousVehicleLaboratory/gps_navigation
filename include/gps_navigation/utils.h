#ifndef UTILS 
#define UTILS 
#include <vector>
#include <tf/transform_datatypes.h>
#include <gps_navigation/graph.h>

namespace gps_navigation{
  double GreatCircleDistance(Node* point1, Node* point2);
}
#endif
