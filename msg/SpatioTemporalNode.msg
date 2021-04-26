# Node ID
uint32 id

# Node Type: (0) Stop (1), Crossing (2), Signal(3), 
#            (3) Traversed Point, (4) Planned Point
#            (5) FootPath, (6) Road Element
uint32 type

# Graph ID: unique identifier utilized for graph search
uint32 graph_id

# Latitude/Longitude of point
float64 lat
float64 lon

# Edges:
bool has_neighbors
uint32[] edges

# Relative (x, y) position to ego-vehicle Pose(2). Units in meters.
geometry_msgs/Point position




 


