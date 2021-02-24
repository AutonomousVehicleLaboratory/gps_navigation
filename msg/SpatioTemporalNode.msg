# Node ID
uint32 id

# Node Type: Traversed (0), Planned (1), Pose(2), Road Other(3)
uint32 type

# Relative (x, y) position to ego-vehicle Pose(2). Units in meters.
geometry_msgs/Point[] position

# Edges assigned by connectivity
uint32[] neighbors

 


