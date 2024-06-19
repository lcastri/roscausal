# Message describing a person candidate ROI in a 3D point cloud, represented as a cuboid aligned to the coordinate frame axes.
#

uint64          detection_id
float64         confidence

geometry_msgs/Point  min_coords
geometry_msgs/Point  max_coords

