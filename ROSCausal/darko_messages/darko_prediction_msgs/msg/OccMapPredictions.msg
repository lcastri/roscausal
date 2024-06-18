# Message representing predictions as occupancy map
std_msgs/Header                     header

bool                                aggregated  # true when publishing an aggregated map
float64                             frequency   # prediction frequency
nav_msgs/OccupancyGrid[]              prediction  # prediction