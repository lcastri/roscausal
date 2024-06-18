# Message representing predictions as trajectory
std_msgs/Header                     header
uint64                              id          # ID of the prediction
float64                             frequency   # prediction frequency
Intention                           intention   # intention for this human
geometry_msgs/PoseWithCovariance[]  sample  # prediction sample for a single human
