# mytiago_causal_discovery/msg/CausalModel.msg

Header header
string[] features
std_msgs/Int32MultiArray causal_structure
std_msgs/Float32MultiArray val_matrix
std_msgs/Float32MultiArray pval_matrix
int32[] original_shape