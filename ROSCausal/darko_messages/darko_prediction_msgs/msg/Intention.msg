# Message representing predictions as trajectory
std_msgs/Header                     header
uint64                              id          # unique ID of the prediction
geometry_msgs/PoseWithCovariance    pose        # pose describing a goal to reach

# TODO: we can add a SceneObject as field of an intention