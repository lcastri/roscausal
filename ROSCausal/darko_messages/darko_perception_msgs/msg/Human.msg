# Message representing a single human in a scene, as output by T2.5.
# For oriented 3D bounding boxes of humans, see instead the SceneObject message!

uint64                              id          # unique ID of the human
float64                             confidence  # detection confidence or track score, usually between 0.0 and 1.0

geometry_msgs/PoseWithCovariance    centroid    # body centroid, usually centered around the root (pelvis) joint
geometry_msgs/TwistWithCovariance   velocity    # the movement direction of the human

string                              classified_pose  # the estimated discrete pose label (e.g. "standing", "walking", "sitting", "kneeing"...)

HumanBodyJoint[]                    body_joints  # 3D body pose. See Humans.skeleton_config for the underlying skeleton definition and joint connectivity

# The following direction vectors are all-zero / zero norm if they are not available or set.
# Subscribers should check for this.
geometry_msgs/Vector3               upper_body_orientation  # upper-body orientation in coordinate frame specified by Humans.header
geometry_msgs/Vector3               head_orientation  # head-orientation in coordinate frame specified by Humans.header
geometry_msgs/Vector3               gaze_direction  # gaze direction in coordinate frame specified by Humans.header



