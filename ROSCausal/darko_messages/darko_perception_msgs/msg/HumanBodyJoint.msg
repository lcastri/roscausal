# Message representing a single body joint pose of a human's 3D body pose.

string                      name  # name of the body joint. depends on skeleton configuration
float32                     detection_score  # score between 0 and 1 how confident we are in the detection of this joint
geometry_msgs/Pose          pose  # position and optionally orientation of the body joint
float32[]                   covariance  # optional covariances of position, and possibly orientation