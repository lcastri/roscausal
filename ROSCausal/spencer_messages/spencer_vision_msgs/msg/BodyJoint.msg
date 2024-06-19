# Message describing a single body joint of a PersonSkeleton
#

string                  name              # joint name following the convention specified in PersonSkeletons msg. usually in lower-case
float32                 detection_score   # detection confidence of this joint
geometry_msgs/Pose      pose              # absolute joint position + orientation wrt. coordinate frame specified in PersonSkeletons.header.frame_id
float32[]               covariance        # uncertainty in position+orientation of the body joint. either a 6x6 matrix (for x, y, z) represented as row-major 36D vector, a 6D vector with only the diagonal elements, or empty if not available.

