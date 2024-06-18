# Message representing the skeleton configuration used for all 3D body poses in a Humans message.

string          skeleton_convention  # name of the skeleton convention

string[]        joint_names  # all joints in the skeleton
uint8           root_joint_idx  # index of the root joint, usually the pelvis (between the hips) acting as a "centroid"

# SKELETAL BONES:
# The following arrays must have the same length. List elements refer
# to indices within the joint_names array, to define connectivity.
#
# e.g. bone_joint1_idx[0] = 7, bone_joint1_idx[0] = 3
# means that there exists a bone from joint joint_names[7] to joint_names[3]
# in the skeleton.
uint8[]         bone_joint1_idx  # start joints of bones
uint8[]         bone_joint2_idx  # end joints of bones
