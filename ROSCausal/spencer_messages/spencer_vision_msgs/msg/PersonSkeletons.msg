# Message describing an array of articulated human body poses (skeletons)
#

std_msgs/Header     header
PersonSkeleton[]    elements  # all skeletons detected or tracked in the current time frame

string              skeleton_convention  # the convention after which joints are named and the skeleton is composed

string              SKELETON_CONVENTION_COCO = coco
string              SKELETON_CONVENTION_SYNTHETIC_COMPLEX = synthetic_complex
string              SKELETON_CONVENTION_SYNTHETIC_SIMPLIFIED = synthetic_simplified
string              SKELETON_CONVENTION_K4A = k4a