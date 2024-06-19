# NOTE: In contrast to sensor_msgs/RegionOfInterest, the x_offset and y_offset fields are signed integers,
# to allow to specify 2D bounding boxes that lie partially outside of the image. 

int32   x_offset
int32   y_offset

uint32   width
uint32   height