# Message describing a rectangular region of interest in a depth or RGB image containing a part of a person (e.g. head, face, full body...), which is usually encoded in the topic title
#

uint64          detection_id
float64         confidence

float64         mean_distance  # mean distance of the person in meters (from the sensor origin). set to NaN if unknown

ROI             roi  # similar to sensor_msgs/RegionOfInterest, but with signed x and y offset

