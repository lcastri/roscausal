# Message describing a point cloud containing a person

uint64                    person_id   # usually associated to a track or detection
sensor_msgs/PointCloud2   cloud

float32                   distance              # from the sensor, in meters
float32                   azimuth_angle_deg     # relative to the sensor, in meters