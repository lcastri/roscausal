# Message representing a single (large-scale) 3D object with 9DoF in the scene, as output by T2.1.

uint64                              id          # unique ID of each object. for object detections, uniqueness is guaranteed per frame. for tracked objects, it should be unique across time.
string                              class_label # object class
float64                             confidence  # detection confidence or track score, usually between 0.0 and 1.0
geometry_msgs/PoseWithCovariance    pose        # 6DoF pose of the oriented 3D bounding box -- comprised of centroid, and orientation relative to the parent frame
geometry_msgs/Vector3               extents     # 3D extents of the bounding box in object-centric xyz coordinates (+x=forward, +y=left, +z=up)
geometry_msgs/TwistWithCovariance   velocity    # current velocity of the object, if any. always 0 for static objects, or single-frame detections.
