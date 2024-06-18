#!/usr/bin/env python
import rospy
import copy
import numpy as np
import re
import rospkg
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from darko_perception_msgs.msg import SceneObjects
from visualization_msgs.msg import MarkerArray, Marker
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

class SceneObjectVisualizer(object):
    MESH_MAPPING = {
        'blue_bin_.*': {
            'mesh_uri': 'package://darko_models/models/blue_bin_large/meshes/blue_bin.dae',
            'mesh_size': [0.5, 0.3, 0.20],
            # FIXME: The origin offset should be applied in object-centric coordinates, not world coordintes!
            # This leads to wrong results e.g. if the object has roll or pitch.
            # i.e. we should multiply these offsets with the object rotation matrix.
            'mesh_origin': [0, +0.070, -0.07]
        },
    }

    def __init__(self):
        # Mapping of class labels to numeric class IDs, for jsk_recognition_msgs/BoundingBoxArray
        self.class_label_to_id_map = dict()

        # Check if we have DARKO mesh models available. Otherwise, we don't publish mesh markers
        # to avoid tons of Rviz error messages in the console.
        try:
            rospack = rospkg.RosPack()
            rospack.get_path('darko_models')
            self.has_darko_meshes = True
        except rospkg.common.ResourceNotFound as e:
            self.has_darko_meshes = False

        # Parameters
        queue_size = 3

        #
        # ROS publishers
        #
        marker_array_topic = rospy.resolve_name('~marker_array')
        self.marker_array_pub = rospy.Publisher(marker_array_topic, MarkerArray, queue_size=queue_size)
        
        bboxes3d_topic = rospy.resolve_name('~bboxes3d')
        self.bboxes3d_pub = rospy.Publisher(bboxes3d_topic, BoundingBoxArray, queue_size=queue_size)
        
        rospy.loginfo('Publishing markers for DARKO scene objects on topic: {} and 3D bounding boxes on topic: {}'.format(marker_array_topic, bboxes3d_topic))
        
        #
        # ROS subscribers
        #
        scene_objects_topic = rospy.resolve_name('/perception/scene_objects')
        rospy.loginfo('Visualizer is subscribing to scene objects on topic: {}'.format(scene_objects_topic))
        self.scene_objects_sub = rospy.Subscriber(scene_objects_topic, SceneObjects, self.on_scene_objects_received, queue_size=queue_size, buff_size=queue_size * 10000000)


    def get_mesh_properties(self, class_label):
        for pattern in self.MESH_MAPPING.keys():
            if re.match(pattern, class_label):
                return self.MESH_MAPPING[pattern]
        return None

    def create_markers_for_scene_object(self, scene_object, scene_object_index, header):
        # Centroid marker
        centroid_size = 0.05
        centroid_marker = Marker(header=header)
        centroid_marker.id = scene_object_index
        centroid_marker.ns = 'Centroids'
        centroid_marker.action = Marker.ADD
        centroid_marker.type = Marker.SPHERE
        centroid_marker.color = ColorRGBA(1,1,0,1)
        centroid_marker.scale = Vector3(centroid_size, centroid_size, centroid_size)
        centroid_marker.pose = copy.deepcopy(scene_object.pose.pose)
        markers = [ centroid_marker ]

        # Twist marker, if velocity is non-zero
        p = scene_object.pose.pose.position
        v = scene_object.velocity.twist.linear
        speed = np.linalg.norm([v.x, v.y, v.z])
        if speed > 0.05:
            velocity_marker = Marker(header=header)
            velocity_marker.id = scene_object_index
            velocity_marker.ns = 'Velocity arrows'
            velocity_marker.action = Marker.ADD
            velocity_marker.type = Marker.ARROW
            velocity_marker.color = ColorRGBA(0.7,0,0,1)

            start_point = Point(p.x, p.y, p.z)
            end_point = Point(p.x + v.x, p.y + v.y, p.z + v.z)
            velocity_marker.points.append(start_point)
            velocity_marker.points.append(end_point)
            velocity_marker.scale.x = 0.03
            velocity_marker.scale.y = velocity_marker.scale.x * 3.0
            velocity_marker.scale.z = 0.1
            
            markers += [velocity_marker]
        

        # Text marker
        text_marker = Marker(header=header)
        text_marker.id = scene_object_index
        text_marker.ns = 'Text labels'
        text_marker.action = Marker.ADD
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = str(scene_object.class_label)

        # FIXME: Put these label remappings into a dict
        if 'blue_bin' in text_marker.text:
            text_marker.text = 'bin'

        text_marker.color = ColorRGBA(1,1,1,1)
        text_marker.scale.z = 0.1
        text_marker.pose = copy.deepcopy(centroid_marker.pose)
        text_marker.pose.position.z += scene_object.extents.z / 2.0

        markers += [text_marker]
        
        # Mesh marker, if a mesh exists
        mesh_properties = self.get_mesh_properties(scene_object.class_label)
        if mesh_properties:
            mesh_marker = Marker(header=header)
            mesh_marker.id = scene_object_index
            mesh_marker.ns = 'Meshes'
            mesh_marker.action = Marker.ADD
            mesh_marker.type = Marker.MESH_RESOURCE
            mesh_marker.mesh_resource = mesh_properties['mesh_uri']
            mesh_marker.mesh_use_embedded_materials = True

            mesh_size = np.array(mesh_properties['mesh_size'])
            target_size = np.array([scene_object.extents.x, scene_object.extents.y, scene_object.extents.z])
            scale_factors = np.divide(target_size, mesh_size)
            mesh_marker.scale = Vector3(*scale_factors.tolist())

            mesh_origin = np.array(mesh_properties['mesh_origin'])
            mesh_marker.pose = copy.deepcopy(centroid_marker.pose)
            obj_position = np.array([mesh_marker.pose.position.x, mesh_marker.pose.position.y, mesh_marker.pose.position.z])
            mesh_position = obj_position + mesh_origin
            mesh_marker.pose.position = Point(*mesh_position.tolist())

            markers += [ mesh_marker ]

        return markers

    def on_scene_objects_received(self, scene_objects):
        rospy.loginfo_once('Visualizer received first set of SceneObjects!')

        #
        # Create markers
        #

        marker_array = MarkerArray()

        delete_marker = Marker(header=scene_objects.header)
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for scene_object_index, scene_object in enumerate(scene_objects.objects):
            markers = self.create_markers_for_scene_object(scene_object, scene_object_index, scene_objects.header)
            for marker in markers:
                if marker is not None: marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)

        #
        # Create 3D bounding boxes
        #

        bbox3d_array = BoundingBoxArray(header=scene_objects.header)
        for scene_object_index, scene_object in enumerate(scene_objects.objects):
            # FIXME: In darko_perception_messages, we could also have a helper module to do this string->ID mapping consistently
            if not scene_object.class_label in self.class_label_to_id_map:
                self.class_label_to_id_map[scene_object.class_label] = len(self.class_label_to_id_map) 

            bbox3d = BoundingBox(header=bbox3d_array.header)
            bbox3d.label = self.class_label_to_id_map[scene_object.class_label] # should be int
            bbox3d.pose = scene_object.pose.pose
            bbox3d.dimensions = scene_object.extents
            bbox3d.value = scene_object.confidence

            bbox3d_array.boxes.append(bbox3d)

        self.bboxes3d_pub.publish(bbox3d_array)


rospy.init_node('visualize_scene_objects')
visualizer = SceneObjectVisualizer()
rospy.spin()