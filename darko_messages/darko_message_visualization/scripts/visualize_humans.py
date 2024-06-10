#!/usr/bin/env python3
import rospy
import copy
import numpy as np
import re
import rospkg
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from darko_perception_msgs.msg import Humans
from visualization_msgs.msg import MarkerArray, Marker
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

class HumanVisualizer(object):
    def __init__(self):
        # Parameters
        queue_size = 3

        #
        # ROS publishers
        #
        marker_array_topic = rospy.resolve_name('~marker_array')
        self.marker_array_pub = rospy.Publisher(marker_array_topic, MarkerArray, queue_size=queue_size)
        
        bboxes3d_topic = rospy.resolve_name('~bboxes3d')
        self.bboxes3d_pub = rospy.Publisher(bboxes3d_topic, BoundingBoxArray, queue_size=queue_size)
        
        rospy.loginfo('Publishing markers for DARKO humans on topic: {} and 3D bounding boxes on topic: {}'.format(marker_array_topic, bboxes3d_topic))
        
        #
        # ROS subscribers
        #
        humans_topic = rospy.resolve_name('/perception/humans')
        rospy.loginfo('Visualizer is subscribing to humans on topic: {}'.format(humans_topic))
        self.scene_objects_sub = rospy.Subscriber(humans_topic, Humans, self.on_humans_received, queue_size=queue_size, buff_size=queue_size * 10000000)


    def get_mesh_properties(self, class_label):
        for pattern in self.MESH_MAPPING.keys():
            if re.match(pattern, class_label):
                return self.MESH_MAPPING[pattern]
        return None

    def create_markers_for_human(self, human, human_index, header, skeleton_config):
        # Create body-joint lookup
        joint_lookup = dict()
        for joint in human.body_joints:
            joint_lookup[joint.name] = joint
        
        # Centroid marker
        centroid_size = 0.2
        centroid_marker = Marker(header=header)
        centroid_marker.id = human_index
        centroid_marker.ns = 'Centroids'
        centroid_marker.action = Marker.ADD
        centroid_marker.type = Marker.SPHERE
        centroid_marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.75)
        centroid_marker.scale = Vector3(centroid_size, centroid_size, centroid_size)
        centroid_marker.pose = copy.deepcopy(human.centroid.pose)
        markers = [ centroid_marker ]

        # Twist marker, if velocity is non-zero
        p = human.centroid.pose.position
        v = human.velocity.twist.linear
        speed = np.linalg.norm([v.x, v.y, v.z])
        if speed > 0.05:
            velocity_marker = Marker(header=header)
            velocity_marker.id = human_index
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
            
            velocity_marker.pose.orientation.w = 1
            
            markers += [velocity_marker]
        
        # Head orientation marker
        p = joint_lookup["nose"].pose.position
        v = human.head_orientation
        dir_norm = np.linalg.norm([v.x, v.y, v.z])
        arrow_len = 0.4
        if dir_norm > 0.05:
            orientation_marker = Marker(header=header)
            orientation_marker.id = human_index
            orientation_marker.ns = 'Head orientation'
            orientation_marker.action = Marker.ADD
            orientation_marker.type = Marker.ARROW
            orientation_marker.color = ColorRGBA(0.9, 0.0, 0.7, 1)

            start_point = Point(p.x, p.y, p.z)
            end_point = Point(p.x + arrow_len*v.x, p.y + arrow_len*v.y, p.z + arrow_len*v.z)
            orientation_marker.points.append(start_point)
            orientation_marker.points.append(end_point)
            orientation_marker.scale.x = 0.03
            orientation_marker.scale.y = orientation_marker.scale.x * 3.0
            orientation_marker.scale.z = 0.1
            
            orientation_marker.pose.orientation.w = 1
            
            markers += [orientation_marker]
            
        # Upper-body orientation marker
        p = joint_lookup["spine"].pose.position
        v = human.upper_body_orientation
        dir_norm = np.linalg.norm([v.x, v.y, v.z])
        arrow_len = 0.8
        if dir_norm > 0.05:
            orientation_marker = Marker(header=header)
            orientation_marker.id = human_index
            orientation_marker.ns = 'Upper-body orientation'
            orientation_marker.action = Marker.ADD
            orientation_marker.type = Marker.ARROW
            orientation_marker.color = ColorRGBA(0.9, 0.0, 0.7, 1)

            start_point = Point(p.x, p.y, p.z)
            end_point = Point(p.x + arrow_len*v.x, p.y + arrow_len*v.y, p.z + arrow_len*v.z)
            orientation_marker.points.append(start_point)
            orientation_marker.points.append(end_point)
            orientation_marker.scale.x = 0.05
            orientation_marker.scale.y = orientation_marker.scale.x * 3.0
            orientation_marker.scale.z = 0.1
            
            orientation_marker.pose.orientation.w = 1
            
            markers += [orientation_marker]
            
        # Human skeletons
        joints_marker = Marker(header=header)
        joints_marker.id = human_index
        joints_marker.ns = 'Body Joints'
        joints_marker.action = Marker.ADD
        joints_marker.type = Marker.SPHERE_LIST
        joints_marker.color = ColorRGBA(0.9, 0.0, 0.9, 1)
        joints_marker.pose.orientation.w = 1
        joints_marker.scale.x = joints_marker.scale.y = joints_marker.scale.z = 0.05
        
        for joint_name in skeleton_config.joint_names:
            joints_marker.points.append(joint_lookup[joint_name].pose.position)
            
        markers += [joints_marker]
            
        skeleton_marker = Marker(header=header)
        skeleton_marker.id = human_index
        skeleton_marker.ns = 'Skeletons'
        skeleton_marker.action = Marker.ADD
        skeleton_marker.type = Marker.LINE_LIST
        skeleton_marker.color = ColorRGBA(0.8, 0.0, 0.8, 1)
        skeleton_marker.pose.orientation.w = 1
        skeleton_marker.scale.x = 0.025
            
        for edge in zip(skeleton_config.bone_joint1_idx, skeleton_config.bone_joint2_idx):
            idx1, idx2 = edge
            name1, name2 = skeleton_config.joint_names[idx1], skeleton_config.joint_names[idx2]
            pos1, pos2 = joint_lookup[name1].pose.position, joint_lookup[name2].pose.position
            
            skeleton_marker.points.append(pos1)
            skeleton_marker.points.append(pos2)
        
        markers += [skeleton_marker]
            
        # Text marker
        text_marker = Marker(header=header)
        text_marker.id = human_index
        text_marker.ns = 'Text labels'
        text_marker.action = Marker.ADD
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = str(human.id)

        text_marker.color = ColorRGBA(1,1,1,1)
        text_marker.scale.z = 0.1
        text_marker.pose = copy.deepcopy(centroid_marker.pose)
        text_marker.pose.position.z += 1.0

        markers += [text_marker]

        return markers

    def on_humans_received(self, humans):
        rospy.loginfo_once('Visualizer received first set of Humans!')

        #
        # Create markers
        #

        marker_array = MarkerArray()

        delete_marker = Marker(header=humans.header)
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for human_index, human in enumerate(humans.humans):
            markers = self.create_markers_for_human(human, human_index, humans.header, humans.skeleton_config)
            for marker in markers:
                if marker is not None: marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)

        #
        # Create 3D bounding boxes
        #

        bbox3d_array = BoundingBoxArray(header=humans.header)
        for human_index, human in enumerate(humans.humans):
            bbox3d = BoundingBox(header=bbox3d_array.header)
            bbox3d.label = human.id
            bbox3d.pose = human.centroid.pose
            bbox3d.dimensions.x = 0.4
            bbox3d.dimensions.y = 0.4
            bbox3d.dimensions.z = 1.8
            bbox3d.value = human.confidence

            bbox3d_array.boxes.append(bbox3d)

        self.bboxes3d_pub.publish(bbox3d_array)


rospy.init_node('visualize_humans')
visualizer = HumanVisualizer()
rospy.spin()