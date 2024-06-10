#!/usr/bin/env python3
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
import tf.transformations as trf
from typing import Union

class SceneObjectVisualizer(object):

    MODEL_MAPPING = {
        'klt.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/klt_tall.dae',
            'extents': (0.595, 0.395, 0.28),  # x y z in meters, from Blender (press N)
            'rescale': True,
            'mesh_origin': 'bottom'
        },
        '.*active_?shuttle.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/active_shuttle.dae',
            'extents': (1.01, 0.411, 0.726),
            'rescale': False,
        },
        '.*spencer.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/spencer.dae',
            'extents': (0.81, 0.81, 2.00),
            'rescale': False,
        },
        '.*chair.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/chair.dae',
            'extents': (0.739, 0.651, 0.992),
            'rescale': False,
            'mesh_origin': 'bottom'
        },
        '.*dolley.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/dolley.dae',
            'extents': (0.60, 0.40, 0.161),
            'rescale': False,
            'mesh_origin': 'bottom'
        },
        '.*pallet_truck.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/pallet_truck.dae',
            'extents': (1.52, 0.519, 1.13),
            'rescale': False,
            'mesh_origin': 'bottom',
            'align_at': 'bottom',
        },
        '.*pallet_stacker.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/pallet_stacker.dae',
            'extents': (1.7, 0.772, 2.03),
            'rescale': False,
            'mesh_origin': 'bottom',
        },
        '.*plant.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/potted_plant.dae',
            'extents': (0.242, 0.231, 0.705),
            'rescale': True,
        },
        '.*cardboard_bin_bsh.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/cardboard_bin_bsh.dae',
            'extents': (0.583, 0.187, 0.172),
            'rescale': True, #?
        },
        '.*cardboard_bin_raja.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/cardboard_bin_raja_large.dae',
            'extents': (0.42, 0.247, 0.12),
            'rescale': True, #?
            'mesh_origin': 'bottom',
        },
        '.*darko_conveyor.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/darko_conveyor.dae',
            'extents': (2.10, 0.54, 0.788),
            'rescale': False, #?
            'mesh_origin': 'bottom',
        },
        '.*darko_shelf.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/darko_shelf_complete.dae',
            'extents': (0.98, 1.41, 1.66),
            'rescale': False,
            'mesh_origin': 'bottom',
            'align_at': 'bottom',
        },
        '.*sofa_red.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/sofa_red.dae',
            'extents': (0.839, 1.66, 1.36),
            'rescale': False,
            'mesh_origin': 'bottom',
        },
        '.*desk.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/table_desk_single.dae',
            'extents': (0.8, 1.60, 0.75),
            'rescale': False,
            'mesh_origin': 'bottom',
        },
        '.*table.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/table_round_single.dae',
            'extents': (0.8, 0.8, 1),
            'rescale': True,
            'mesh_origin': 'bottom',
        },
        '.*plastic_tray.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/plastic_tray.dae',
            'extents': (0.75, 0.48, 0.25),
            'rescale': False, #?
            'mesh_origin': 'bottom',
        },
        '.*plastic_bin.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/plastic_bin_large.dae',
            'extents': (0.48, 0.32, 0.2),
            'rescale': True, #?
            'mesh_origin': 'bottom',
        },
        '.*screen.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/tv_screen.dae',
            'extents': (0.07, 0.97, 0.57),
            'rescale': True,
        },
        '.*step_stool.*': {
            'model_uri': 'package://darko_models/darko_bosch_meshes/step_stool.dae',
            'extents': (0.747, 0.747, 0.601),
            'rescale': True,
            'mesh_origin': 'bottom',
        },
    }

    # used for "mesh_origin" and "align_at"
    def get_relative_offset(self, model_cfg, attr_name):
        attr_value = model_cfg[attr_name]
        if attr_value == 'center':
            rel_offset = (0,0,0)
        elif attr_value == 'bottom':
            rel_offset = (0,0,-0.5)  # as multiple of OBB extents
        elif attr_value == 'top':
            rel_offset = (0,0,0.5)  # as multiple of OBB extents
        else:
            raise ValueError('mesh_origin "%s" is not supported!' % (attr_value))
        return rel_offset

    def create_mesh_marker(self, obb, marker_id, header, marker_ns='') -> Union[Marker, None]:
        label = obb.class_label

        # Find matching model, if any
        model_cfg = None
        for label_regex, cfg in SceneObjectVisualizer.MODEL_MAPPING.items():
            if re.match(label_regex, label):
                model_cfg = cfg
                break

        if model_cfg is None:
            return None

        # Compute target mesh_scale_factors if rescaling is enabled for this model
        mesh_extents = np.array(model_cfg['extents'])
        rescale = model_cfg['rescale'] if 'rescale' in model_cfg else False
        obb_size = np.array([obb.extents.x, obb.extents.y, obb.extents.z])
        mesh_scale_factors = np.divide(obb_size, mesh_extents).tolist() if rescale else [1,1,1]
        scaled_mesh_extents = np.multiply(mesh_extents, mesh_scale_factors)

        # Generate marker
        marker = Marker(header=header)
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = model_cfg['model_uri']
        marker.mesh_use_embedded_materials = True
        marker.scale = Vector3(*mesh_scale_factors)
        marker.pose = copy.deepcopy(obb.pose.pose)
        marker.ns = marker_ns

        # Adjust mesh position if we are using a non-center mesh origin
        if 'mesh_origin' in model_cfg:
            rel_mesh_offset = self.get_relative_offset(model_cfg, 'mesh_origin')
            abs_mesh_offset = np.multiply(rel_mesh_offset, scaled_mesh_extents)
        else:
            abs_mesh_offset = np.zeros_like(scaled_mesh_extents)

        # Optional non-center alignment of mesh to labeled bbox
        if 'align_at' in model_cfg:
            rel_align_offset = self.get_relative_offset(model_cfg, 'align_at')
            abs_align_offset_mesh  = np.multiply(rel_align_offset, scaled_mesh_extents)
            abs_align_offset_label = np.multiply(rel_align_offset, obb_size)
            abs_align_offset = abs_align_offset_label - abs_align_offset_mesh
        else:
            abs_align_offset = np.zeros_like(scaled_mesh_extents)

        # Compute total offset
        abs_total_offset = abs_mesh_offset + abs_align_offset

        # Apply the total offset
        rot_q = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]
        rot_mtx = trf.quaternion_matrix(rot_q)[:3,:3] #R_cam_bboxR_cam_bbox
        rotated_mesh_offset = np.dot(rot_mtx, abs_total_offset)
        label_centroid = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
        mesh_origin_position = label_centroid + rotated_mesh_offset
        marker.pose.position = Point(*mesh_origin_position.tolist())

        # Determine custom color
        if label.startswith('klt_'):
            if 'blue' in label: color = [0,0.2,0.7,1]
            elif 'red' in label: color = [0.7,0,0,1]
            else: color = [0.7,0.7,0.7,1]

            marker.color = ColorRGBA(*color)

        return marker

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
        text_marker.text = f"{scene_object.class_label}, {scene_object.confidence:.2f}"

        # FIXME: Put these label remappings into a dict
        if 'blue_bin' in text_marker.text:
            text_marker.text = 'bin'

        text_marker.color = ColorRGBA(1,1,1,1)
        text_marker.scale.z = 0.15
        text_marker.pose = copy.deepcopy(centroid_marker.pose)
        text_marker.pose.position.z += scene_object.extents.z / 2.0

        markers += [text_marker]

        # Mesh marker, if a mesh exists
        mesh_marker = self.create_mesh_marker(scene_object, scene_object_index, header, 'Meshes')
        if mesh_marker is not None:
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
