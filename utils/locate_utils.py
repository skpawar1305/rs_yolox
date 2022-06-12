#!/usr/bin/env python

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from tf2_ros.transform_broadcaster import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs


class Locate(Node):
    def __init__(self, cv_bridge):
        super().__init__('rs_locator')
        self.cv_bridge = cv_bridge

        # 
        self._intrinsics = rs.intrinsics()
        self._intrinsics.width = 640
        self._intrinsics.height = 480
        K = [
                617.50146484375, 0.0, 327.347900390625,
                0.0, 617.783447265625, 238.0932159423828,
                0.0, 0.0, 1.0
            ]
        self._intrinsics.ppx = K[2]
        self._intrinsics.ppy = K[5]
        self._intrinsics.fx = K[0]
        self._intrinsics.fy = K[4]
        self._intrinsics.model = rs.distortion.none
        D = [0., 0., 0., 0., 0.]
        self._intrinsics.coeffs = D

        #
        self.global_frame = 'camera_link'
        self.tfb_ = TransformBroadcaster(self)

    def locate(self, depth_frame, bboxes, cls, cls_names):
        for idx, bbox in enumerate(bboxes):
            if bbox[0] < 0:
                bbox[0] = 0
            if bbox[1] < 0:
                bbox[1] = 0
            if bbox[2] < 0:
                bbox[2] = 0
            if bbox[3] < 0:
                bbox[3] = 0
            xmin = int(bbox[0])
            ymin = int(bbox[1])
            xmax = int(bbox[2])
            ymax = int(bbox[3])

            class_id = str(cls_names[int(cls[idx])])
            frame = str(class_id) + '_' + str(idx)
            
            point = [round((xmin + xmax)/2), round((ymin + ymax)/2)]

            # Show distance for a specific point
            distance = depth_frame[point[1], point[0]]
            coor = self.convert_depth_to_phys_coord_using_realsense(point[0], point[1], distance)

            self.tf(coor, frame)

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):
        result = rs.rs2_deproject_pixel_to_point(self._intrinsics, [x, y], depth)
        #result[0]: right, result[1]: down, result[2]: forward
        return round(result[2]), round(-result[0]), round(-result[1])

    def tf(self, pose, frame):
        tfs = TransformStamped()
        
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id= self.global_frame
        tfs._child_frame_id = frame
        tfs.transform.translation.x = float(pose[0]) / 1000
        tfs.transform.translation.y = float(pose[1]) / 1000
        tfs.transform.translation.z = float(pose[2]) / 1000
        r = R.from_euler('xyz',[0., 0., 0.])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
