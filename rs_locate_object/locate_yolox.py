#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

from bboxes_ex_msgs.msg import BoundingBoxes
from bboxes_ex_msgs.msg import BoundingBox

from tf2_ros.transform_broadcaster import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
import pyrealsense2 as rs


class Locate(Node):
    def __init__(self):
        super().__init__("rs_locate_object")
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", self.depth_img_cb, 10)
        self.create_subscription(BoundingBoxes, "/bounding_boxes", self.run, 1)
        self.cv_bridge = CvBridge()
        self.depth_frame = None

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
        self.marker_idx = 0
        self.tfb_ = TransformBroadcaster(self)

    def depth_img_cb(self, msg):
        self.depth_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

    def run(self, msg):
        if self.depth_frame is not None:
            for idx, box in enumerate(msg.bounding_boxes):
                frame = str(box.class_id) + '_' + str(idx)
                point = [round((box.xmin + box.xmax)/2), round((box.ymin + box.ymax)/2)]

                # Show distance for a specific point
                distance = self.depth_frame[point[1], point[0]]
                coor = self.convert_depth_to_phys_coord_using_realsense(point[0], point[1], distance)

                self.mark(coor, frame)

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):
        result = rs.rs2_deproject_pixel_to_point(self._intrinsics, [x, y], depth)
        #result[0]: right, result[1]: down, result[2]: forward
        return round(result[2]), round(-result[0]), round(-result[1])

    def mark(self, pose, frame):
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


def main():
    rclpy.init()
    rs_locate_object = Locate()
    try:
        rclpy.spin(rs_locate_object)
    except KeyboardInterrupt:
        pass
    rs_locate_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
