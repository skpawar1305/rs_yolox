#!/usr/bin/env python

from loguru import logger
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from tf2_ros.transform_broadcaster import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
from realsense_interfaces.msg import DetectedObjectCoordinates

def getMaxLengthOfObject(object_name):
    
    objects = getObjectsToDetect()
    for object in objects:
        if(object.name == object_name):
            return object.max_length
    return 0

def getObjectsToDetect():
    vase = ObjectsToDetect("vase",150)
    tv = ObjectsToDetect("tv",350)
    chair = ObjectsToDetect("chair",1000)
    
    return [vase,tv,chair]

def center_point_depth_mean(bbox_depth_frame, center_point_distance, max_length_of_object):

        height = bbox_depth_frame.shape[0]
        width = bbox_depth_frame.shape[1]

        observed_pixels_amount = height*width
        
        sum = 0
        
        nearest_depth = center_point_distance - max_length_of_object
        farest_depth = center_point_distance + max_length_of_object

        try:
            for i in range(height):
                for j in range(width):
                    # ignore pixels with 0 as depth as it's incorrect data
                    if(bbox_depth_frame[i,j] == 0):
                        observed_pixels_amount-=1
                    elif(bbox_depth_frame[i,j] < nearest_depth or bbox_depth_frame[i,j] > farest_depth):
                        observed_pixels_amount-=1
                    else:
                        sum += bbox_depth_frame[i,j]

        except Exception as e:
            logger.error(e)

        mean_depth = sum/observed_pixels_amount
        return mean_depth

def center_point_depth_median(bbox_depth_frame, center_point_distance, max_length_of_object):

    height = bbox_depth_frame.shape[0]
    width = bbox_depth_frame.shape[1]

    observed_pixels_amount = height*width
    
    median_depth = 0
    depth_array = []
    
    nearest_depth = center_point_distance - max_length_of_object
    farest_depth = center_point_distance + max_length_of_object

    try:
        for i in range(height):
            for j in range(width):
                # ignore pixels with 0 as depth as it's incorrect data
                if(bbox_depth_frame[i,j] == 0):
                    observed_pixels_amount-=1
                elif(bbox_depth_frame[i,j] < nearest_depth or bbox_depth_frame[i,j] > farest_depth):
                    observed_pixels_amount-=1
                else:
                    depth_array.append(bbox_depth_frame[i,j])

    except Exception as e:
        logger.error(e)

    sorted(depth_array)
    if observed_pixels_amount % 2 != 0:
        median_depth = float(depth_array[int(observed_pixels_amount/2)])
    else:
        median_depth = float((depth_array[int((observed_pixels_amount-1)/2)] + depth_array[int(observed_pixels_amount/2)])/2.0)
    
    return median_depth

def mean_depth(bbox_depth_frame):

    height = bbox_depth_frame.shape[0]
    width = bbox_depth_frame.shape[1]

    observed_pixels_amount = height*width
   
    sum = 0
    
    try:
        for i in range(height):
            for j in range(width):
                # ignore pixels with 0 as depth as it's incorrect data
                if(bbox_depth_frame[i,j] == 0):
                    observed_pixels_amount-=1
                else:
                    sum += bbox_depth_frame[i,j]

    except Exception as e:
        logger.error(e)

    mean_depth = sum/observed_pixels_amount
    return mean_depth

def median_depth(bbox_depth_frame):

    height = bbox_depth_frame.shape[0]
    width = bbox_depth_frame.shape[1]

    observed_pixels_amount = height*width
    median_depth = 0
    depth_array = []
    
    try:
        for i in range(height):
            for j in range(width):
                # ignore pixels with 0 as depth as it's incorrect data
                if(bbox_depth_frame[i,j] == 0):
                    observed_pixels_amount -= 1
                else:
                    depth_array.append(bbox_depth_frame[i,j])

    except Exception as e:
        logger.error(e)

    sorted(depth_array)
    if observed_pixels_amount % 2 != 0:
        median_depth = float(depth_array[int(observed_pixels_amount/2)])
    else:
        median_depth = float((depth_array[int((observed_pixels_amount-1)/2)] + depth_array[int(observed_pixels_amount/2)])/2.0)
    
    return median_depth

class ObjectsToDetect:

  def __init__(self, name, max_length):
    self.name = name
    self.max_length = max_length
    
class Locate(Node):
    def __init__(self, cv_bridge):
        super().__init__('rs_locator')
        self.pub = self.create_publisher(DetectedObjectCoordinates,"detected_object_coordinates", 10)
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

    def locate(self, depth_frame, bboxes, cls, cls_names, depth_estimation_method):
        
        depth_estimation_methods = {'center_point_depth','mean','median','center_point_depth_mean','center_point_depth_median'}
        if depth_estimation_method not in depth_estimation_methods:
            logger.error("depth_estimation_method must be one of ...")#%r." % depth_estimation_methods)

        # extract bbox value
            # xmin, ymin -> left bottom corner
            # xmax, ymax -> right upper corner
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

            class_name = str(cls_names[int(cls[idx])])    # name of detected object
            frame = str(class_name) + '_' + str(idx)
            
            
            distance = 0
            center_point = [round((xmin + xmax)/2), round((ymin + ymax)/2)] # center point of bounding box
            bbox_depth_frame = depth_frame[ymin:ymax, xmin:xmax]
            
            if (depth_estimation_method == "center_point_depth"):
                distance = depth_frame[center_point[1], center_point[0]]

            elif (depth_estimation_method == "mean"):
                distance = mean_depth(bbox_depth_frame)

            elif (depth_estimation_method == "median"):
                distance = median_depth(bbox_depth_frame)

            elif (depth_estimation_method == "center_point_depth_mean"):
                distance_center_point = depth_frame[center_point[1], center_point[0]]
                max_length_of_object = getMaxLengthOfObject(class_name)

                if distance_center_point != 0 and max_length_of_object != 0:
                    distance = center_point_depth_mean(self, bbox_depth_frame,distance_center_point,max_length_of_object)
                else:
                    distance = distance_center_point
            
            elif (depth_estimation_method == "center_point_depth_median"):
                distance_center_point = depth_frame[center_point[1], center_point[0]]
                max_length_of_object = getMaxLengthOfObject(class_name)

                if distance_center_point != 0 and max_length_of_object != 0:
                    distance = center_point_depth_median(self, bbox_depth_frame,distance_center_point,max_length_of_object)
                else:
                    distance = distance_center_point            

            coor = self.convert_depth_to_phys_coord_using_realsense(center_point[0], center_point[1], distance)

            try:
                msg = DetectedObjectCoordinates()
                msg.name = class_name
                msg.coordinates.x = float(coor[0])
                msg.coordinates.y= float(coor[1])
                msg.coordinates.z = float(coor[2])
                
                self.pub.publish(msg)

            except Exception as e:
                logger.error(e)

            self.tf(coor, frame)

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):
        result = rs.rs2_deproject_pixel_to_point(self._intrinsics, [x, y], depth)
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
