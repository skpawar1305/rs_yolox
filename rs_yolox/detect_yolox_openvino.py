from pathlib import Path

import openvino.runtime as ov
from openvino.preprocess import PrePostProcessor
from openvino.preprocess import ColorFormat
from openvino.runtime import Layout, Type

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
rs_yolox_dir = get_package_share_directory('rs_yolox')
import yaml

SCORE_THRESHOLD = 0.2
NMS_THRESHOLD = 0.4
CONFIDENCE_THRESHOLD = 0.4


def resize_and_pad(image, new_shape):
    old_size = image.shape[:2] 
    ratio = float(new_shape[-1]/max(old_size))#fix to accept also rectangular images
    new_size = tuple([int(x*ratio) for x in old_size])

    image = cv2.resize(image, (new_size[1], new_size[0]))
    
    delta_w = new_shape[1] - new_size[1]
    delta_h = new_shape[0] - new_size[0]
    
    color = [100, 100, 100]
    new_im = cv2.copyMakeBorder(image, 0, delta_h, 0, delta_w, cv2.BORDER_CONSTANT, value=color)
    
    return new_im


class Detector(Node):
    def __init__(self) -> None:
        super().__init__('YOLOV5')
        self.bridge = CvBridge()

        core = ov.Core()
        model = core.read_model(str(Path(rs_yolox_dir + "/weights/yolov5s/yolov5s.xml")))
        with open(str(Path(rs_yolox_dir + "/weights/yolov5s/yolov5s.yaml")), 'r') as stream:
            self.classes = yaml.safe_load(stream)['names']

        ppp = PrePostProcessor(model)
        ppp.input().tensor().set_element_type(Type.u8).set_layout(Layout("NHWC")).set_color_format(ColorFormat.BGR)
        ppp.input().preprocess().convert_element_type(Type.f32).convert_color(ColorFormat.RGB).scale([255., 255., 255.])
        ppp.input().model().set_layout(Layout("NCHW"))
        ppp.output().tensor().set_element_type(Type.f32)
        model = ppp.build()
        self.compiled_model = core.compile_model(model, "CPU")

        self.sub = self.create_subscription(Image,"image_raw",self.imageflow_callback, 1)
        cv2.namedWindow("YOLOV5s")

    def imageflow_callback(self,msg:Image) -> None:
        img = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        img_resized = resize_and_pad(img, (640, 640))

        input_tensor = np.expand_dims(img_resized, 0)
        infer_request = self.compiled_model.create_infer_request()
        infer_request.infer({0: input_tensor})
        output = infer_request.get_output_tensor()
        detections = output.data[0]

        boxes = []
        class_ids = []
        confidences = []
        for prediction in detections:
            confidence = prediction[4].item()
            if confidence >= CONFIDENCE_THRESHOLD:
                classes_scores = prediction[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):
                    confidences.append(confidence)
                    class_ids.append(class_id)
                    x, y, w, h = prediction[0].item(), prediction[1].item(), prediction[2].item(), prediction[3].item()
                    xmin = x - (w / 2)
                    ymin = y - (h / 2)
                    box = np.array([xmin, ymin, w, h])
                    boxes.append(box)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD)

        detections = []
        for i in indexes:
            j = i.item()
            detections.append({"class_index": class_ids[j], "confidence": confidences[j], "box": boxes[j]})

        for detection in detections:
            box = detection["box"]
            classId = detection["class_index"]
            confidence = detection["confidence"]
            print( f"Bbox {i} Class: {classId} Confidence: {confidence} Scaled coords: [ cx: {(box[0] + (box[2] / 2)) / img.shape[1]}, cy: {(box[1] + (box[3] / 2)) / img.shape[0]}, w: {box[2]/ img.shape[1]}, h: {box[3] / img.shape[0]} ]" )
            xmax = box[0] + box[2]
            ymax = box[1] + box[3]
            img = cv2.rectangle(img, (int(box[0]), int(box[1])), (int(xmax), int(ymax)), (0, 255, 0), 3)
            img = cv2.rectangle(img, (int(box[0]), int(box[1]) - 20), (int(xmax), int(box[1])), (0, 255, 0), cv2.FILLED)
            img = cv2.putText(img, str(self.classes[classId]), (int(box[0]), int(box[1]) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

        cv2.imshow("YOLOV5s",img)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args=args)
    ros_class = Detector()

    try:
        rclpy.spin(ros_class)
    except KeyboardInterrupt:
        pass
    ros_class.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
