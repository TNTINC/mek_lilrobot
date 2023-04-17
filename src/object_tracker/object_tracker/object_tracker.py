import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

import object_tracker.vision as vision

class ObjectDetector(Node):
    tuning = True
    def __init__(self):
        super().__init__("object_detector")

        self.img_sub = self.create_subscription(
            Image, 
            "/image_in", 
            self.recieve_img, 
            qos_profile_sensor_data)
        
        if(self.tuning):
            self.img_pub = self.create_publisher(Image, "/image_tuning", qos_profile_sensor_data)

        self.det_pub = self.create_publisher(Point, "/det_deviation", 1)  
        # Publish X and Y frame coords ([-1,1] with 0,0 at center), with Z representing color

        self.bridge = CvBridge()

    def recieve_img(self, data: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detection = vision.process(img)
            if detection.x is not None:
                pt = Point()
                pt.x = detection.x
                pt.y = detection.y
                pt.z = float(int(detection.color))
                self.det_pub.publish(pt)
            if self.tuning:
                msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.img_pub.publish(msg)
        
        except CvBridgeError as e:
            print(e)

def main(args=None):
    #TODO: Add logging
    rclpy.init(args=args)
    node = ObjectDetector()
    while rclpy.ok():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()