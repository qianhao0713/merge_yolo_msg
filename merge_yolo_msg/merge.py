

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import time
import sys,os
sys.path.append("/home/gywrc-s1/xfy/xufengyu_BasePerception_0720_sam/src/msgs")
import message_filters
from perception_msgs.msg import VitImageEmbedding
from perception_msgs.msg import BoundingBoxes
from perception_msgs.msg import VitBoundingBoxes

class MinimalSubscriber(Node):

    def __init__(self, name):
        super().__init__('minimal_subscriber')
        self.get_logger().info("启动%s node" % name)
        self.bridge = CvBridge()

        self.image_emb_sub = message_filters.Subscriber(self, VitImageEmbedding, '/Image_embedding')
        self.yolo_sub = message_filters.Subscriber(self, BoundingBoxes, '/yolov8/bounding_boxes')
        ts = message_filters.TimeSynchronizer([self.image_emb_sub, self.yolo_sub], queue_size=25)
        ts.registerCallback(self.callback)
        self.publisher_ = self.create_publisher(VitBoundingBoxes, '/vit_bounding_boxes', 1)

    def callback(self, msg_emb, msg_yolo):
        out_msg = VitBoundingBoxes()
        out_msg.header.stamp.sec = msg_emb.header.stamp.sec
        out_msg.header.stamp.nanosec = msg_emb.header.stamp.nanosec
        out_msg.data = msg_yolo.data
        out_msg.ipc_header = msg_emb.ipc_header
        out_msg.check_header = msg_emb.check_header
        out_msg.image = msg_emb.image
        print(out_msg.header)
        self.publisher_.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber('MergeVitBoundingBox')
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
