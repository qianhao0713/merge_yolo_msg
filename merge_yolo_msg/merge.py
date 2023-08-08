

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
import time
import sys,os
sys.path.append("/home/gywrc-s1/xfy/xufengyu_BasePerception_0720_sam/src/msgs")
import message_filters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from perception_msgs.msg import VitImageEmbedding
from perception_msgs.msg import BoundingBoxes
from perception_msgs.msg import VitBoundingBoxes
from collections import deque

def compare_ts(ts1, ts2):
    return (ts1.sec-ts2.sec) * 1e9 + (ts1.nanosec-ts2.nanosec)

class MinimalSubscriber(Node):

    def __init__(self, name):
        super().__init__('minimal_subscriber')
        self.get_logger().info("启动%s node" % name)
        self.bridge = CvBridge()

        # self.image_emb_sub = message_filters.Subscriber(self, VitImageEmbedding, '/Image_embedding')
        # self.image_emb_sub = message_filters.Subscriber(self, VitImageEmbedding, '/vit_embedding_raw')
        # self.yolo_sub = message_filters.Subscriber(self, BoundingBoxes, '/yolov8/bounding_boxes')
        img_callback_group = MutuallyExclusiveCallbackGroup()
        yolo_callback_group = MutuallyExclusiveCallbackGroup()
        self.image_emb_sub = self.create_subscription(VitImageEmbedding, "/vit_embedding_raw", self.callback, 1, callback_group=img_callback_group)
        self.yolo_sub = self.create_subscription(BoundingBoxes, "/yolov8/bounding_boxes", self.yolo_callback, 1, callback_group=yolo_callback_group)
        self.yolo_queue = deque()
        self.queue_size = 30
        # ts = message_filters.TimeSynchronizer([self.image_emb_sub, self.yolo_sub], queue_size=25)
        # ts.registerCallback(self.callback)
        self.publisher_ = self.create_publisher(VitBoundingBoxes, '/vit_bounding_boxes', 1)

    def callback(self, msg_emb):
        while True:
            if len(self.yolo_queue) == 0:
                self.get_logger().info('yolo_msg not received')
                return
            msg_yolo = self.yolo_queue[0]
            comp = compare_ts(msg_emb.header.stamp, msg_yolo.header.stamp)
            comp = comp / 1e9
            if abs(comp)<=0.001:
                self.get_logger().info('found yolo_msg')
                break
            elif comp < -0.001:
                self.get_logger().info('yolo_msg too late')
                return
            else:
                self.yolo_queue.popleft()
        out_msg = VitBoundingBoxes()
        out_msg.header.stamp.sec = msg_emb.header.stamp.sec
        out_msg.header.stamp.nanosec = msg_emb.header.stamp.nanosec
        out_msg.data = msg_yolo.data
        out_msg.ipc_header = msg_emb.ipc_header
        out_msg.check_header = msg_emb.check_header
        out_msg.raw_emb = msg_emb.raw_emb
        out_msg.image = msg_emb.image
        print(out_msg.header)
        self.publisher_.publish(out_msg)

    def yolo_callback(self, msg_yolo):
        self.yolo_queue.append(msg_yolo)
        if len(self.yolo_queue) > self.queue_size:
            self.yolo_queue.popleft()

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber('MergeVitBoundingBox')
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
