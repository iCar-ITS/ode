import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import torch
import torchvision.transforms.functional as TF
import numpy as np  

import cv2

from ode_predict.model import ODEModel_ResNet
from cv_bridge import CvBridge

# add image and message types as needed
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32MultiArray
from boundingboxes.msg import BoundingBoxes

from message_filters import Subscriber, Cache

class PredictNode(Node):
    def __init__(self):
        super().__init__('predict_node')

        self.declare_parameter('model_path', '/path/to/your/model.pth')
        self.declare_parameter('device', 'cpu')

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.model_load()

        self.cv_bridge = CvBridge()

        # Sync image and bounding boxes
        self.bb_subs_ = Subscriber(self, BoundingBoxes, '/bounding_boxes', qos_profile=qos_profile_sensor_data)
        self.img_subs_ = Subscriber(self, Image, '/image', qos_profile=qos_profile_sensor_data)

        self.cache_ = Cache(self.img_subs_, 5)

        self.bb_subs_.registerCallback(self.callback)

        self.get_logger().info('ODE Predict Node has been started.')

    def callback(self, bb_data):

        if bb_data.bounding_boxes is None or len(bb_data.bounding_boxes) == 0:
            # self.get_logger().warn('No bounding boxes found in the message.')
            return
        
        stamp = Time.from_msg(bb_data.header.stamp)
        
        # Get the latest image from the cache
        # Note: This assumes that the cache is synchronized with the image subscriber
        time_window = Duration(seconds=0.001)
        im_candidate = self.cache_.getInterval(stamp - time_window, stamp + time_window)
        if not im_candidate:
            self.get_logger().warn('No image data found for the given bounding box timestamp.')
            return
        
        # Find the image with the smallest interval difference
        min_diff = float('inf')
        im_data = None
        for img in im_candidate:
            diff = abs((Time.from_msg(img.header.stamp) - Time.from_msg(bb_data.header.stamp)).nanoseconds)
            if diff < min_diff:
                min_diff = diff
                im_data = img

        if im_data is None:
            self.get_logger().warn('No suitable image found for the given bounding box timestamp.')
            return
        
        points = []

        for bb in bb_data.bounding_boxes:
            if bb.xmax <= bb.xmin or bb.ymax <= bb.ymin:
                self.get_logger().warn('Invalid bounding box detected. Skipping this bounding box.')
                continue

            # convert bounding boxes to yolo form
            x_c = (bb.xmax + bb.xmin) / 2.0
            y_c = (bb.ymax + bb.ymin) / 2.0
            w_bb = bb.xmax - bb.xmin
            h_bb = bb.ymax - bb.ymin

            pt = [x_c, y_c]
            points.append(pt)
            self.get_logger().info(f'Bounding box: {bb.class_id} at {pt} with width {w_bb} and height {h_bb}')

        b_points = []
        b_points.append(torch.tensor(points))

        # convert ros image to cv2 image
        img = self.cv_bridge.imgmsg_to_cv2(im_data, desired_encoding='bgr8')
        if img is None:
            self.get_logger().error('Failed to convert ROS image to OpenCV image.')
            return
        
        # convert image to tensor
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_tensor = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0)
        img_tensor = img_tensor.to(self.device)
        img = TF.normalize(img_tensor, mean=[0.485, 0.456, 0.406],
                                       std=[0.229, 0.224, 0.225])

        self.get_logger().info(f'Processing {img_tensor.size()} bounding boxes.')

        pred = self.model_(img_tensor, b_points)

        # convert prediction to numpy array
        pred_np = [p.detach().cpu().numpy() for p in pred]
        pred_np = np.squeeze(pred_np, axis=0)  # Remove batch dimension

        self.get_logger().info(f'Prediction : {pred_np}')

        # dists = Float32MultiArray()
        # dists.data = pred_np.tolist()
        # dists.header = bb_data.header


    @torch.no_grad()
    def model_load(self):

        if self.device == 'cuda':
            if not torch.cuda.is_available():
                self.get_logger().error('CUDA is not available. Switching to CPU.')
                self.device = 'cpu'
        elif self.device == 'cpu':
            if torch.cuda.is_available():
                self.get_logger().warn('CUDA is available. Consider switching to GPU for better performance.')
        elif self.device.startswith('cuda:'):
            try:
                device_id = int(self.device.split(':')[1])
                if torch.cuda.is_available() and device_id < torch.cuda.device_count():
                    self.get_logger().info(f'Using CUDA device {device_id}.')
                else:
                    self.get_logger().error(f'Invalid CUDA device {device_id}. Defaulting to CPU.')
                    self.device = 'cpu'
            except ValueError:
                self.get_logger().error(f'Invalid device format: {self.device}. Defaulting to CPU.')
                self.device = 'cpu'
        else:
            self.get_logger().error(f'Invalid device specified: {self.device}. Defaulting to CPU.')
            self.device = 'cpu'

        

        self.get_logger().info('Loading model...')
        self.model_ = ODEModel_ResNet()
        self.model_.load_state_dict(torch.load(self.model_path, map_location=torch.device(self.device)))
        self.model_.to(self.device)
        self.model_.eval()
        self.get_logger().info('Model loaded successfully.')


def main(args=None):
    rclpy.init(args=args)
    node = PredictNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
