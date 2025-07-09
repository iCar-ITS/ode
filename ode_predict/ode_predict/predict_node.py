import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data

import torch
import torchvision.transforms.functional as TF
import numpy as np  

import cv2

import time

from ode_predict.model import ODEModel_ResNet
from cv_bridge import CvBridge

from ode_predict.utils import crop_image

# add image and message types as needed
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32MultiArray
from boundingboxes.msg import BoundingBoxes

from ode_msgs.msg import BoundingBoxes as ode_bb
from ode_msgs.msg import BoundingBox as ode_box

from message_filters import Subscriber, Cache, TimeSynchronizer

classes_ = ["person", "bicycle", "car", "motorcycle", "truck", "bus"]

class PredictNode(Node):
    def __init__(self):
        super().__init__('predict_node')

        self.declare_parameter('model_path', 'model.pth')
        self.declare_parameter('device', 'cuda')

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        self.model_load()

        self.cv_bridge = CvBridge()

        # Sync image and bounding boxes

        self.bb_subs_ = Subscriber(self, BoundingBoxes, '/bounding_boxes', qos_profile=qos_profile_sensor_data)
        self.img_subs_ = Subscriber(self, Image, '/image', qos_profile=qos_profile_sensor_data)

        # Create synchronizer
        self.sync_ = TimeSynchronizer([self.bb_subs_, self.img_subs_], 10)
        self.sync_.registerCallback(self.callback)

        # self.cache_ = Cache(self.img_subs_, 1000)

        # self.bb_subs_.registerCallback(self.callback)

        self.bb_pubs_ = self.create_publisher(ode_bb, '/predicted/bb', qos_profile=qos_profile_sensor_data)
        self.img_pub = self.create_publisher(Image, '/predicted/image', qos_profile=qos_profile_sensor_data)

        self.get_logger().info('ODE Predict Node has been started.')

    def callback(self, bb_msg, img_msg):

        img_header = img_msg.header

        # convert ros image to cv2 image
        img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        if img is None:
            self.get_logger().error('Failed to convert ROS image to OpenCV image.')
            return
        
        img = crop_image(img, (832, 480))

        bb_proc = []

        if bb_msg.bounding_boxes:

            xywh = []

            for bb in bb_msg.bounding_boxes:
                if bb.xmax <= bb.xmin or bb.ymax <= bb.ymin:
                    # self.get_logger().warn('Invalid bounding box detected. Skipping this bounding box.')
                    continue

                if bb.class_id not in classes_:
                    # self.get_logger().warn(f'Unknown class_id {bb.class_id} detected. Skipping this bounding box.')
                    continue

                # convert bounding boxes to yolo form
                x_c = (bb.xmax + bb.xmin) / 2.0
                y_c = (bb.ymax + bb.ymin) / 2.0

                ratio = img.shape[1] / img_msg.width  # width ratio
                x_c = x_c * ratio
                y_c = y_c * ratio

                w_bb = (bb.xmax - bb.xmin) * ratio
                h_bb = (bb.ymax - bb.ymin) * ratio

                xywh_p = [x_c, y_c, w_bb, h_bb]
                xywh.append(xywh_p)

                bb_proc.append([
                    bb.class_id, 
                    bb.xmax * ratio, 
                    bb.xmin * ratio, 
                    bb.ymax * ratio, 
                    bb.ymin * ratio,
                    bb.id])
                # self.get_logger().info(f'Bounding box: {bb.class_id} at {pt} with width {w_bb} and height {h_bb}')

            b_xywh = []
            b_xywh.append(torch.tensor(xywh).to(self.device, dtype=torch.float32))

            if b_xywh[0].numel():
                               
                # convert image to tensor
                # img_a = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img_a = img.copy()  # Use a copy of the image to avoid modifying the original
                # img_tensor = torch.tensor(img, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0)
                img_tensor = TF.to_tensor(img_a).unsqueeze(0)  # Convert to tensor and add batch dimension
                img_tensor = TF.normalize(img_tensor, mean=[0.485, 0.456, 0.406],
                                            std=[0.229, 0.224, 0.225])
                img_tensor = img_tensor.to(self.device)
            
                # self.get_logger().info(f'Processing {img_tensor.size()} bounding boxes.')

                # torch.cuda.synchronize()
                # start_time = time.time()

                pred = self.model_(img_tensor, b_xywh)

                # torch.cuda.synchronize()
                # end_time = time.time()
                # elapsed = end_time - start_time

                # self.get_logger().info(f'Inference time: {elapsed:.4f} seconds')

                # convert prediction to numpy array
                pred_np = [p.detach().cpu().numpy() for p in pred]
                pred_np = np.squeeze(pred_np, axis=0)  # Remove batch dimension

                # Create a new BoundingBoxes message
                new_bb_msg = ode_bb()
                new_bb_msg.header = img_header
                new_bb_msg.bounding_boxes = []

                for i, bb in enumerate(bb_proc):
                    # Create a new bounding box with the prediction
                    new_bb = ode_box()
                    new_bb.x = float(b_xywh[0][i,0])
                    new_bb.y = float(b_xywh[0][i,0])
                    new_bb.width = float(b_xywh[0][i,0])
                    new_bb.height = float(b_xywh[0][i,0])
                    new_bb.id = bb[5]
                    new_bb.class_id = bb[0]
                    new_bb.distance = float(pred_np[i])

                    # Add the new bounding box to the message
                    new_bb_msg.bounding_boxes.append(new_bb)

                # Publish the new bounding boxes message
                self.bb_pubs_.publish(new_bb_msg)

                # Attach bounding boxes to the image for visualization
                img = self.attach_bb_to_img(img, bb_proc, new_bb_msg)
        
        # Convert the image back to ROS Image message
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header = img_header # Use the same header as the original image 

        # Publish the image with bounding boxes
        self.img_pub.publish(img_msg)

    def attach_bb_to_img(self, img, bb, pred):
        """
        Attach bounding boxes to the image for visualization.
        """
        if pred is None:
            self.get_logger().error('Invalid image or bounding boxes. Cannot attach bounding boxes to the image.')
            return img
        for box, p in zip(bb, pred.bounding_boxes):
            if box[1] <= box[2] or box[3] <= box[4]:
                self.get_logger().warn('Invalid bounding box detected. Skipping this bounding box.')
                continue

            h, w, _ = img.shape
            
            cv2.rectangle(img, (int(box[2]), int(box[4])), (int(box[1]), int(box[3])), (0, 255, 0), 2)

            # draw label with background
            cv2.rectangle(img, (max(0, int(box[2])-1), int(box[4]) - 15), (min(max(int(box[1])+1, int(box[2]) + 50), w), int(box[4])), (0, 255, 0), -1)
            cv2.putText(img, f'{p.distance:.2f}m', (int(box[2]), int(box[4]) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
        
        return img

    #@torch.no_grad()
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
        torch.set_grad_enabled(False)
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
