#!/usr/bin/env python3
"""
ROS2 Python 节点（推理专用）：
- 订阅 /armor_crop_canvas（sensor_msgs/Image, MONO8, 20x28），加载 PyTorch 模型并做前向推理；
- 在小图左上角叠加小字预测，发布 /armor_crop_result（BGR8）。

使用方法（在 ros2 环境下）:
    ros2 run hikvision_ros2_driver armor_recognizer_node.py
或直接：
    ./armor_recognizer_node.py

注意：需要安装 rclpy、cv_bridge、torch、torchvision。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
HAS_TORCH = True
try:
    HAS_TORCH = True
    try:
        import torch
        import torch.nn as nn
        import torchvision.transforms as transforms
    except Exception:
        HAS_TORCH = False
except Exception:
    HAS_TORCH = False
import numpy as np
import os

DEFAULT_MODEL_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'best_digit_recognizer_model.pth'))

# 与 train.py 中的模型定义保持一致
if HAS_TORCH:
    class DigitRecognizer(nn.Module):
        def __init__(self, num_classes=9):
            super(DigitRecognizer, self).__init__()
            self.conv1 = nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
            self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
            self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
            self.dropout1 = nn.Dropout(0.25)
            self.dropout2 = nn.Dropout(0.5)
            self.fc1 = nn.Linear(64 * 10 * 14, 128)
            self.fc2 = nn.Linear(128, 64)
            self.fc3 = nn.Linear(64, num_classes)
            self.relu = nn.ReLU()

        def forward(self, x):
            x = self.relu(self.conv1(x))
            x = self.relu(self.conv2(x))
            x = self.pool(x)
            x = self.dropout1(x)
            x = x.view(x.size(0), -1)
            x = self.relu(self.fc1(x))
            x = self.dropout2(x)
            x = self.relu(self.fc2(x))
            x = self.fc3(x)
            return x
else:
    # Dummy placeholder so file can be imported when torch is unavailable.
    class DigitRecognizer(object):
        def __init__(self, num_classes=9):
            pass
        def to(self, device):
            return self
        def eval(self):
            pass

class ArmorRecognizerNode(Node):
    def __init__(self):
        super().__init__('armor_recognizer')
        self.bridge = CvBridge()
        # 仅订阅 C++ 端发布的 20x28 MONO8 画布
        self.crop_sub = self.create_subscription(Image, '/armor_crop_canvas', self.on_crop_canvas, 10)
        # 发布渲染结果图像：/armor_crop_result
        self.result_pub = self.create_publisher(Image, '/armor_crop_result', 10)

        # 类别映射（与 train.py 一致）
        self.idx_to_class = {
            0: '1', 1: '2', 2: '3', 3: '4', 4: '5',
            5: '6outpost', 6: '7guard', 7: '8base', 8: '9neg'
        }

        # 参数化模型路径
        self.declare_parameter('model_path', DEFAULT_MODEL_PATH)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # load model (if torch available)
        if HAS_TORCH:
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.get_logger().info(f'Loading model from: {model_path} on device {self.device}')
            self.model = DigitRecognizer(num_classes=9).to(self.device)
            if os.path.exists(model_path):
                try:
                    state = torch.load(model_path, map_location=self.device)
                    self.model.load_state_dict(state)
                except Exception as e:
                    self.get_logger().error(f'Failed to load state_dict: {e}')
            else:
                self.get_logger().warning(f'Model file not found at {model_path}, node will not perform real inference')
            self.model.eval()
        else:
            self.device = None
            self.model = None
            self.get_logger().warning('PyTorch not available: running in fallback mode; publishing input as result')
    def on_crop_canvas(self, msg: Image):
        """接收 20x28 MONO8 输入，直接推理并叠字后发布。"""
        try:
            canvas = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # 输入防御
        if canvas is None or canvas.size == 0:
            self.get_logger().warning('empty canvas received')
            return
        if canvas.shape[:2] != (20, 28):
            # 容错：若尺寸不符，按 letterbox 目标重置尺寸
            try:
                canvas = cv2.resize(canvas, (28, 20), interpolation=cv2.INTER_AREA)
            except Exception as e:
                self.get_logger().error(f'failed to resize canvas: {e}')
                return

        # 模型推理
        label = -1
        score = 0.0
        if HAS_TORCH and self.model is not None:
            try:
                inp = torch.from_numpy(canvas).float().div(255.0).unsqueeze(0).unsqueeze(0).to(self.device)
                with torch.no_grad():
                    out = self.model(inp)
                    probs = torch.softmax(out, dim=1).cpu().numpy()[0]
                    label = int(np.argmax(probs))
                    score = float(np.max(probs))
            except Exception as e:
                self.get_logger().error(f'inference failed: {e}')

        # 可视化小图：MONO -> BGR 并叠字
        vis_bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
        label_name = self.idx_to_class.get(label, str(max(0, label)))
        txt = f"{label_name} {score:.2f}"
        cv2.putText(vis_bgr, txt, (6, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        out_msg = self.bridge.cv2_to_imgmsg(vis_bgr, encoding='bgr8')
        out_msg.header = msg.header
        self.result_pub.publish(out_msg)
        self.get_logger().info('published /armor_crop_result')


def main(args=None):
    rclpy.init(args=args)
    node = ArmorRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
