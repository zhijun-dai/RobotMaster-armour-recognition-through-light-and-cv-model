#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import os

out_dir = os.path.expanduser('~/hri_debug')
os.makedirs(out_dir, exist_ok=True)

class Verifier(Node):
    def __init__(self):
        super().__init__('verify_crop_vs_preproc')
        self.bridge = CvBridge()
        self.prep = None
        self.crop = None
        self.start_t = self.get_clock().now()
        self.create_subscription(Image, '/armor_preprocessed_cpp', self.cb_prep, 10)
        self.create_subscription(Image, '/armor_crop_result', self.cb_crop, 10)
        # timer to enforce timeout
        self.create_timer(0.5, self._check_timeout)

    def cb_prep(self, msg):
        if self.prep is None:
            self.prep = (msg.header, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
            self.get_logger().info('got preproc')
            self.try_done()

    def cb_crop(self, msg):
        if self.crop is None:
            self.crop = (msg.header, self.bridge.imgmsg_to_cv2(msg, 'bgr8'))
            self.get_logger().info('got crop')
            self.try_done()

    def try_done(self):
        if self.prep is None or self.crop is None:
            return
        # analyze preproc: find green rectangle (0,255,0 lines)
        prep_img = self.prep[1].copy()
        crop_img = self.crop[1].copy()
        # detect green edges by HSV
        hsv = cv2.cvtColor(prep_img, cv2.COLOR_BGR2HSV)
        lower = np.array([50, 150, 50])
        upper = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 50:
                continue
            r = cv2.minAreaRect(c)
            rects.append(r)
        # Dump results
        with open(os.path.join(out_dir, 'verify.txt'), 'w') as f:
            f.write(f'prep_size: {prep_img.shape}\n')
            f.write(f'crop_size: {crop_img.shape}\n')
            f.write(f'found_green_rects: {len(rects)}\n')
            for i,r in enumerate(rects):
                f.write(f'rect_{i}: center={r[0]}, size={r[1]}, angle={r[2]}\n')
        # save images
        cv2.imwrite(os.path.join(out_dir, 'preproc.png'), prep_img)
        cv2.imwrite(os.path.join(out_dir, 'crop.png'), crop_img)
        self.get_logger().info(f'saved debug images to {out_dir}')
        rclpy.shutdown()

    def _check_timeout(self):
        # exit after 6 seconds if both messages not received
        now = self.get_clock().now()
        if (now - self.start_t).nanoseconds > 6 * 1e9:
            self.get_logger().warn('timeout waiting for messages; dumping what we have')
            try:
                if self.prep is not None:
                    cv2.imwrite(os.path.join(out_dir, 'preproc_partial.png'), self.prep[1])
                if self.crop is not None:
                    cv2.imwrite(os.path.join(out_dir, 'crop_partial.png'), self.crop[1])
                with open(os.path.join(out_dir, 'verify_partial.txt'), 'w') as f:
                    f.write(f'prep_exists: {self.prep is not None}\n')
                    f.write(f'crop_exists: {self.crop is not None}\n')
            except Exception as e:
                self.get_logger().error(f'error during timeout dump: {e}')
            rclpy.shutdown()


def main():
    rclpy.init()
    node = Verifier()
    try:
        rclpy.spin(node)
    except Exception:
        pass

if __name__ == '__main__':
    main()
