# Copyright 2017 Muhammad Furqan Habibi
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
from io import BytesIO
from math import modf
from time import time
from picamera import PiCamera
from camera_info_manager import CameraInfoManager

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    Image,
    CameraInfo
)
from builtin_interfaces.msg import Time

class FrameProcessor(object):
    def __init__(self, node):
        self._node = node
        self._frames = 0
        self._time = modf(time())[1]

    def write(self, frame):
        self._node.publish_image(frame)
        if (time() - self._time < 1):
            self._frames += 1
        else:
            print('fps = ' + str(self._frames))
            self._frames = 0
            self._time = modf(time())[1]

    def flush(self):
        pass

class RaspicamRos2(Node):

    def __init__(self, args):
        super().__init__('raspicam_ros2')

        # vars
        self._topic_name = 'raspicam'
        if(len(args)>1):
            self._topic_name = args[1]
        self._camera_info_manager = CameraInfoManager(self, 'raspicam', namespace='/' + self._topic_name)
        camera_info_url = 'file://' + os.path.dirname(os.path.abspath(__file__)) + '/config/raspicam_416x320.yaml'

        # pubs
        self._img_pub = self.create_publisher(Image, '/' + self._topic_name + '/image', qos_profile=rclpy.qos.qos_profile_sensor_data)
        self._camera_info_pub = self.create_publisher(CameraInfo, '/' + self._topic_name + '/camera_info')

        # camera info manager
        self._camera_info_manager.setURL(camera_info_url)
        self._camera_info_manager.loadCameraInfo()
    
    def _get_stamp(self, time_float):
        time_frac_int = modf(time_float)
        stamp = Time()
        stamp.sec = int(time_frac_int[1])
        stamp.nanosec = int(time_frac_int[0] * 1000000000) & 0xffffffff
        return stamp

    def publish_image(self, image):
        img = Image()
        img.encoding = 'rgb8'
        img.width = 416
        img.height = 320
        img.step = img.width * 3
        img.data = image
        img.header.frame_id = 'raspicam'
        img.header.stamp = self._get_stamp(time())
        self._img_pub.publish(img)
        camera_info = self._camera_info_manager.getCameraInfo()
        camera_info.header = img.header
        self._camera_info_pub.publish(camera_info)

    def run(self):
        with PiCamera() as camera:
            camera.resolution = (416, 320)
            # Construct the frame processor and start recording data to it
            frame_processor = FrameProcessor(self)
            camera.start_recording(frame_processor, 'rgb')
            try:
                while True:
                    camera.wait_recording(1)
            finally:
                camera.stop_recording()


def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    raspicam_ros2 = RaspicamRos2(args)

    raspicam_ros2.run()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    raspicam_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
