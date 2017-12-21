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

class RaspicamRos2(Node):

    def __init__(self):
        super().__init__('raspicam_ros2')

        # timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # vars
        self._camera = PiCamera()
        self._camera.resolution = (416, 320)
        #self._camera.start_preview()
        self._camera_info_manager = CameraInfoManager(self, 'raspicam', namespace='/raspicam')
        camera_info_url = 'file://' + os.path.dirname(os.path.abspath(__file__)) + '/config/raspicam_410x308.yaml'

        # pubs
        self._img_pub = self.create_publisher(Image, '/raspicam/image')
        self._camera_info_pub = self.create_publisher(CameraInfo, 'raspicam/camera_info')

        # camera info manager
        self._camera_info_manager.setURL(camera_info_url)
        self._camera_info_manager.loadCameraInfo()

    
    def _get_stamp(self, time_float):
        time_frac_int = modf(time_float)
        stamp = Time()
        stamp.sec = int(time_frac_int[1])
        stamp.nanosec = int(time_frac_int[0] * 1000000000) & 0xffffffff
        return stamp

    def timer_callback(self):
        cam_stream = BytesIO()
        self._camera.capture(cam_stream, 'rgb')
        img = Image()
        img.encoding = 'rgb8'
        img.width = 416
        img.height = 320
        img.step = img.width * 3
        img.data = cam_stream.getvalue()
        img.header.frame_id = 'raspicam'
        img.header.stamp = self._get_stamp(time())
        self._img_pub.publish(img)
        camera_info = self._camera_info_manager.getCameraInfo()
        camera_info.header = img.header
        self._camera_info_pub.publish(camera_info)
        cam_stream.close()
        print(img.header.stamp)


def main(args=None):
    rclpy.init(args=args)

    raspicam_ros2 = RaspicamRos2()

    rclpy.spin(raspicam_ros2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    raspicam_ros2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
