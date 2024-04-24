# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author:  Hugo PAGÈS, Clémence DOVILLERS, Youssef NAITALI, Hugo MORO, Carl NORGATE

from acquisition_package import Acquisition as acq
from reconstruction_package import Reconstruction as rec
import open3d as o3d
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
#Times access and conversions
import time
import tkinter as tk

from interface_roboticam3d.srv import CreationFichierPly

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(CreationFichierPly, 'creation_fichier_ply', self.creation_fichier_ply_callback)

    def creation_fichier_ply_callback(self, request, response):
        response.sum = request.a
        self.get_logger().info('Incoming request\na: %d' % (request.a))
        rec.reconstruction(acq.acquisition('./Data/Acquisition.ply'),'./Data/Reconstruction.stl')
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

