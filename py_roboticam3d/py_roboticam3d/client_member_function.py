# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author:  Hugo PAGÈS, Clémence DOVILLERS, Youssef NAITALI, Hugo MORO, Carl NORGATE

import sys

from interface_roboticam3d.srv import CreationFichierPly
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(CreationFichierPly, 'creation_fichier_ply')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = CreationFichierPly.Request()

    def send_request(self, a):
        self.req.a = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]))
    minimal_client.get_logger().info(
        (int(sys.argv[1]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
