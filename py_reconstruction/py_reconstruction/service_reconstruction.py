from reconstruction_package import Reconstruction as rec
from interface_reconstruction.srv import Reconstruction
import open3d as o3d
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Reconstruction, 'reconstruction', self.reconstruction_callback)

    def reconstruction_callback(self, request, response):
        response.sum = request.a
        self.get_logger().info('Incoming request\na: %d' % (request.a))
        rec.reconstruction('./Data/Acquisition.ply', './Data/Reconstruction.stl')
        print("Done")

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
