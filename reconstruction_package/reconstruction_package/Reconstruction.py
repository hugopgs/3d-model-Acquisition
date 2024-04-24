# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author:  Hugo PAGÈS, Clémence DOVILLERS, Youssef NAITALI, Hugo MORO, Carl NORGATE

import open3d as o3d
import numpy as np
import time
class PLYConverter:
    @staticmethod
    def convert_to_point_cloud(file_path):
        """
        Convert a PLY file to a point cloud.

        Args:
            file_path (str): Path to the PLY file.

        Returns:
            open3d.geometry.PointCloud: Point cloud object.
        """
        pcd = o3d.io.read_point_cloud(file_path)
        o3d.visualization.draw_geometries([pcd])
        print("visualisation pcd")
        # Step 3: Return the resulting Point Cloud (PCD)
        return pcd

class PCDConverter:
    @staticmethod
    def convert_to_mesh(pcd):
        """
        Convert a point cloud to a mesh.

        Args:
            pcd (open3d.geometry.PointCloud): Point cloud object.

        Returns:
            open3d.geometry.TriangleMesh: Mesh object.
        """
        # Estimate normals if not available
        if not pcd.has_normals():
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
          # Orient normals consistently
        if pcd.has_normals():
            pcd.orient_normals_towards_camera_location()
        # Remove statistical outliers
        pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        # Remove radius outliers
        pcd.remove_radius_outlier(nb_points=30, radius=0.01)
        # Poisson surface reconstruction
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9, n_threads=1)
        vertices_to_remove = densities < np.quantile(densities, 0.06)
        mesh.remove_vertices_by_mask(vertices_to_remove)
        # Smooth mesh
        mesh.filter_smooth_taubin(number_of_iterations=100)
        mesh.compute_vertex_normals()
        print("convert to mesh...")
        return mesh
    
    
class MESHConverter:
    def __init__(self, file_path):
        """
        Initialize the converter with a file path.

        Args:
            file_path (str): Path to the input PLY file.
        """
        # Convert PLY to point cloud
        print('initialize')
        pcd = PLYConverter.convert_to_point_cloud(file_path)
        pcd = pcd.voxel_down_sample(voxel_size=0.001)
        # Convert point cloud to mesh
        print("PCD converter")
        self.mesh = PCDConverter.convert_to_mesh(pcd) if pcd else None

    def convert_to_stl(self, stl_file_path):
        """
        Convert mesh to STL format and save.

        Args:
            stl_file_path (str): Path to save the resulting STL file.
        """
        start = time.time()
        if self.mesh:
            # Write mesh to STL file
            o3d.io.write_triangle_mesh(stl_file_path, self.mesh)
            print('Temps de reconstruction = ', time.time()-start)
            print(f"Mesh successfully saved to {stl_file_path}")
            # Visualize mesh
            o3d.visualization.draw_geometries([self.mesh])
        else:
            print("Error: Mesh is None. Conversion failed.")


def reconstruction(ply_file_path,stl_file_path):
    converter = MESHConverter(ply_file_path)
    print("Converting Acquisition.ply to Reconstruction.stl...")
    converter.convert_to_stl(stl_file_path)
    print("Reconstruction done")
