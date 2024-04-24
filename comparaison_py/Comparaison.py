# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author:  Hugo PAGÈS, Clémence DOVILLERS, Youssef NAITALI, Hugo MORO, Carl NORGATE



import numpy as np
import open3d as py3d

def remove_background(point_cloud1, point_cloud2, tolerance=1e-6):
    # Create a copy of the point cloud to avoid modifying the original
    modified_point_cloud = np.copy(point_cloud1.points)

    # Iterate over each point in point_cloud2
    for point2 in point_cloud2.points:
        # Find the indices of points in point_cloud1 that are close to point2
        distances = np.linalg.norm(modified_point_cloud - point2, axis=1)
        close_indices = np.where(distances < tolerance)[0]

        # Remove the close points from modified_point_cloud
        modified_point_cloud = np.delete(modified_point_cloud, close_indices, axis=0)

    return modified_point_cloud


def register_and_visualize_stl(stl_file_path_1, stl_file_path_2,stl_file_path_3='',  matching_threshold=0.004):
    """
    Register and visualize two STL files.

    Args:
        stl_file_path_1 (str): Path to the first STL file.
        stl_file_path_2 (str): Path to the second STL file.
        matching_threshold (float): Threshold for the distance between points to be considered matching.
    """
    # Read STL files
    pcd3 = None
    try:
        pcd1 = py3d.io.read_triangle_mesh(stl_file_path_1).sample_points_poisson_disk(number_of_points=200000)
        pcd2 = py3d.io.read_triangle_mesh(stl_file_path_2).sample_points_poisson_disk(number_of_points=200000)
        if stl_file_path_3!='':
            pcd3 = py3d.io.read_triangle_mesh(stl_file_path_3).sample_points_poisson_disk(number_of_points=10000)
    except Exception as e:
        print("Error:", e)
        return
    
    if len(pcd1.points) == 0 or len(pcd2.points) == 0:
        print("Error: Empty point clouds")
        return

    if pcd3:
        pcd1 = remove_background(pcd1, pcd3, tolerance=0.02)
        pcd2 = remove_background(pcd2, pcd3, tolerance=0.02)


    # Set colors for point clouds
    pcd1.colors = py3d.utility.Vector3dVector(np.repeat(np.asarray([[1, 0, 0]]), len(np.asarray(pcd1.points)), axis=0))
    pcd2.colors = py3d.utility.Vector3dVector(np.repeat(np.asarray([[0, 0, 1]]), len(np.asarray(pcd2.points)), axis=0))

    # Initial transformation matrix
    T = np.identity(4)

    # Initial registration using evaluate_registration
    #info = py3d.pipelines.registration.evaluate_registration(pcd1, pcd2, max_correspondence_distance=matching_threshold, transformation=T)
    #print("Evaluation result for initial registration:")
    #print("correspondences:", np.asarray(info.correspondence_set))
    #print("fitness: ", info.fitness)
    #print("RMSE: ", info.inlier_rmse)
    #print("transformation: ", info.transformation)

    # ICP registration
    info = py3d.pipelines.registration.registration_icp(pcd1, pcd2, max_correspondence_distance=matching_threshold, init=T,
                                                        estimation_method=py3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True))
    print("Evaluation result after ICP registration:")
    print("correspondences:", np.asarray(info.correspondence_set))
    print("fitness: ", info.fitness)
    print("RMSE: ", info.inlier_rmse)
    print("transformation: ", info.transformation)

    # Transform first point cloud according to the registration result
    pcd1.transform(info.transformation)

    # Get the matched points
    matched_points = []
    for pair in info.correspondence_set:
        matched_points.append([pcd1.points[pair[0]], pcd2.points[pair[1]]])

    # Convert matched points to Open3D format
    matched_pcd = py3d.geometry.PointCloud()
    for pair in matched_points:
        matched_pcd.points.append(pair[0])
        matched_pcd.colors.append([0, 1, 0])  # Set color to green for matched points
        matched_pcd.points.append(pair[1])
        matched_pcd.colors.append([0, 1, 0])  # Set color to green for matched points

    # Calculate percentage of matching points
    num_matching_points = len(matched_points)
    total_points = max(len(pcd1.points),len(pcd2.points))
    matching_percentage = (num_matching_points / total_points) * 100
    print("Percentage of matching points:", matching_percentage)

    # Visualize registered point clouds and matching points
    py3d.visualization.draw_geometries([pcd1, pcd2, matched_pcd], "Registered STL files with matching points", 640, 480)


register_and_visualize_stl("./test_integration/fichier_stl_reference.stl", "./Data/Reconstruction.stl")
