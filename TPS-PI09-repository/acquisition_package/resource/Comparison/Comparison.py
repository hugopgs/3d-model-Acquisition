import numpy as np
import open3d as py3d

def register_and_visualize_stl(stl_file_path_1, stl_file_path_2, matching_threshold=0.02):
    """
    Register and visualize two STL files.

    Args:
        stl_file_path_1 (str): Path to the first STL file.
        stl_file_path_2 (str): Path to the second STL file.
        matching_threshold (float): Threshold for the distance between points to be considered matching.
    """
    # Read STL files
    try:
        pcd1 = py3d.io.read_triangle_mesh(stl_file_path_1).sample_points_poisson_disk(number_of_points=10000)
        pcd2 = py3d.io.read_triangle_mesh(stl_file_path_2).sample_points_poisson_disk(number_of_points=10000)
    except Exception as e:
        print("Error:", e)
        return
    
    if len(pcd1.points) == 0 or len(pcd2.points) == 0:
        print("Error: Empty point clouds")
        return

    # Set colors for point clouds
    pcd1.colors = py3d.utility.Vector3dVector(np.repeat(np.asarray([[1, 0, 0]]), len(np.asarray(pcd1.points)), axis=0))
    pcd2.colors = py3d.utility.Vector3dVector(np.repeat(np.asarray([[0, 0, 1]]), len(np.asarray(pcd2.points)), axis=0))

    # Initial transformation matrix
    T = np.identity(4)

    # Initial registration using evaluate_registration
    info = py3d.pipelines.registration.evaluate_registration(pcd1, pcd2, max_correspondence_distance=matching_threshold, transformation=T)
    print("Evaluation result for initial registration:")
    print("correspondences:", np.asarray(info.correspondence_set))
    print("fitness: ", info.fitness)
    print("RMSE: ", info.inlier_rmse)
    print("transformation: ", info.transformation)

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



# Example usage:
register_and_visualize_stl("/home/naitali/TPS-PI09-repository/Comparison/CS96-25.stl", 
                            "/home/naitali/TPS-PI09-repository/Comparison/test2.stl", matching_threshold=0.02)






