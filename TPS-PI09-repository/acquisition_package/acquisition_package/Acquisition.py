# Copyright 2023 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author:  Hugo PAGÈS, Clémence DOVILLERS, Youssef NAITALI, Hugo MORO, Carl NORGATE


# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import Open3D for easy 3d processing
import open3d as o3d


def acquisition(ply_file_path):
    '''
    Acquisition d'une image en 3d à l'aide d'une camera RGBD de la marque intel.
    
    Arg:
    ply_file_path: str, nom du fichier .ply à construire

    return:
    ply_file_path: str, nom du fichier .ply construit
    '''
    # Create a pipeline
    pipeline = rs.pipeline()

    #Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   #640*480 pixels 30fps
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)    #640*480 pixels 30fps

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)
    #Getting the depth sensor's depth laser power
    depth_sensor.set_option(rs.option.laser_power, 150)
    laser_pwr = depth_sensor.get_option(rs.option.laser_power)
    print("laser power is : ", laser_pwr, 'mW')

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Getting camera intrinsics
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)

    # Streaming loop
    num = 0
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            color_image = np.asanyarray(color_frame.get_data())
            color = o3d.geometry.Image(color_image)

            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = (depth_image < clipping_distance) * depth_image
            depth = o3d.geometry.Image(depth_image)

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

            # Generate the pointcloud and texture mappings
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity = False)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

            # Rotate
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            # Estimate Normal
            pcd.estimate_normals()

  

            # Render images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((bg_removed, depth_colormap))
            cv2.namedWindow('aligned_frame', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('aligned_frame', images)
            key = cv2.waitKey(1)
            # Press 's' to save the point cloud
            if key & 0xFF == ord('s'):
                print("Saving to Acquisition.ply".format(num))
                o3d.io.write_point_cloud(ply_file_path.format(num), pcd)
                print('Acquisition done')
                
             
                
                return ply_file_path

            # Press esc or 'q' to close the image window
            elif key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
        
