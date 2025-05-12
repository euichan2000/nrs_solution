import pyrealsense2 as rs
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

# Initialize ROS node
rospy.init_node("stream_realsense_camera_node")
rgb_pub = rospy.Publisher("camera/rgb/image_raw", Image, queue_size=10)
depth_pub = rospy.Publisher("camera/depth/image_raw", Image, queue_size=10)
points_pub = rospy.Publisher("camera/depth/points", PointCloud2, queue_size=10)

# Initialize CvBridge
bridge = CvBridge()

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# Create RealSense alignment object to align depth to color
align_to = rs.stream.color
align = rs.align(align_to)

# Start streaming
pipeline_profile = pipeline.start(config)

# Get depth scale
depth_sensor = pipeline_profile.get_device().first_depth_sensor()
# Apply settings
depth_sensor.set_option(rs.option.visual_preset, rs.rs400_visual_preset.high_accuracy)
depth_sensor.set_option(rs.option.alternate_ir, 0.0)
depth_sensor.set_option(rs.option.confidence_threshold, 3.0)
depth_sensor.set_option(rs.option.digital_gain, 2.0)
depth_sensor.set_option(rs.option.enable_ir_reflectivity, 0.0)
depth_sensor.set_option(rs.option.enable_max_usable_range, 0.0)
depth_sensor.set_option(rs.option.error_polling_enabled, 1.0)
depth_sensor.set_option(rs.option.frames_queue_size, 16.0)
depth_sensor.set_option(rs.option.freefall_detection_enabled, 1.0)
depth_sensor.set_option(rs.option.global_time_enabled, 0.0)
depth_sensor.set_option(rs.option.inter_cam_sync_mode, 0.0)
depth_sensor.set_option(rs.option.invalidation_bypass, 0.0)
depth_sensor.set_option(rs.option.laser_power, 0.0)
depth_sensor.set_option(rs.option.min_distance, 0.0)
depth_sensor.set_option(rs.option.noise_filtering, 6.0)
depth_sensor.set_option(rs.option.post_processing_sharpening, 2.0)
depth_sensor.set_option(rs.option.pre_processing_sharpening, 0.0)
depth_sensor.set_option(rs.option.visual_preset, 0.0)  # Overwrites high_accuracy above
pc = rs.pointcloud()

# Create RealSense alignment object to align depth to color
align_to = rs.stream.color
align = rs.align(align_to)
depth_scale = depth_sensor.get_depth_scale()
rospy.loginfo(f"Depth Scale: {depth_scale}")

# Main loop
rate = rospy.Rate(30)  # 30 Hz
try:
    while not rospy.is_shutdown():
        # Wait for frames
        frames = pipeline.wait_for_frames()

        # Align the depth frame to the color frame
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        aligned_depth_frame = aligned_frames.get_depth_frame()

        # Validate frames
        if not color_frame or not aligned_depth_frame:
            rospy.logwarn("Failed to capture aligned frames from RealSense camera.")
            continue

        # Convert RealSense frames to OpenCV format
        color_image = np.asanyarray(color_frame.get_data())
        aligned_depth_image = np.asanyarray(aligned_depth_frame.get_data())

        # Convert OpenCV images to ROS messages
        color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        aligned_depth_msg = bridge.cv2_to_imgmsg(aligned_depth_image, encoding="16UC1")

        # Publish messages
        rgb_pub.publish(color_msg)
        depth_pub.publish(aligned_depth_msg)

        pc.map_to(color_frame)
        points = pc.calculate(aligned_depth_frame)

        # 포인트 데이터를 numpy array로 변환
        # 각 포인트는 (x, y, z) 형태를 가지며, 총 width*height 개의 포인트를 포함함
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        # PointCloud2 메시지를 위한 헤더 설정
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_frame" 

        # 포인트 클라우드 메시지 생성 (x, y, z만 포함)
        pc_msg = pc2.create_cloud_xyz32(header, vtx.tolist())
        points_pub.publish(pc_msg)
        rate.sleep()

finally:
    # Stop streaming
    pipeline.stop()