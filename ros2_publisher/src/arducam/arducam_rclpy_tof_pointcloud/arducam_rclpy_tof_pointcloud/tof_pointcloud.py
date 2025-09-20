"""
Arducam TOF Camera ROS2 Point Cloud Publisher

This ROS2 node publishes depth data from the Arducam TOF camera as both point cloud messages
and raw depth arrays. It provides real-time 3D point cloud data that can be used with
ROS2 visualization tools like RViz2 and other ROS2 applications.

Features:
- Real-time point cloud publishing via ROS2
- Raw depth data publishing for custom processing
- Multi-threaded processing for optimal performance
- Support for both VGA and HQVGA camera types
- Configurable camera settings via command line
- Automatic coordinate system transformation

Published Topics:
- /point_cloud (sensor_msgs/PointCloud2): 3D point cloud data
- /depth_frame (std_msgs/Float32MultiArray): Raw depth data array

Author: Arducam
Version: 1.0
Date: 2024
"""

from argparse import ArgumentParser
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header
import numpy as np
from threading import Thread


from ArducamDepthCamera import (
    ArducamCamera,
    Connection,
    DeviceType,
    FrameType,
    Control,
    DepthData,
)

# ============================================================================
# Configuration Classes
# ============================================================================

class Option:
    """
    Configuration options for the TOF camera publisher.
    
    Attributes:
        cfg (Optional[str]): Path to camera configuration file
    """
    cfg: Optional[str]
    

# ============================================================================
# ROS2 Node Class
# ============================================================================

class TOFPublisher(Node):
    """
    ROS2 node for publishing Arducam TOF camera data as point clouds.
    
    This node initializes the TOF camera, processes depth data in a separate thread,
    and publishes both point cloud and raw depth data via ROS2 topics.
    """
    
    def __init__(self, options: Option):
        """
        Initialize the TOF publisher node.
        
        Args:
            options (Option): Configuration options for the camera
        """
        super().__init__("arducam")

        # Initialize camera
        tof = self.__init_camera(options)
        if tof is None:
            raise Exception("Failed to initialize camera")

        # Store camera reference
        self.tof_ = tof
        
        # Calculate point cloud size
        self.pointsize_ = self.width_ * self.height_
        
        # ROS2 message setup
        self.frame_id = "sensor_frame"
        self.depth_msg_ = Float32MultiArray()
        
        # Create ROS2 publishers
        self.publisher_ = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.publisher_depth_ = self.create_publisher(
            Float32MultiArray, "depth_frame", 10
        )
        
        # Get camera intrinsic parameters
        self.fx = tof.getControl(Control.INTRINSIC_FX) / 100
        self.fy = tof.getControl(Control.INTRINSIC_FY) / 100
        
        # Setup message header
        self.header = Header()
        self.header.frame_id = "map"
        
        # Point cloud data storage
        self.points = None
        
        # Threading control
        self.running_ = True
        
        # Create timer for publishing (30 Hz)
        self.timer_ = self.create_timer(1 / 30, self.update)
        
        # Start point cloud processing thread
        self.process_point_cloud_thr = Thread(
            target=self.__generateSensorPointCloud, daemon=True
        )
        self.process_point_cloud_thr.start()

    def __init_camera(self, options: Option):
        """
        Initialize the Arducam TOF camera.
        
        This method opens the camera connection, starts depth frame capture,
        and configures camera settings based on the device type.
        
        Args:
            options (Option): Configuration options for the camera
            
        Returns:
            ArducamCamera: Initialized camera object, or None if initialization failed
        """
        print("pointcloud publisher init")
        tof = ArducamCamera()
        ret = 0
        
        # Open camera connection
        if options.cfg is not None:
            ret = tof.openWithFile(options.cfg, 0)
        else:
            ret = tof.open(Connection.CSI, 0)
        if ret != 0:
            print("Failed to open camera. Error code:", ret)
            return None

        # Start depth frame capture
        ret = tof.start(FrameType.DEPTH)
        if ret != 0:
            print("Failed to start camera. Error code:", ret)
            tof.close()
            return None

        # Get camera information and configure based on device type
        info = tof.getCameraInfo()
        if info.device_type == DeviceType.HQVGA:
            # HQVGA camera configuration
            self.width_ = info.width
            self.height_ = info.height
            tof.setControl(Control.RANGE, 4)  # Set 4-meter range
        elif info.device_type == DeviceType.VGA:
            # VGA camera configuration
            self.width_ = info.width
            self.height_ = info.height // 10 - 1  # Adjust height for VGA
        print(f"Open camera success, width: {self.width_}, height: {self.height_}")

        print("Pointcloud publisher start")
        return tof

    def __generateSensorPointCloud(self):
        """
        Generate point cloud data from depth frames in a separate thread.
        
        This method runs continuously in a background thread, processing depth frames
        from the camera and converting them to 3D point cloud coordinates. It applies
        confidence filtering and coordinate system transformations.
        """
        while self.running_:
            # Request a new frame from the camera (200ms timeout)
            frame = self.tof_.requestFrame(200)
            if frame is not None and isinstance(frame, DepthData):
                # Update camera intrinsic parameters
                self.fx = self.tof_.getControl(Control.INTRINSIC_FX) / 100
                self.fy = self.tof_.getControl(Control.INTRINSIC_FY) / 100
                
                # Extract depth and confidence data
                depth_buf = frame.depth_data
                confidence_buf = frame.confidence_data

                # Apply confidence filtering (remove pixels with confidence < 30)
                depth_buf[confidence_buf < 30] = 0

                # Prepare depth message data (convert mm to meters)
                self.depth_msg_.data = depth_buf.flatten() / 1000

                # Convert depth values from millimeters to meters
                z = depth_buf / 1000.0
                z[z <= 0] = np.nan  # Handle invalid depth values

                # Create coordinate grids
                u = np.arange(self.width_)
                v = np.arange(self.height_)
                u, v = np.meshgrid(u, v)

                # Calculate 3D point cloud coordinates using camera intrinsics
                # x = (pixel_x - cx) * depth / fx
                # y = (pixel_y - cy) * depth / fy
                x = (u - self.width_ / 2) * z / self.fx
                y = (v - self.height_ / 2) * z / self.fy

                # Combine coordinates into point cloud
                points = np.stack((x, y, z), axis=-1)
                
                # Filter out invalid points (NaN values)
                self.points = points[~np.isnan(points).any(axis=-1)]

                # Release frame buffer
                self.tof_.releaseFrame(frame)

    def update(self):
        """
        Update method called by the ROS2 timer to publish point cloud data.
        
        This method is called at 30 Hz to publish the latest point cloud and depth data
        to ROS2 topics. It creates PointCloud2 messages from the processed point data.
        """
        # Check if we have valid point cloud data
        if self.points is None:
            return
            
        # Update message timestamp
        self.header.stamp = self.get_clock().now().to_msg()

        # Create PointCloud2 message from 3D points
        pc2_msg_ = point_cloud2.create_cloud_xyz32(self.header, self.points)

        # Publish both point cloud and raw depth data
        self.publisher_.publish(pc2_msg_)
        self.publisher_depth_.publish(self.depth_msg_)

    def stop(self):
        """
        Stop the camera and clean up resources.
        
        This method stops the background processing thread, stops the camera,
        and closes the camera connection.
        """
        # Signal the processing thread to stop
        self.running_ = False
        
        # Wait for the processing thread to finish
        self.process_point_cloud_thr.join()
        
        # Stop and close the camera
        self.tof_.stop()
        self.tof_.close()


# ============================================================================
# Main Application
# ============================================================================

def main(args=None):
    """
    Main function for the ROS2 TOF camera publisher node.
    
    This function initializes ROS2, parses command line arguments, creates the
    TOF publisher node, and runs the ROS2 event loop.
    
    Args:
        args: Command line arguments (optional)
    """
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = ArgumentParser()
    parser.add_argument("--cfg", type=str, help="Path to camera configuration file")
    
    ns = parser.parse_args()
    
    # Create configuration options
    options = Option()
    options.cfg = ns.cfg
    
    # Create and run the TOF publisher node
    tof_publisher = TOFPublisher(options)

    try:
        # Run the ROS2 event loop
        rclpy.spin(tof_publisher)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up resources
        tof_publisher.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
