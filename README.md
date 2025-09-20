# Arducam TOF Depth Camera SDK

## Overview

This project provides a comprehensive SDK and example applications for the Arducam Time-of-Flight (TOF) depth camera. It includes real-time depth visualization, 3D point cloud generation, and ROS2 integration for robotics applications.

### Camera Specifications

- **Resolution**: 240×180 pixels (HQVGA) / 640×480 pixels (VGA)
- **Range Modes**: 2 meters and 4 meters
- **Measurement Accuracy**: Within 2 cm
- **Connection Methods**: CSI and USB interfaces
- **Power Requirements**: 5V 2A external power supply
- **Technology**: Phase-difference based TOF measurement using modulated light pulses

### Key Features

- **Real-time Depth Visualization**: Live depth data display with confidence filtering
- **3D Point Cloud Generation**: Convert depth data to 3D point clouds using Open3D
- **ROS2 Integration**: Publish depth data and point clouds via ROS2 topics
- **Multi-language Support**: C++, Python, and C examples
- **Interactive Controls**: Mouse selection for distance measurement
- **Data Export**: Save depth data and point clouds to files
- **Cross-platform**: Support for Raspberry Pi, Jetson, and other Linux platforms

## Quick Start

### Prerequisites

- Linux-based system (Raspberry Pi, Jetson, or compatible platform)
- Python 3.6+ (for Python examples)
- CMake 3.10+ (for C++ compilation)
- OpenCV 4.0+ (for image processing)
- Open3D (for 3D point cloud visualization)
- ROS2 (for ROS2 integration)

### Installation

#### 1. Clone the Repository

```bash
git clone https://github.com/ArduCAM/Arducam_tof_camera.git
cd Arducam_tof_camera
```

#### 2. Install Dependencies

**For Raspberry Pi:**
```bash
./Install_dependencies.sh
```

**For Rock 5A platform:**
```bash
./setup_rock_5a.sh
```

**For Jetson platforms:**
```bash
./Install_dependencies_jetson.sh
```

#### 3. Hardware Setup

1. Connect the TOF camera to your device via CSI or USB interface
2. Connect the external 5V 2A power supply to the camera
3. Ensure proper camera mounting and positioning

## Running Examples

### Depth Visualization Examples

#### Python Examples

Navigate to the Python examples directory:
```bash
cd example/python
```

**Depth Preview with Interactive Controls:**
```bash
python3 preview_depth.py
```
- Real-time depth visualization with confidence filtering
- Mouse interaction for distance measurement
- Dynamic confidence threshold adjustment (VGA cameras only)
- Press 'q' to exit

**Raw Data Capture:**
```bash
python3 capture_raw.py
```
- Display raw sensor data before depth processing
- Useful for debugging and calibration
- Press 'q' to exit

#### C/C++ Examples

**Compile the examples:**
```bash
./compile.sh
```

**Run C++ Depth Preview:**
```bash
cd build/example/cpp
./preview_depth
```
- Interactive depth visualization with mouse selection
- FPS monitoring
- Press 's' to save depth data, 'q' or ESC to exit

**Run C++ Raw Data Capture:**
```bash
cd build/example/cpp
./capture_raw
```
- Raw sensor data visualization
- Press 'q' or ESC to exit

**Run C Example:**
```bash
cd build/example/c
./preview_depth_c
```
- Basic depth frame capture and display

### 3D Point Cloud Visualization

#### Prerequisites

Install Open3D development libraries:
```bash
sudo apt update
sudo apt-get install libopen3d-dev
```

#### Compile Point Cloud Example

```bash
./compile_pointcloud.sh
```

#### Run 3D Point Cloud Preview

```bash
cd build/open3d_preview
./preview_pointcloud
```

**Features:**
- Real-time 3D point cloud visualization using Open3D
- Interactive depth preview window
- Point cloud saving functionality (PCD format)
- Depth data saving functionality (RAW format)
- Dynamic confidence threshold adjustment
- Mouse interaction for region selection

**Controls:**
- **ESC or 'q'**: Exit application
- **'s'**: Save current point cloud as PCD file
- **'r'**: Save current depth data as RAW file
- **'+/-' or '.,'**: Adjust confidence threshold
- **'[/]'**: Adjust confidence threshold by 5
- **Mouse**: Select regions for distance measurement

**Troubleshooting:**
If you don't see the point cloud window, try setting the environment variable:
```bash
export MESA_GL_VERSION_OVERRIDE=4.5
./preview_pointcloud
```

### ROS2 Integration

#### Prerequisites

- ROS2 (Humble, Foxy, or compatible version)
- Python packages: `rclpy`, `sensor_msgs`, `sensor_msgs_py`

#### Build ROS2 Package

```bash
cd ros2_publisher
./build.sh
```

#### Run ROS2 Publisher

```bash
./run.sh
```

**Published Topics:**
- `/point_cloud` (sensor_msgs/PointCloud2): 3D point cloud data
- `/depth_frame` (std_msgs/Float32MultiArray): Raw depth data array

**Usage with RViz2:**
1. Start the publisher: `./run.sh`
2. Launch RViz2: `rviz2`
3. Add a PointCloud2 display and set the topic to `/point_cloud`
4. Add a TF frame for the camera if needed

## API Documentation

### Core Classes and Functions

#### ArducamTOFCamera (C++)
- `open(Connection, int)`: Initialize camera connection
- `start(FrameType)`: Start frame capture
- `requestFrame(int)`: Request a frame with timeout
- `getControl(Control, void*)`: Get camera control value
- `setControl(Control, int)`: Set camera control value
- `getCameraInfo()`: Get camera information
- `releaseFrame(FrameBuffer*)`: Release frame buffer
- `stop()`: Stop frame capture
- `close()`: Close camera connection

#### ArducamCamera (Python)
- `open(Connection, int)`: Initialize camera connection
- `start(FrameType)`: Start frame capture
- `requestFrame(int)`: Request a frame with timeout
- `getControl(Control)`: Get camera control value
- `setControl(Control, int)`: Set camera control value
- `getCameraInfo()`: Get camera information
- `releaseFrame(Frame)`: Release frame
- `stop()`: Stop frame capture
- `close()`: Close camera connection

### Frame Types
- `DEPTH_FRAME`: Processed depth data
- `RAW_FRAME`: Raw sensor data
- `CONFIDENCE_FRAME`: Confidence/amplitude data

### Connection Types
- `CSI`: Camera Serial Interface
- `USB`: USB connection

### Device Types
- `HQVGA`: 240×180 resolution
- `VGA`: 640×480 resolution

## Troubleshooting

### Common Issues

1. **Camera not detected:**
   - Check power supply connection (5V 2A)
   - Verify CSI/USB connection
   - Check camera permissions

2. **Poor depth quality:**
   - Adjust confidence threshold
   - Check lighting conditions
   - Verify camera range setting

3. **Low frame rate:**
   - Check system resources
   - Reduce processing load
   - Verify camera settings

4. **Open3D visualization issues:**
   - Set `MESA_GL_VERSION_OVERRIDE=4.5`
   - Check OpenGL drivers
   - Verify Open3D installation

### Performance Optimization

- Use appropriate confidence thresholds for your environment
- Consider reducing resolution for higher frame rates
- Optimize processing pipeline for your specific use case
- Monitor system resources during operation

## Contributing

We welcome contributions to improve the SDK. Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes with proper documentation
4. Test on multiple platforms
5. Submit a pull request

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Support

For technical support and questions:
- GitHub Issues: [Create an issue](https://github.com/ArduCAM/Arducam_tof_camera/issues)
- Documentation: Check the inline code documentation
- Examples: Refer to the example applications for usage patterns
