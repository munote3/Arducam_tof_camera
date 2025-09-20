"""
Arducam TOF Camera Raw Data Capture Application

This application demonstrates how to capture and display raw sensor data from the Arducam TOF camera
using Python. Unlike the depth preview application, this captures the raw sensor data before depth
processing, which can be useful for debugging, calibration, or custom depth processing algorithms.

Features:
- Raw sensor data capture and display
- Real-time visualization of raw sensor values
- Simple grayscale visualization
- Support for configuration file input

Author: Arducam
Version: 1.0
Date: 2024
"""

import cv2
import numpy as np
import ArducamDepthCamera as ac

# ============================================================================
# Main Application
# ============================================================================

def main():
    """
    Main function for the Arducam TOF Camera Raw Data Capture Application.
    
    This function initializes the TOF camera for raw data capture, sets up the display window,
    and runs the main processing loop for real-time raw sensor data visualization.
    
    The application flow:
    1. Initialize and open the TOF camera
    2. Start raw frame capture (not depth frames)
    3. Set up OpenCV display window
    4. Run main processing loop
    5. Clean up resources on exit
    """
    print("Arducam Depth Camera Demo.")
    print("  SDK version:", ac.__version__)

    # ========================================================================
    # Camera Initialization
    # ========================================================================
    
    cam = ac.ArducamCamera()
    cfg_path = None
    # cfg_path = "file.cfg"  # Uncomment to use configuration file

    # Open camera connection
    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, 0)
    else:
        ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("initialization failed. Error code:", ret)
        return

    # Start raw frame capture (not processed depth frames)
    ret = cam.start(ac.FrameType.RAW)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    # ========================================================================
    # Main Processing Loop
    # ========================================================================
    
    while True:
        # Request a new raw frame from the camera (2000ms timeout)
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.RawData):
            # Extract raw sensor data from frame
            buf = frame.raw_data
            cam.releaseFrame(frame)

            # Convert 16-bit raw data to 8-bit for display
            # Scale factor: 1/(2^4) = 1/16 to fit 16-bit data into 8-bit range
            buf = (buf / (1 << 4)).astype(np.uint8)

            # Display the raw sensor data
            cv2.imshow("window", buf)

        # Handle keyboard input
        key = cv2.waitKey(1)
        if key == ord("q"):
            break  # Exit on 'q' key

    # ========================================================================
    # Cleanup and Exit
    # ========================================================================
    
    # Stop camera capture and close connection
    cam.stop()
    cam.close()


if __name__ == "__main__":
    main()
