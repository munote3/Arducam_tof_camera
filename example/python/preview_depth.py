"""
Arducam TOF Depth Camera Preview Application

This application demonstrates how to capture and display depth data from the Arducam TOF camera
using Python. It provides real-time depth visualization with confidence filtering, interactive
region selection, and distance measurement capabilities.

Features:
- Real-time depth frame capture and display
- Confidence-based filtering for noise reduction
- Interactive mouse selection for distance measurement
- Color-mapped depth visualization
- Support for both VGA and HQVGA camera types
- Dynamic confidence threshold adjustment

Author: Arducam
Version: 1.0
Date: 2024
"""

import cv2
import numpy as np
import ArducamDepthCamera as ac

# ============================================================================
# Configuration Constants
# ============================================================================

# MAX_DISTANCE value modifiable is 2000 or 4000
# 2000 = 2 meters range, 4000 = 4 meters range
MAX_DISTANCE = 4000


# ============================================================================
# Utility Classes
# ============================================================================

class UserRect:
    """
    Utility class for managing rectangular regions selected by the user.
    
    This class provides convenient properties for accessing rectangle coordinates
    in different formats (OpenCV rectangle format, numpy slice format) and
    checking if the rectangle is empty.
    """
    
    def __init__(self) -> None:
        """Initialize an empty rectangle."""
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

    @property
    def rect(self):
        """
        Get rectangle in OpenCV format (x, y, width, height).
        
        Returns:
            tuple: (x, y, width, height) for use with cv2.rectangle()
        """
        return (
            self.start_x,
            self.start_y,
            self.end_x - self.start_x,
            self.end_y - self.start_y,
        )

    @property
    def slice(self):
        """
        Get rectangle as numpy slice for array indexing.
        
        Returns:
            tuple: (row_slice, col_slice) for use with numpy arrays
        """
        return (slice(self.start_y, self.end_y), slice(self.start_x, self.end_x))

    @property
    def empty(self):
        """
        Check if the rectangle is empty (has zero area).
        
        Returns:
            bool: True if rectangle is empty, False otherwise
        """
        return self.start_x == self.end_x and self.start_y == self.end_y


# ============================================================================
# Global Variables
# ============================================================================

# Confidence threshold for depth filtering
# Values below this threshold are considered unreliable and filtered out
confidence_value = 30

# Rectangle objects for user interaction
selectRect, followRect = UserRect(), UserRect()

# ============================================================================
# Utility Functions
# ============================================================================

def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    """
    Apply confidence filtering to RGB preview image.
    
    This function filters out pixels with low confidence values by setting them to black.
    It also handles NaN values that might be present in the depth data.
    
    Args:
        preview (np.ndarray): RGB preview image to filter
        confidence (np.ndarray): Confidence/amplitude image for filtering
        
    Returns:
        np.ndarray: Filtered RGB preview image
    """
    preview = np.nan_to_num(preview)  # Handle NaN values
    preview[confidence < confidence_value] = (0, 0, 0)  # Set low confidence pixels to black
    return preview


def on_mouse(event, x, y, flags, param):
    """
    Mouse callback function for interactive region selection.
    
    Handles mouse events for selecting regions of interest in the depth image.
    Creates 8x8 pixel rectangles for distance measurement and visual feedback.
    
    Args:
        event: Mouse event type (click, move, etc.)
        x: Mouse x-coordinate
        y: Mouse y-coordinate
        flags: Additional mouse flags (unused)
        param: User parameter (unused)
    """
    global selectRect, followRect

    if event == cv2.EVENT_LBUTTONDOWN:
        # Start of selection (currently not used)
        pass

    elif event == cv2.EVENT_LBUTTONUP:
        # Set selected rectangle for distance measurement (8x8 pixels around click point)
        selectRect.start_x = x - 4
        selectRect.start_y = y - 4
        selectRect.end_x = x + 4
        selectRect.end_y = y + 4
    else:
        # Update follow rectangle for visual feedback (8x8 pixels around mouse position)
        followRect.start_x = x - 4
        followRect.start_y = y - 4
        followRect.end_x = x + 4
        followRect.end_y = y + 4


def on_confidence_changed(value):
    """
    Callback function for confidence threshold changes.
    
    This function is called when the confidence trackbar value changes.
    It updates the global confidence threshold for depth filtering.
    
    Args:
        value (int): New confidence threshold value (0-255)
    """
    global confidence_value
    confidence_value = value


def usage(argv0):
    """
    Display usage information for the application.
    
    Args:
        argv0 (str): Name of the script (argv[0])
    """
    print("Usage: python " + argv0 + " [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


# ============================================================================
# Main Application
# ============================================================================

def main():
    """
    Main function for the Arducam TOF Depth Camera Preview Application.
    
    This function initializes the TOF camera, sets up the display windows,
    and runs the main processing loop for real-time depth visualization.
    
    The application flow:
    1. Initialize and open the TOF camera
    2. Start depth frame capture
    3. Configure camera settings (range, etc.)
    4. Set up OpenCV windows and mouse callbacks
    5. Run main processing loop
    6. Clean up resources on exit
    """
    print("Arducam Depth Camera Demo.")
    print("  SDK version:", ac.__version__)

    # ========================================================================
    # Camera Initialization
    # ========================================================================
    
    cam = ac.ArducamCamera()
    cfg_path = None
    # cfg_path = "file.cfg"  # Uncomment to use configuration file

    # Color definitions for rectangle drawing
    black_color = (0, 0, 0)
    white_color = (255, 255, 255)

    # Open camera connection
    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, 0)
    else:
        ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    # Start depth frame capture
    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    # Configure camera range setting
    cam.setControl(ac.Control.RANGE, MAX_DISTANCE)
    r = cam.getControl(ac.Control.RANGE)

    # Get camera information
    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    # ========================================================================
    # Display Setup
    # ========================================================================
    
    # Create OpenCV windows
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preview", on_mouse)

    # Add confidence trackbar for VGA cameras (only VGA supports confidence)
    if info.device_type == ac.DeviceType.VGA:
        cv2.createTrackbar(
            "confidence", "preview", confidence_value, 255, on_confidence_changed
        )

    # ========================================================================
    # Main Processing Loop
    # ========================================================================
    
    while True:
        # Request a new frame from the camera (2000ms timeout)
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            # Extract depth and confidence data from frame
            depth_buf = frame.depth_data
            confidence_buf = frame.confidence_data

            # Convert depth data to 8-bit for visualization
            result_image = (depth_buf * (255.0 / r)).astype(np.uint8)
            
            # Apply rainbow color map for better depth visualization
            result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
            
            # Apply confidence filtering to remove unreliable pixels
            result_image = getPreviewRGB(result_image, confidence_buf)

            # Normalize confidence data for display
            cv2.normalize(confidence_buf, confidence_buf, 1, 0, cv2.NORM_MINMAX)

            # Display confidence window
            cv2.imshow("preview_confidence", confidence_buf)

            # Draw selection rectangles for distance measurement
            cv2.rectangle(result_image, followRect.rect, white_color, 1)  # White rectangle for mouse follow
            if not selectRect.empty:
                cv2.rectangle(result_image, selectRect.rect, black_color, 2)  # Black rectangle for selected area
                print("select Rect distance:", np.mean(depth_buf[selectRect.slice]))

            # Display main preview window
            cv2.imshow("preview", result_image)
            cam.releaseFrame(frame)

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
