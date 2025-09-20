/**
 * @file preview_depth.cpp
 * @brief Arducam TOF Depth Camera Preview Application
 * 
 * This application demonstrates how to capture and display depth data from the Arducam TOF camera.
 * It provides real-time depth visualization with confidence filtering, interactive region selection,
 * and the ability to save depth data to files.
 * 
 * Features:
 * - Real-time depth frame capture and display
 * - Confidence-based filtering for noise reduction
 * - Depth threshold filtering to remove far objects
 * - Interactive mouse selection for distance measurement
 * - FPS monitoring and display
 * - Depth data saving functionality
 * - Color-mapped depth visualization
 * - Adjustable depth threshold with trackbar control
 * 
 * @author Arducam
 * @version 1.0
 * @date 2024
 */

#include "ArducamTOFCamera.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace Arducam;

// ============================================================================
// Configuration Constants
// ============================================================================

/**
 * @brief Maximum distance range for the TOF camera in millimeters
 * 
 * Valid values are 2000 (2 meters) or 4000 (4 meters).
 * This affects the camera's measurement range and accuracy.
 */
#define MAX_DISTANCE 4000

// ============================================================================
// Global Variables
// ============================================================================

/**
 * @brief Rectangle for selected region (where user clicked)
 * Used for distance measurement at specific points
 */
cv::Rect seletRect(0, 0, 0, 0);

/**
 * @brief Rectangle for mouse follow region (current mouse position)
 * Provides visual feedback of current mouse position
 */
cv::Rect followRect(0, 0, 0, 0);

/**
 * @brief Maximum width of the camera frame
 * Updated dynamically based on camera resolution
 */
int max_width = 240;

/**
 * @brief Maximum height of the camera frame
 * Updated dynamically based on camera resolution
 */
int max_height = 180;

/**
 * @brief Current maximum range setting of the camera
 * Retrieved from camera control settings
 */
int max_range = 0;

/**
 * @brief Confidence threshold for depth filtering
 * Values below this threshold are considered unreliable and filtered out
 */
int confidence_value = 30;

/**
 * @brief Depth threshold for filtering far objects
 * Values above this threshold (in millimeters) are filtered out and displayed as black
 */
int depth_threshold = 2000;

// ============================================================================
// Callback Functions
// ============================================================================

/**
 * @brief Callback function for confidence threshold changes
 * 
 * This function is called when the confidence trackbar value changes.
 * Currently not implemented but can be used to dynamically adjust
 * the confidence filtering threshold.
 * 
 * @param pos New confidence threshold value (0-255)
 * @param userdata User data pointer (unused)
 */
void on_confidence_changed(int pos, void* userdata)
{
    // TODO: Implement dynamic confidence threshold adjustment
    // confidence_value = pos;
}

/**
 * @brief Callback function for depth threshold changes
 * 
 * This function is called when the depth threshold trackbar value changes.
 * It updates the global depth threshold for filtering far objects.
 * 
 * @param pos New depth threshold value (0-4000mm)
 * @param userdata User data pointer (unused)
 */
void on_depth_threshold_changed(int pos, void* userdata)
{
    depth_threshold = pos;
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Display and calculate FPS (Frames Per Second)
 * 
 * This function tracks the number of frames processed and calculates
 * the current FPS, displaying it every second. Uses high-resolution
 * timing for accurate measurements.
 * 
 * The FPS is calculated by counting frames over a 1-second interval
 * and resetting the counter after each display.
 */
void display_fps(void)
{
    using std::chrono::high_resolution_clock;
    using namespace std::literals;
    static int count = 0;
    static auto time_beg = high_resolution_clock::now();
    auto time_end = high_resolution_clock::now();
    ++count;
    auto duration_ms = (time_end - time_beg) / 1ms;
    if (duration_ms >= 1000) {
        std::cout << "fps:" << count << std::endl;
        count = 0;
        time_beg = time_end;
    }
}

/**
 * @brief Save depth image data to a binary file
 * 
 * Saves the current depth frame as a raw binary file with a timestamped filename.
 * The file format is: depth_[width]_[height]_f32_[timestamp].raw
 * 
 * @param image Pointer to the depth image data (float array)
 * @param width Width of the depth image
 * @param height Height of the depth image
 */
void save_image(float* image, int width, int height)
{
    using namespace std::literals;
    // filename = "depth_$width$_$height$_f32_$time.raw"
    auto now = std::chrono::system_clock::now().time_since_epoch() / 1ms;
    std::string filename =
        "depth_" + std::to_string(width) + "_" + std::to_string(height) + "_f32_" + std::to_string(now) + ".raw";
    std::ofstream file(filename, std::ios::binary);
    file.write(reinterpret_cast<char*>(image), width * height * sizeof(float));
    file.close();
}

/**
 * @brief Rotate an OpenCV Mat 180 degrees clockwise
 * 
 * Performs a 180-degree rotation by flipping the image both horizontally and vertically.
 * This is useful for correcting camera orientation if needed.
 * 
 * @param src Source image matrix to rotate
 * @return Rotated image matrix
 */
cv::Mat matRotateClockWise180(cv::Mat src)
{
    if (src.empty()) {
        std::cerr << "RorateMat src is empty!";
    }

    flip(src, src, 0);  // Flip vertically
    flip(src, src, 1);  // Flip horizontally
    return src;
}

/**
 * @brief Apply confidence filtering to grayscale preview image
 * 
 * This function filters out pixels with low confidence values by setting them to white (255).
 * It iterates through each pixel and compares the confidence value against the threshold.
 * 
 * @param preview_ptr Grayscale preview image to filter
 * @param amplitude_image_ptr Confidence/amplitude image for filtering
 */
void getPreview(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    auto len = preview_ptr.rows * preview_ptr.cols;
    for (int line = 0; line < preview_ptr.rows; line++) {
        for (int col = 0; col < preview_ptr.cols; col++) {
            if (amplitude_image_ptr.at<float>(line, col) < confidence_value)
                preview_ptr.at<uint8_t>(line, col) = 255;
        }
    }
}

/**
 * @brief Apply confidence filtering to RGB preview image
 * 
 * This function filters out pixels with low confidence values by setting them to black (0,0,0).
 * Uses OpenCV's setTo function for efficient vectorized operations.
 * 
 * @param preview_ptr RGB preview image to filter
 * @param amplitude_image_ptr Confidence/amplitude image for filtering
 */
void getPreviewRGB(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    preview_ptr.setTo(cv::Scalar(0, 0, 0), amplitude_image_ptr < confidence_value);
    // Optional: Apply Gaussian blur for noise reduction
    // cv::GaussianBlur(preview_ptr, preview_ptr, cv::Size(7, 7), 0);
}

/**
 * @brief Create depth threshold filtered image
 * 
 * This function creates a filtered depth image where pixels with depth values
 * greater than the threshold are set to black, effectively filtering out far objects.
 * 
 * @param depth_frame Input depth frame (CV_32F)
 * @param confidence_frame Input confidence frame for additional filtering
 * @return Filtered depth image (CV_8U)
 */
cv::Mat createDepthThresholdImage(cv::Mat depth_frame, cv::Mat confidence_frame)
{
    // Create a copy of the depth frame for filtering
    cv::Mat filtered_depth = depth_frame.clone();
    
    // Apply depth threshold filtering - set far pixels to 0
    filtered_depth.setTo(0, depth_frame > depth_threshold);
    
    // Also apply confidence filtering
    filtered_depth.setTo(0, confidence_frame < confidence_value);
    
    // Convert to 8-bit for display
    cv::Mat result;
    filtered_depth.convertTo(result, CV_8U, 255.0 / 7000, 0);
    
    // Apply rainbow color map for better visualization
    cv::applyColorMap(result, result, cv::COLORMAP_RAINBOW);
    
    return result;
}

/**
 * @brief Mouse callback function for interactive region selection
 * 
 * Handles mouse events for selecting regions of interest in the depth image.
 * Creates 8x8 pixel rectangles for distance measurement and visual feedback.
 * 
 * Mouse Events:
 * - Left button down: Start selection (currently unused)
 * - Left button up: Set selected rectangle for distance measurement
 * - Mouse move: Update follow rectangle for visual feedback
 * 
 * @param event Mouse event type (click, move, etc.)
 * @param x Mouse x-coordinate
 * @param y Mouse y-coordinate
 * @param flags Additional mouse flags (unused)
 * @param param User parameter (unused)
 */
void onMouse(int event, int x, int y, int flags, void* param)
{
    // Ensure mouse position is within valid bounds (4 pixels from edges)
    if (x < 4 || x > (max_width - 4) || y < 4 || y > (max_height - 4))
        return;
        
    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        // Start of selection (currently not used)
        break;

    case cv::EVENT_LBUTTONUP:
        // Set selected rectangle for distance measurement
        seletRect.x = x - 4 ? x - 4 : 0;
        seletRect.y = y - 4 ? y - 4 : 0;
        seletRect.width = 8;
        seletRect.height = 8;
        break;
    default:
        // Update follow rectangle for visual feedback
        followRect.x = x - 4 ? x - 4 : 0;
        followRect.y = y - 4 ? y - 4 : 0;
        followRect.width = 8;
        followRect.height = 8;
        break;
    }
}

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main function for the Arducam TOF Depth Camera Preview Application
 * 
 * This function initializes the TOF camera, sets up the display windows,
 * and runs the main processing loop for real-time depth visualization.
 * 
 * The application flow:
 * 1. Initialize and open the TOF camera
 * 2. Start depth frame capture
 * 3. Configure camera settings (range, etc.)
 * 4. Set up OpenCV windows, trackbars, and mouse callbacks
 * 5. Run main processing loop with real-time filtering
 * 6. Clean up resources on exit
 * 
 * Windows:
 * - "preview": Main depth visualization with mouse interaction
 * - "confidence": Confidence/amplitude data display
 * - "depth_threshold": Depth threshold filtered view with adjustable trackbar
 * 
 * @return 0 on success, -1 on error
 */
int main()
{
    // ========================================================================
    // Camera Initialization
    // ========================================================================
    
    ArducamTOFCamera tof;
    ArducamFrameBuffer* frame;
    
    // Open camera connection (CSI interface)
    if (tof.open(Connection::CSI, 0)) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    // Start depth frame capture
    if (tof.start(FrameType::DEPTH_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    
    // Configure camera range setting
    // Note: Modify MAX_DISTANCE constant to change range
    tof.setControl(Control::RANGE, MAX_DISTANCE);
    tof.getControl(Control::RANGE, &max_range);
    
    // Get camera information
    auto info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ")" << std::endl;

    // ========================================================================
    // Display Setup
    // ========================================================================
    
    // Allocate buffer for preview image
    uint8_t* preview_ptr = new uint8_t[info.width * info.height * 2];
    
    // Create OpenCV windows
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("preview", onMouse);
    
    // Create depth threshold filtering window
    cv::namedWindow("depth_threshold", cv::WINDOW_AUTOSIZE);
    
    // Create trackbar for depth threshold (0-4000mm)
    cv::createTrackbar("Depth Threshold (mm)", "depth_threshold", &depth_threshold, 4000, on_depth_threshold_changed);

    // ========================================================================
    // Main Processing Loop
    // ========================================================================
    
    for (;;) {
        // Request a new frame from the camera (200ms timeout)
        Arducam::FrameFormat format;
        frame = tof.requestFrame(200);
        if (frame == nullptr) {
            continue;  // Skip this iteration if no frame available
        }
        
        // Get frame format information
        frame->getFormat(FrameType::DEPTH_FRAME, format);
        std::cout << "frame: (" << format.width << "x" << format.height << ")" << std::endl;
        max_height = format.height;
        max_width = format.width;

        // Extract depth and confidence data from frame
        float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
        float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);

        // Create OpenCV matrices from raw data
        cv::Mat result_frame(format.height, format.width, CV_8U, preview_ptr);
        cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
        cv::Mat confidence_frame(format.height, format.width, CV_32F, confidence_ptr);

        // Optional: Apply 180-degree rotation if needed
        // depth_frame = matRotateClockWise180(depth_frame);
        // result_frame = matRotateClockWise180(result_frame);
        // confidence_frame = matRotateClockWise180(confidence_frame);
        
        // Convert depth data to 8-bit for visualization (scale to 7000mm max)
        depth_frame.convertTo(result_frame, CV_8U, 255.0 / 7000, 0);

        // Apply rainbow color map for better depth visualization
        cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);
        
        // Apply confidence filtering to remove unreliable pixels
        getPreviewRGB(result_frame, confidence_frame);

        // Convert confidence data to 8-bit for display
        confidence_frame.convertTo(confidence_frame, CV_8U, 255.0 / 1024, 0);

        // Display confidence window
        cv::imshow("confidence", confidence_frame);

        // Create and display depth threshold filtered image
        cv::Mat depth_threshold_image = createDepthThresholdImage(depth_frame, confidence_frame);
        cv::imshow("depth_threshold", depth_threshold_image);

        // Draw selection rectangles for distance measurement
        cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);      // Black rectangle for selected area
        cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1); // White rectangle for mouse follow

        // Calculate and display average distance in selected region
        std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << std::endl;

        // Display main preview window
        cv::imshow("preview", result_frame);

        // Handle keyboard input
        auto key = cv::waitKey(1);
        if (key == 27 || key == 'q') {
            break;  // Exit on ESC or 'q'
        } else if (key == 's') {
            save_image(depth_ptr, format.width, format.height);  // Save depth data
        }
        
        // Update FPS counter
        display_fps();
        
        // Release frame buffer
        tof.releaseFrame(frame);
    }

    // ========================================================================
    // Cleanup and Exit
    // ========================================================================
    
    // Stop camera capture
    if (tof.stop()) {
        return -1;
    }

    // Close camera connection
    if (tof.close()) {
        return -1;
    }

    // Clean up allocated memory
    delete[] preview_ptr;

    return 0;
}
