/**
 * @file capture_raw.cpp
 * @brief Arducam TOF Camera Raw Data Capture Application
 * 
 * This application demonstrates how to capture and display raw sensor data from the Arducam TOF camera.
 * Unlike the depth preview application, this captures the raw sensor data before depth processing,
 * which can be useful for debugging, calibration, or custom depth processing algorithms.
 * 
 * Features:
 * - Raw sensor data capture and display
 * - Real-time visualization of raw sensor values
 * - FPS monitoring and display
 * - Simple grayscale visualization
 * 
 * @author Arducam
 * @version 1.0
 * @date 2024
 */

#include "ArducamTOFCamera.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace Arducam;

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
 * @brief Process keyboard input for application control
 * 
 * Handles keyboard input to control the application flow.
 * Currently supports exit commands (ESC and 'q').
 * 
 * @return true to continue running, false to exit
 */
bool processKey()
{
    int key = cv::waitKey(1);
    switch (key) {
    case 27:  // ESC key
    case 'q': // 'q' key
        return false;
    default:
        break;
    }
    return true;
}

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main function for the Arducam TOF Camera Raw Data Capture Application
 * 
 * This function initializes the TOF camera for raw data capture, sets up the display window,
 * and runs the main processing loop for real-time raw sensor data visualization.
 * 
 * The application flow:
 * 1. Initialize and open the TOF camera
 * 2. Start raw frame capture (not depth frames)
 * 3. Configure camera settings
 * 4. Set up OpenCV display window
 * 5. Run main processing loop
 * 6. Clean up resources on exit
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
    if (tof.open(Connection::CSI)) {
        std::cerr << "initialization failed" << std::endl;
        return -1;
    }
    
    // Start raw frame capture (not processed depth frames)
    if (tof.start(FrameType::RAW_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    
    // Configure camera range setting
    int max_range = 4000;
    // Note: Uncomment the line below to set the range explicitly
    // tof.setControl(Control::RANGE, 4000);
    tof.getControl(Control::RANGE, &max_range);

    // Get camera information
    CameraInfo info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ") with range " << max_range << std::endl;

    // ========================================================================
    // Display Setup
    // ========================================================================
    
    // Create OpenCV window for raw data display
    cv::namedWindow("preview", cv::WINDOW_AUTOSIZE);

    // ========================================================================
    // Main Processing Loop
    // ========================================================================
    
    for (;;) {
        // Request a new raw frame from the camera (2000ms timeout)
        frame = tof.requestFrame(2000);
        if (frame == nullptr) {
            continue;  // Skip this iteration if no frame available
        }
        
        // Get frame format information
        FrameFormat format;
        frame->getFormat(FrameType::RAW_FRAME, format);
        std::cout << "frame: (" << format.width << "x" << format.height << ")" << std::endl;

        // Extract raw sensor data from frame
        int16_t* raw_ptr = (int16_t*)frame->getData(FrameType::RAW_FRAME);
        if (raw_ptr == nullptr) {
            tof.releaseFrame(frame);
            continue;  // Skip if no raw data available
        }

        // Create OpenCV matrices for visualization
        cv::Mat result_frame(format.height, format.width, CV_8U);
        cv::Mat raw_frame(format.width, format.height, CV_16S, raw_ptr);

        // Convert 16-bit raw data to 8-bit for display
        // Scale factor: 1/(2^4) = 1/16 to fit 16-bit data into 8-bit range
        raw_frame.convertTo(result_frame, CV_8U, 1. / (1 << 4), 0);
        
        // Display the raw sensor data
        cv::imshow("preview", result_frame);

        // Process keyboard input
        if (!processKey()) {
            break;  // Exit on ESC or 'q'
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

    return 0;
}
