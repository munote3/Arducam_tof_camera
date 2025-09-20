/**
 * @file preview_pointcloud.cpp
 * @brief Arducam TOF Camera 3D Point Cloud Preview Application
 * 
 * This application demonstrates how to capture depth data from the Arducam TOF camera
 * and visualize it as a real-time 3D point cloud using Open3D. It provides interactive
 * 3D visualization with the ability to save point clouds and depth data.
 * 
 * Features:
 * - Real-time 3D point cloud visualization using Open3D
 * - Interactive depth preview with confidence filtering
 * - Point cloud saving functionality (PCD format)
 * - Depth data saving functionality (RAW format)
 * - Dynamic confidence threshold adjustment
 * - Mouse interaction for region selection
 * - FPS monitoring and display
 * - Support for both VGA and HQVGA camera types
 * 
 * Controls:
 * - ESC or 'q': Exit application
 * - 's': Save current point cloud as PCD file
 * - 'r': Save current depth data as RAW file
 * - '+/-' or '.,': Adjust confidence threshold
 * - '[/]': Adjust confidence threshold by 5
 * - Mouse: Select regions for distance measurement
 * 
 * @author Arducam
 * @version 1.0
 * @date 2024
 */

#include "ArducamTOFCamera.hpp"
// #include <atomic>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Dense>
#include <open3d/Open3D.h>

// ============================================================================
// Configuration Constants
// ============================================================================

#define LOCAL        static inline
// MAX_DISTANCE value modifiable is 2000 or 4000
// 2000 = 2 meters range, 4000 = 4 meters range
#define MAX_DISTANCE 4000
#define WITH_VT_100  1

// VT100 escape sequences for terminal formatting
#if WITH_VT_100
#define ESC(x) "\033" x
#define NL     ESC("[K\n")
#else
#define ESC(x) ""
#define NL     "\n"
#endif

using namespace Arducam;

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
int max_width = 640;

/**
 * @brief Maximum height of the camera frame
 * Updated dynamically based on camera resolution
 */
int max_height = 480;

/**
 * @brief Current maximum range setting of the camera
 * Retrieved from camera control settings
 */
int max_range = 0;

/**
 * @brief Flag to enable confidence threshold control
 * Only enabled for VGA cameras that support confidence
 */
bool enable_confidence_ct = false;

/**
 * @brief Confidence threshold for depth filtering
 * Values below this threshold are considered unreliable and filtered out
 */
int confidence_value = 0;
// std::atomic<int> confidence_value{0};  // Alternative atomic version

// ============================================================================
// Callback Functions
// ============================================================================

/**
 * @brief Callback function for confidence threshold changes
 * 
 * This function is called when the confidence trackbar value changes.
 * It updates the global confidence threshold for depth filtering.
 * 
 * @param pos New confidence threshold value (0-255)
 * @param userdata User data pointer (unused)
 */
void on_confidence_changed(int pos, void* userdata)
{
    confidence_value = pos;
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
        // Set selected rectangle for distance measurement (8x8 pixels around click point)
        seletRect.x = x - 4 ? x - 4 : 0;
        seletRect.y = y - 4 ? y - 4 : 0;
        seletRect.width = 8;
        seletRect.height = 8;
        break;
    default:
        // Update follow rectangle for visual feedback (8x8 pixels around mouse position)
        followRect.x = x - 4 ? x - 4 : 0;
        followRect.y = y - 4 ? y - 4 : 0;
        followRect.width = 8;
        followRect.height = 8;
        break;
    }
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Display and calculate FPS (Frames Per Second)
 * 
 * This function tracks the number of frames processed and calculates
 * the current FPS, displaying it every second. Uses high-resolution
 * timing for accurate measurements and VT100 escape sequences for
 * clean terminal output.
 * 
 * The FPS is calculated by counting frames over a 1-second interval
 * and resetting the counter after each display.
 */
LOCAL void display_fps(void)
{
    using std::chrono::high_resolution_clock;
    using namespace std::literals;
    static int count = 0;
    static auto time_beg = high_resolution_clock::now();
    auto time_end = high_resolution_clock::now();
    ++count;
    auto duration_ms = (time_end - time_beg) / 1ms;
    if (duration_ms >= 1000) {
        std::cout << "fps: " << count << NL;
        count = 0;
        time_beg = time_end;
    }
#if WITH_VT_100
    else {
        std::cout << "\n";
    }
#endif
}

LOCAL void save_image(float* image, int width, int height)
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

LOCAL cv::Mat matRotateClockWise180(cv::Mat src)
{
    if (src.empty()) {
        std::cerr << "RorateMat src is empty!";
    }

    flip(src, src, 0);
    flip(src, src, 1);
    return src;
}

LOCAL void getPreview(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    auto len = preview_ptr.rows * preview_ptr.cols;
    for (int line = 0; line < preview_ptr.rows; line++) {
        for (int col = 0; col < preview_ptr.cols; col++) {
            if (amplitude_image_ptr.at<float>(line, col) < confidence_value)
                preview_ptr.at<uint8_t>(line, col) = 255;
        }
    }
}

LOCAL void getPreviewRGB(cv::Mat preview_ptr, cv::Mat amplitude_image_ptr)
{
    preview_ptr.setTo(cv::Scalar(0, 0, 0), amplitude_image_ptr < confidence_value);
    // cv::GaussianBlur(preview_ptr, preview_ptr, cv::Size(7, 7), 0);
}

LOCAL bool getControl(ArducamTOFCamera& tof, Arducam::Control mode, float& val, float alpha = 1.0)
{
    int tmp = 0;
    Arducam::TofErrorCode ret = tof.getControl(mode, &tmp);
    if (ret != 0) {
        std::cerr << "Failed to get control" << std::endl;
        return false;
    }
    val = tmp / alpha;
    return true;
}

LOCAL bool checkExit()
{
    int key = cv::waitKey(500);
    switch (key) {
    case 27:
    case 'q':
        return false;
    }
    return true;
}

bool processKey(const open3d::geometry::PointCloud& pcd, float* depth_ptr, const Arducam::FrameFormat& format)
{
    auto key = cv::waitKey(1);
    switch (key) {
    case 27:
    case 'q': {
        return false;
    } break;
    case 's': {
        open3d::io::WritePointCloudToPCD( //
            "pointcloud.pcd", pcd,
            {
                open3d::io::WritePointCloudOption::IsAscii::Ascii,
                open3d::io::WritePointCloudOption::Compressed::Uncompressed,
            });
    } break;
    case 'r': {
        save_image(depth_ptr, format.width, format.height);
    } break;
    case '=':
    case '.': {
        if (enable_confidence_ct) {
            confidence_value += 1;
            if (confidence_value > 255) {
                confidence_value = 255;
            }
            std::cout << "confidence value: " << confidence_value << NL;
        }
    } break;
    case '-':
    case ',': {
        if (enable_confidence_ct) {
            confidence_value -= 1;
            if (confidence_value < 0) {
                confidence_value = 0;
            }
            std::cout << "confidence value: " << confidence_value << NL;
        }
    } break;
    case '>':
    case ']': {
        if (enable_confidence_ct) {
            confidence_value += 5;
            if (confidence_value > 255) {
                confidence_value = 255;
            }
            std::cout << "confidence value: " << confidence_value << NL;
        }
    } break;
    case '<':
    case '[': {
        if (enable_confidence_ct) {
            confidence_value -= 5;
            if (confidence_value < 0) {
                confidence_value = 0;
            }
            std::cout << "confidence value: " << confidence_value << NL;
        }
    } break;
    default:
        break;
    }
    return true;
}

bool pc_loop(ArducamTOFCamera& tof, std::shared_ptr<open3d::geometry::PointCloud>& pcd, Eigen::Matrix4d transfrom)
{
    Arducam::FrameFormat format;
    ArducamFrameBuffer* frame = tof.requestFrame(500);
    if (frame == nullptr) {
        return checkExit();
    }
    frame->getFormat(FrameType::DEPTH_FRAME, format);
    std::cout << "frame: (" << format.width << "x" << format.height << ")" << NL;
    max_height = format.height;
    max_width = format.width;

    float* depth_ptr = (float*)frame->getData(FrameType::DEPTH_FRAME);
    float* confidence_ptr = (float*)frame->getData(FrameType::CONFIDENCE_FRAME);

    cv::Mat result_frame(format.height, format.width, CV_8U);
    cv::Mat depth_frame(format.height, format.width, CV_32F, depth_ptr);
    cv::Mat confidence_frame(format.height, format.width, CV_32F, confidence_ptr);

    // depth_frame = matRotateClockWise180(depth_frame);
    // result_frame = matRotateClockWise180(result_frame);
    // confidence_frame = matRotateClockWise180(confidence_frame);
    depth_frame.convertTo(result_frame, CV_8U, 255.0 / 7000, 0);

    cv::applyColorMap(result_frame, result_frame, cv::COLORMAP_RAINBOW);
    getPreviewRGB(result_frame, confidence_frame);

    // confidence_frame.convertTo(confidence_frame, CV_8U, 255.0 / 1024, 0);

    // cv::imshow("confidence", confidence_frame);

    cv::rectangle(result_frame, seletRect, cv::Scalar(0, 0, 0), 2);
    cv::rectangle(result_frame, followRect, cv::Scalar(255, 255, 255), 1);

    std::cout << "select Rect distance: " << cv::mean(depth_frame(seletRect)).val[0] << NL;

    cv::imshow("preview", result_frame);
    depth_frame.setTo(0, confidence_frame < confidence_value);
    auto depth_image = open3d::geometry::Image();
    depth_image.Prepare(max_width, max_height, 1, 4);

    memcpy(depth_image.data_.data(), depth_frame.data, depth_image.data_.size());

    float fx, fy, cx, cy;

    getControl(tof, Control::INTRINSIC_FX, fx, 100);
    getControl(tof, Control::INTRINSIC_FY, fy, 100);
    getControl(tof, Control::INTRINSIC_CX, cx, 100);
    getControl(tof, Control::INTRINSIC_CY, cy, 100);

    std::cout << "fx: " << fx << " fy: " << fy << " cx: " << cx << " cy: " << cy << NL;

    auto curr = open3d::geometry::PointCloud::CreateFromDepthImage( //
        depth_image, {max_width, max_height, fx, fy, cx, cy}, Eigen::Matrix4d::Identity(), 1000.0, 5000.0);

    curr->Transform(transfrom);

    // auto voxel_down_pcd = pcd->VoxelDownSample(20);
    // auto tuple = voxel_down_pcd->RemoveStatisticalOutliers(20, 2.0);
    // auto inlier_cloud = voxel_down_pcd->SelectByIndex(std::get<std::vector<size_t>>(tuple));
    // pcd->points_ = inlier_cloud->points_;

    display_fps();
    if (!processKey(*curr, depth_ptr, format)) {
        return false;
    }
    pcd->points_ = curr->points_;
    tof.releaseFrame(frame);
    return true;
}

// ============================================================================
// Main Application
// ============================================================================

/**
 * @brief Main function for the Arducam TOF Camera 3D Point Cloud Preview Application
 * 
 * This function initializes the TOF camera, sets up Open3D visualization,
 * and runs the main processing loop for real-time 3D point cloud visualization.
 * 
 * Command line arguments:
 * - argv[1]: Camera configuration file path (optional, use "-" for default)
 * - argv[2]: Initial confidence threshold value (optional)
 * 
 * The application flow:
 * 1. Parse command line arguments
 * 2. Initialize and open the TOF camera
 * 3. Start depth frame capture
 * 4. Configure camera settings
 * 5. Set up Open3D visualization and OpenCV windows
 * 6. Run main processing loop
 * 7. Clean up resources on exit
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments array
 * @return 0 on success, -1 on error
 */
int main(int argc, char* argv[])
{
    // ========================================================================
    // Command Line Argument Parsing
    // ========================================================================
    
    ArducamTOFCamera tof;
    const char* cfg_path = nullptr;

    // Parse confidence threshold from command line (optional)
    if (argc > 2) {
        confidence_value = atoi(argv[2]);
    }
    
    // Parse configuration file path from command line (optional)
    if (argc > 1) {
        if (strcmp(argv[1], "-") != 0) {
            cfg_path = argv[1];
        }
    }
    
    // ========================================================================
    // Camera Initialization
    // ========================================================================
    
    // Open camera connection
    if (cfg_path != nullptr) {
        if (tof.openWithFile(cfg_path)) {
            std::cerr << "Failed to open camera with cfg: " << cfg_path << std::endl;
            return -1;
        }
    } else {
        if (tof.open(Connection::CSI)) {
            std::cerr << "Failed to open camera" << std::endl;
            return -1;
        }
    }

    if (tof.start(FrameType::DEPTH_FRAME)) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    //  Modify the range also to modify the MAX_DISTANCE
    tof.setControl(Control::RANGE, MAX_DISTANCE);
    tof.getControl(Control::RANGE, &max_range);
    auto info = tof.getCameraInfo();
    std::cout << "open camera with (" << info.width << "x" << info.height << ")" << std::endl;

    cv::namedWindow("preview", cv::WINDOW_NORMAL);
    cv::setMouseCallback("preview", onMouse);
    if (info.device_type == Arducam::DeviceType::DEVICE_VGA) {
        // only vga support confidence
        enable_confidence_ct = true;
        cv::createTrackbar("confidence", "preview", NULL, 255, on_confidence_changed);
        cv::setTrackbarPos("confidence", "preview", confidence_value);
    }

    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    // clang-format off
    m << 1,  0,  0,  0,
         0, -1,  0,  0,
         0,  0, -1,  0,
         0,  0,  0,  1;
    // clang-format on

    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    open3d::visualization::Visualizer vis;
    vis.CreateVisualizerWindow("Point Cloud", max_width, max_height);
    bool first = true;

    std::cout << ESC("[?25l"); // hide cursor
    std::cout << ESC("7");     // save cursor position
    for (; pc_loop(tof, pcd, m);) {
        if (first) {
            vis.AddGeometry(pcd);
            first = false;
        }
        vis.UpdateGeometry(pcd);
        vis.PollEvents();
        vis.UpdateRender();
        std::cout << ESC("8"); // restore cursor position
    }
    std::cout << ESC("[?25h");     // show cursor
    std::cout << ESC("[6B") << NL; // move cursor down 6 lines

    vis.DestroyVisualizerWindow();

    if (tof.stop()) {
        return -1;
    }

    if (tof.close()) {
        return -1;
    }

    return 0;
}
