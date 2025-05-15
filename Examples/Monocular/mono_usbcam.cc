// -----------------------------------------------------------------------------
// mono_webcam.cc  —  Run ORB-SLAM3 in live monocular mode with an OpenCV camera
// -----------------------------------------------------------------------------
// Usage:
//   ./mono_webcam <path_to_vocabulary> <path_to_settings_yaml>
//                 <camera_id|device_path> [fps] [trajectory_suffix]
//
//   <camera_id|device_path>  0, 1, ... for /dev/video*, or full path e.g. /dev/video2
//   [fps]                    optional capture framerate (default 30 Hz)
//   [trajectory_suffix]      if given, trajectories are saved to
//                            KeyFrameTrajectory_<suffix>.txt and
//                            CameraTrajectory_<suffix>.txt.
//
// Build inside ORB_SLAM3 root (replace opencv4 with opencv if needed):
//   g++ -std=c++14 -O3 mono_webcam.cc \
//       `pkg-config --cflags --libs opencv4 pangolin` \
//       -lORB_SLAM3 -o mono_webcam
// -----------------------------------------------------------------------------
// This file is based on ORB-SLAM3 examples and keeps the same licence (GPL-3+).
// -----------------------------------------------------------------------------

#include <System.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

using namespace std;

namespace {
volatile std::sig_atomic_t g_stop_requested = 0;

void SigIntHandler(int) { g_stop_requested = 1; }

// Return current (monotonic) time in seconds as double
inline double Now() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 4 || argc > 6) {
        cerr << "\nUsage: " << argv[0]
             << " <path_to_vocabulary> <path_to_settings_yaml>"
                " <camera_id|device_path> [fps] [trajectory_suffix]\n";
        return EXIT_FAILURE;
    }

    const string vocab_file   = argv[1];
    const string settings_yml = argv[2];
    const string cam_arg      = argv[3];
    const double desired_fps  = (argc >= 5) ? std::stod(argv[4]) : 30.0;
    const double frame_period = 1.0 / desired_fps;   // seconds
    const bool   save_suffix  = (argc == 6);
    const string suffix       = save_suffix ? string(argv[5]) : "";

    // ---------------------------------------------------------------------
    // 1. Open camera -------------------------------------------------------
    // ---------------------------------------------------------------------
    cv::VideoCapture cap;
    try {
        // Try to interpret the argument as an integer camera index first
        int cam_id = std::stoi(cam_arg);
        cap.open(cam_id, cv::CAP_ANY);
    } catch (const std::invalid_argument&) {
        // Fallback: treat as device/path name
        cap.open(cam_arg, cv::CAP_ANY);
    }

    if (!cap.isOpened()) {
        cerr << "ERROR: Could not open camera: " << cam_arg << endl;
        return EXIT_FAILURE;
    }

    cap.set(cv::CAP_PROP_FPS, desired_fps);

    cout << "Camera opened → resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH)
         << "×" << cap.get(cv::CAP_PROP_FRAME_HEIGHT)
         << ", target FPS: " << desired_fps << endl;

    // ---------------------------------------------------------------------
    // 2. Initialise ORB-SLAM3 ---------------------------------------------
    // ---------------------------------------------------------------------
    ORB_SLAM3::System SLAM(vocab_file, settings_yml, ORB_SLAM3::System::MONOCULAR,
                           true /* Enable Pangolin viewer */);

    const float imageScale = SLAM.GetImageScale();

    std::signal(SIGINT, SigIntHandler);

    cout << "Press Ctrl-C, ESC, or close Pangolin window to exit.\n";

    // Timing helpers
    std::chrono::steady_clock::time_point t_prev = std::chrono::steady_clock::now();

    while (!g_stop_requested) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            cerr << "Camera frame empty — aborting." << endl;
            break;
        }

        if (imageScale != 1.0f) {
            cv::resize(frame, frame, cv::Size(), imageScale, imageScale,
                       cv::INTER_LINEAR);
        }

        const double t_now = Now();
        SLAM.TrackMonocular(frame, t_now);

        // Break if SLAM finished (viewer closed)
        if (SLAM.isFinished())
            break;

        // Sleep so we do not outrun the desired FPS
        std::chrono::steady_clock::time_point t_curr =
            std::chrono::steady_clock::now();
        double elapsed =
            std::chrono::duration<double>(t_curr - t_prev).count();
        if (elapsed < frame_period) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(frame_period - elapsed));
        }
        t_prev = std::chrono::steady_clock::now();

        // Allow ESC key to terminate
        if (cv::waitKey(1) == 27)
            break;
    }

    // ---------------------------------------------------------------------
    // 3. Clean shutdown & save trajectories --------------------------------
    // ---------------------------------------------------------------------
    cout << "\nShutting down SLAM …" << endl;
    SLAM.Shutdown();

    const string kf_file  = save_suffix ? "KeyFrameTrajectory_" + suffix + ".txt"
                                        : "KeyFrameTrajectory.txt";
    const string cam_file = save_suffix ? "CameraTrajectory_" + suffix + ".txt"
                                        : "CameraTrajectory.txt";

    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    SLAM.SaveTrajectoryEuRoC(cam_file);

    cout << "Trajectories saved to " << kf_file << " and " << cam_file << endl;
    return EXIT_SUCCESS;
}
