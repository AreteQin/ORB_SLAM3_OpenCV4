// -----------------------------------------------------------------------------
// mono_webcam_rear.cc
//   Run ORB-SLAM3 live monocular on the Surface Pro 6 rear camera.
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

namespace
{
    volatile std::sig_atomic_t g_stop_requested = 0;
    void SigIntHandler(int) { g_stop_requested = 1; }

    // Return current time in seconds
    inline double Now()
    {
        return std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }
} // namespace

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 4)
    {
        cerr << "\nUsage: " << argv[0]
            << " <path_to_vocabulary> <path_to_settings_yaml>\n";
        return EXIT_FAILURE;
    }

    const string vocab_file = argv[1];
    const string settings_yml = argv[2];

    cv::FileStorage fs(settings_yml, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "ERROR: Cannot open settings file: " << settings_yml << std::endl;
        return -1;
    }

    // ---------------------------------------------------------------------
    // 1. Open the Surface Pro 6 rear camera -------------------------------
    // ---------------------------------------------------------------------
    const string device = "/dev/video42";
    cv::VideoCapture cap(device, cv::CAP_V4L2); // use V4L2 backend :contentReference[oaicite:3]{index=3}
    if (!cap.isOpened())
    {
        cerr << "ERROR: Could not open rear camera at " << device << endl;
        return EXIT_FAILURE;
    }

    // Limit to 1280×720 to prevent data-stream errors :contentReference[oaicite:4]{index=4}
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    cout << "Rear camera opened: "
        << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "×"
        << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << endl;

    // ---------------------------------------------------------------------
    // 2. Initialise ORB-SLAM3 ----------------------------------------------
    // ---------------------------------------------------------------------
    try
    {
        ORB_SLAM3::System SLAM(vocab_file, settings_yml, ORB_SLAM3::System::MONOCULAR, true);
    }
    catch (const cv::Exception& e)
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    }
    ORB_SLAM3::System SLAM(vocab_file, settings_yml, ORB_SLAM3::System::MONOCULAR, true);
    signal(SIGINT, SigIntHandler);

    const float imageScale = SLAM.GetImageScale();
    auto t_prev = chrono::steady_clock::now();
    const double desired_fps = 30.0;
    const double frame_period = 1.0 / desired_fps;

    cv::Mat frame;
    const string win = "ORB-SLAM3 Rear Camera";
    cv::namedWindow(win, cv::WINDOW_AUTOSIZE);

    // ---------------------------------------------------------------------
    // 3. Capture, correct orientation, and SLAM track ----------------------
    // ---------------------------------------------------------------------
    while (!g_stop_requested)
    {
        if (!cap.read(frame) || frame.empty())
        {
            cerr << "ERROR: Blank frame grabbed — aborting.\n";
            break;
        }

        // Resize if needed
        if (imageScale != 1.0f)
        {
            cv::resize(frame, frame, cv::Size(), imageScale, imageScale, cv::INTER_LINEAR);
        }

        // Flip both axes = 180° rotation + mirror correction :contentReference[oaicite:5]{index=5}
        cv::flip(frame, frame, -1);

        double t_now = Now();
        SLAM.TrackMonocular(frame, t_now);

        if (SLAM.isFinished()) break;

        // Throttle to ~30 Hz
        auto t_curr = chrono::steady_clock::now();
        double elapsed = chrono::duration<double>(t_curr - t_prev).count();
        if (elapsed < frame_period)
        {
            this_thread::sleep_for(chrono::duration<double>(frame_period - elapsed));
        }
        t_prev = chrono::steady_clock::now();

        if (cv::waitKey(1) == 27) break; // ESC to exit
    }

    // ---------------------------------------------------------------------
    // 4. Shutdown & save trajectories -------------------------------------
    // ---------------------------------------------------------------------
    cout << "\nShutting down SLAM …\n";
    SLAM.Shutdown();

    return EXIT_SUCCESS;
}
