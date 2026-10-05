#include "spinnaker_grabber.hpp"

#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

namespace passive_stereo_capture
{

// ─────────────────────────────────────────────────────────────────────────────
// GenICam helpers
// ─────────────────────────────────────────────────────────────────────────────

static std::string getCameraSerial(CameraPtr cam)
{
    std::string s;
    try {
        GenApi::INodeMap & tlMap = cam->GetTLDeviceNodeMap();
        GenApi::CStringPtr ptrSerial = tlMap.GetNode("DeviceSerialNumber");
        if (GenApi::IsAvailable(ptrSerial) && GenApi::IsReadable(ptrSerial)) {
            s = ptrSerial->GetValue().c_str();
        }
    } catch (...) {}

    if (s.empty()) {
        bool wasInit = cam->IsInitialized();
        if (!wasInit) cam->Init();
        try {
            s = cam->DeviceSerialNumber.GetValue().c_str();
        } catch (...) {}
        if (!wasInit) cam->DeInit();
    }
    return s;
}

static CameraPtr findCameraBySerial(
    const CameraList & camList, const std::string & serial)
{
    for (unsigned int i = 0; i < camList.GetSize(); ++i) {
        CameraPtr cam = camList.GetByIndex(i);
        if (getCameraSerial(cam) == serial) return cam;
    }
    return nullptr;
}

static void setEnum(CameraPtr & cam, const std::string & node_name, const std::string & value)
{
    INodeMap & nm = cam->GetNodeMap();
    CEnumerationPtr node = nm.GetNode(node_name.c_str());
    if (!IsAvailable(node) || !IsWritable(node)) return;
    CEnumEntryPtr entry = node->GetEntryByName(value.c_str());
    if (!IsAvailable(entry) || !IsReadable(entry)) return;
    node->SetIntValue(entry->GetValue());
}

static void setFloat(CameraPtr & cam, const std::string & node_name, double value)
{
    INodeMap & nm = cam->GetNodeMap();
    CFloatPtr node = nm.GetNode(node_name.c_str());
    if (!IsAvailable(node) || !IsWritable(node)) return;
    double min_val = node->GetMin();
    double max_val = node->GetMax();
    value = std::clamp(value, min_val, max_val);
    node->SetValue(value);
}

static void setBool(CameraPtr & cam, const std::string & node_name, bool value)
{
    INodeMap & nm = cam->GetNodeMap();
    CBooleanPtr node = nm.GetNode(node_name.c_str());
    if (!IsAvailable(node) || !IsWritable(node)) return;
    node->SetValue(value);
}

static void setInt(CameraPtr & cam, const std::string & node_name, int64_t value)
{
    INodeMap & nm = cam->GetNodeMap();
    CIntegerPtr node = nm.GetNode(node_name.c_str());
    if (!IsAvailable(node) || !IsWritable(node)) return;
    node->SetValue(value);
}

// ─────────────────────────────────────────────────────────────────────────────
// SpinnakerGrabber
// ─────────────────────────────────────────────────────────────────────────────

SpinnakerGrabber::SpinnakerGrabber(const CameraConfig & cfg)
: cfg_(cfg)
{
    system_ = System::GetInstance();
    CameraList camList = system_->GetCameras();
    const unsigned int num_cams = camList.GetSize();

    std::cout << "[SpinnakerGrabber] Found " << num_cams << " camera(s) attached.\n";

    // Try finding configured serials first
    if (!cfg_.serial_left.empty()) {
        cam_left_ = findCameraBySerial(camList, cfg_.serial_left);
    }
    if (!cfg_.serial_right.empty()) {
        cam_right_ = findCameraBySerial(camList, cfg_.serial_right);
    }

    // Auto-detection fallback: "believe in what you get from cameras"
    if (!cam_left_ || !cam_right_) {
        if (num_cams >= 2) {
            std::cout << "[SpinnakerGrabber] Configured serials ("
                      << cfg_.serial_left << ", " << cfg_.serial_right
                      << ") not matched. Auto-assigning first two detected cameras.\n";
            std::vector<std::pair<std::string, CameraPtr>> detected;
            for (unsigned int i = 0; i < num_cams; ++i) {
                CameraPtr c = camList.GetByIndex(i);
                std::string s = getCameraSerial(c);
                detected.push_back({s, c});
            }
            // Sort by serial to ensure deterministic left/right assignment
            std::sort(detected.begin(), detected.end(),
                      [](const auto & a, const auto & b) { return a.first < b.first; });

            cfg_.serial_left  = detected[0].first;
            cam_left_         = detected[0].second;
            cfg_.serial_right = detected[1].first;
            cam_right_        = detected[1].second;

            std::cout << "[SpinnakerGrabber] Selected Left: " << cfg_.serial_left
                      << ", Right: " << cfg_.serial_right << "\n";
        } else {
            camList.Clear();
            throw std::runtime_error("SpinnakerGrabber: Need at least 2 cameras connected, but found " +
                                     std::to_string(num_cams));
        }
    }

    camList.Clear();

    // Default sync tolerance to half the frame period in milliseconds
    if (cfg_.sync_tolerance_ms > 0.0) {
        sync_tol_ms_ = cfg_.sync_tolerance_ms;
    } else {
        double frame_period_ms = (cfg_.frame_rate > 0.0) ? (1000.0 / cfg_.frame_rate) : 33.3;
        sync_tol_ms_ = frame_period_ms * 0.75; // allow up to 75% frame period difference
    }
}

SpinnakerGrabber::~SpinnakerGrabber()
{
    stop();
}

GrabberStats SpinnakerGrabber::stats() const
{
    GrabberStats s;
    s.left_frames        = n_left_.load();
    s.right_frames       = n_right_.load();
    s.pairs              = n_pairs_.load();
    s.dropped_unmatched  = n_dropped_.load();
    s.incomplete         = n_incomplete_.load();
    return s;
}

void SpinnakerGrabber::configureCamera(CameraPtr cam, bool is_left)
{
    // ── Pixel format — keep as raw Bayer ─────────────────────────────────────
    setEnum(cam, "PixelFormat", "BayerRG8");

    // ── Binning ───────────────────────────────────────────────────────────────
    if (cfg_.binning > 1) {
        setInt(cam, "BinningHorizontal", cfg_.binning);
        setInt(cam, "BinningVertical",   cfg_.binning);
    }

    // ── Frame rate & Exposure ─────────────────────────────────────────────────
    setBool(cam, "AcquisitionFrameRateEnable", true);
    setFloat(cam, "AcquisitionFrameRate", cfg_.frame_rate);

    setEnum(cam, "ExposureAuto", "Off");
    setEnum(cam, "ExposureMode", "Timed");
    // Max exposure time safely below frame period to avoid frame drops
    double max_safe_exp_us = (cfg_.frame_rate > 0.0) ? (1000000.0 / cfg_.frame_rate) - 500.0 : 33000.0;
    double exp_us = std::min(cfg_.exposure_time_us, max_safe_exp_us);
    setFloat(cam, "ExposureTime", exp_us);

    // ── Gain ──────────────────────────────────────────────────────────────────
    if (cfg_.gain_auto) {
        setEnum(cam, "GainAuto", "Continuous");
    } else {
        setEnum(cam, "GainAuto", "Off");
        setFloat(cam, "Gain", cfg_.gain_db);
    }

    // ── White balance ─────────────────────────────────────────────────────────
    if (cfg_.balance_white_auto) {
        setEnum(cam, "BalanceWhiteAuto", "Continuous");
    }

    // ── Chunk data (FrameID + Timestamp) ──────────────────────────────────────
    setBool(cam, "ChunkModeActive", true);

    auto enableChunk = [&](const std::string & selector) {
        setEnum(cam, "ChunkSelector", selector);
        setBool(cam, "ChunkEnable", true);
    };
    enableChunk("FrameID");
    enableChunk("Timestamp");
    enableChunk("ExposureTime");

    // ── Trigger ───────────────────────────────────────────────────────────────
    if (cfg_.trigger_mode) {
        setEnum(cam, "LineSelector",  "Line3");
        setEnum(cam, "LineMode",      "Input");
        setEnum(cam, "TriggerSelector", "FrameStart");
        setEnum(cam, "TriggerSource",   "Line3");
        setEnum(cam, "TriggerOverlap",  "ReadOut");
        setFloat(cam, "TriggerDelay",   static_cast<double>(cfg_.trigger_delay_us));
        setEnum(cam, "TriggerMode",     "On");
    } else {
        setEnum(cam, "TriggerMode", "Off");
    }

    // ── Stream buffer ─────────────────────────────────────────────────────────
    INodeMap & sNodeMap = cam->GetTLStreamNodeMap();
    CEnumerationPtr bufHandling = sNodeMap.GetNode("StreamBufferHandlingMode");
    if (IsAvailable(bufHandling) && IsWritable(bufHandling)) {
        CEnumEntryPtr entry = bufHandling->GetEntryByName("NewestOnly");
        if (IsAvailable(entry)) bufHandling->SetIntValue(entry->GetValue());
    }

    std::cout << "[SpinnakerGrabber] Camera (" << (is_left ? "Left" : "Right")
              << " " << (is_left ? cfg_.serial_left : cfg_.serial_right)
              << ") configured: " << (cfg_.trigger_mode ? "HW Trigger (Line3)" : "Continuous")
              << ", FPS target=" << cfg_.frame_rate
              << ", Exp=" << exp_us << " us\n";
}

void SpinnakerGrabber::start()
{
    if (running_.load()) return;

    if (!cam_left_->IsInitialized())  cam_left_->Init();
    if (!cam_right_->IsInitialized()) cam_right_->Init();

    configureCamera(cam_left_,  true);
    configureCamera(cam_right_, false);

    // Arm acquisition on both cameras before threads run (essential for HW trigger)
    cam_left_->BeginAcquisition();
    cam_right_->BeginAcquisition();

    running_.store(true);
    started_ = true;

    left_grab_thread_  = std::thread(&SpinnakerGrabber::grabThread, this, cam_left_,  true);
    right_grab_thread_ = std::thread(&SpinnakerGrabber::grabThread, this, cam_right_, false);
    sync_thread_       = std::thread(&SpinnakerGrabber::syncThread, this);
}

void SpinnakerGrabber::stop()
{
    if (!started_) return;
    running_.store(false);

    cv_.notify_all();

    if (left_grab_thread_.joinable())  left_grab_thread_.join();
    if (right_grab_thread_.joinable()) right_grab_thread_.join();
    if (sync_thread_.joinable())       sync_thread_.join();

    started_ = false;

    try {
        if (cam_left_  && cam_left_->IsStreaming())  cam_left_->EndAcquisition();
        if (cam_right_ && cam_right_->IsStreaming()) cam_right_->EndAcquisition();
        if (cam_left_)  cam_left_->DeInit();
        if (cam_right_) cam_right_->DeInit();
    } catch (const Spinnaker::Exception & e) {
        std::cerr << "SpinnakerGrabber::stop exception: " << e.what() << "\n";
    }

    cam_left_ = nullptr;
    cam_right_ = nullptr;

    if (system_) {
        system_->ReleaseInstance();
    }
}

void SpinnakerGrabber::grabThread(CameraPtr cam, bool is_left)
{
    int consecutive_errors = 0;

    while (running_.load()) {
        try {
            ImagePtr raw = cam->GetNextImage(cfg_.acquire_timeout_ms);

            if (raw->IsIncomplete()) {
                raw->Release();
                n_incomplete_++;
                continue;
            }

            auto now_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());

            ChunkData chunk  = raw->GetChunkData();
            uint64_t  fid    = static_cast<uint64_t>(chunk.GetFrameID());
            uint64_t  ts_ns  = static_cast<uint64_t>(chunk.GetTimestamp());

            const auto received_at = std::chrono::steady_clock::now();
            int w = static_cast<int>(raw->GetWidth());
            int h = static_cast<int>(raw->GetHeight());
            // Respect SDK row padding; own the pixels before releasing its buffer.
            cv::Mat view(h, w, CV_8UC1, raw->GetData(), raw->GetStride());
            cv::Mat bayer = view.clone();
            raw->Release();

            RawFrame frame{std::move(bayer), fid, ts_ns, received_at, now_ns};
            consecutive_errors = 0;

            {
                std::lock_guard<std::mutex> lock(mtx_);
                auto & q = is_left ? left_queue_ : right_queue_;
                if (q.size() >= kQueueDepth) {
                    q.pop_front();
                    n_dropped_++;
                }
                q.push_back(std::move(frame));
                if (is_left) n_left_++; else n_right_++;
            }
            cv_.notify_one();

        } catch (const Spinnaker::Exception & e) {
            if (!running_.load()) break;

            ++consecutive_errors;
            std::cerr << "SpinnakerGrabber grab error ["
                      << (is_left ? "left" : "right") << "] "
                      << consecutive_errors << "/" << cfg_.max_consecutive_errors
                      << ": " << e.what() << "\n";

            if (consecutive_errors >= cfg_.max_consecutive_errors) {
                std::cerr << "SpinnakerGrabber: camera "
                          << (is_left ? "left" : "right")
                          << " appears disconnected — triggering shutdown!\n";
                running_.store(false);
                cv_.notify_all();
                rclcpp::shutdown();
                break;
            }
        }
    }
}

void SpinnakerGrabber::syncThread()
{
    const int64_t tol_ns = static_cast<int64_t>(sync_tol_ms_ * 1e6);

    while (running_.load()) {
        RawFrame left_frame;
        RawFrame right_frame;
        bool matched = false;

        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                return (!left_queue_.empty() && !right_queue_.empty()) || !running_.load();
            });

            if (!running_.load()) break;

            while (!left_queue_.empty() && !right_queue_.empty()) {
                const auto & l = left_queue_.front();
                const auto & r = right_queue_.front();

                // If hardware trigger with matching FrameIDs
                if (cfg_.trigger_mode && l.frame_id == r.frame_id) {
                    left_frame = std::move(left_queue_.front());
                    right_frame = std::move(right_queue_.front());
                    left_queue_.pop_front();
                    right_queue_.pop_front();
                    matched = true;
                    break;
                }

                // Match by arrival host timestamp (ns)
                int64_t dt_ns = static_cast<int64_t>(l.host_ns) - static_cast<int64_t>(r.host_ns);

                if (std::abs(dt_ns) <= tol_ns) {
                    left_frame = std::move(left_queue_.front());
                    right_frame = std::move(right_queue_.front());
                    left_queue_.pop_front();
                    right_queue_.pop_front();
                    matched = true;
                    break;
                } else if (dt_ns > tol_ns) {
                    // Left arrived later than right -> Right is too old, discard right
                    right_queue_.pop_front();
                    n_dropped_++;
                } else {
                    // Right arrived later than left -> Left is too old, discard left
                    left_queue_.pop_front();
                    n_dropped_++;
                }
            }
        }

        if (matched) {
            n_pairs_++;
            if (callback_) {
                callback_(left_frame, right_frame);
            }
        }
    }
}

}  // namespace passive_stereo_capture
