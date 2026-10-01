#include "spinnaker_grabber.hpp"

#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <iostream>

#include <opencv2/imgproc.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

namespace passive_stereo_capture
{

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static CameraPtr findCameraBySerial(
    const CameraList & camList, const std::string & serial)
{
    for (unsigned int i = 0; i < camList.GetSize(); ++i) {
        CameraPtr cam = camList.GetByIndex(i);
        cam->Init();
        std::string s = cam->DeviceSerialNumber.GetValue().c_str();
        if (s == serial) return cam;
        cam->DeInit();
    }
    throw std::runtime_error("SpinnakerGrabber: camera serial '" + serial + "' not found");
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

    cam_left_  = findCameraBySerial(camList, cfg_.serial_left);
    cam_right_ = findCameraBySerial(camList, cfg_.serial_right);

    camList.Clear();
}

SpinnakerGrabber::~SpinnakerGrabber() { stop(); }

void SpinnakerGrabber::configureCamera(CameraPtr cam, bool /*is_left*/)
{
    // ── Pixel format ──────────────────────────────────────────────────────────
    setEnum(cam, "PixelFormat", "BayerRG8");

    // ── Binning ───────────────────────────────────────────────────────────────
    setInt(cam, "BinningHorizontal", cfg_.binning);
    setInt(cam, "BinningVertical",   cfg_.binning);

    // ── Frame rate ────────────────────────────────────────────────────────────
    setBool(cam, "AcquisitionFrameRateEnable", true);
    setFloat(cam, "AcquisitionFrameRate", cfg_.frame_rate);

    // ── Exposure ──────────────────────────────────────────────────────────────
    setEnum(cam, "ExposureAuto", "Off");
    setEnum(cam, "ExposureMode", "Timed");
    setFloat(cam, "ExposureTime", cfg_.exposure_time_us);

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
        // Line3 as input (hardware trigger from Jetson GPIO)
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
}

void SpinnakerGrabber::start()
{
    if (running_.load()) return;

    configureCamera(cam_left_,  true);
    configureCamera(cam_right_, false);

    cam_left_->BeginAcquisition();
    cam_right_->BeginAcquisition();

    running_.store(true);

    left_grab_thread_  = std::thread(&SpinnakerGrabber::grabThread, this, cam_left_,  true);
    right_grab_thread_ = std::thread(&SpinnakerGrabber::grabThread, this, cam_right_, false);
    sync_thread_       = std::thread(&SpinnakerGrabber::syncThread, this);
}

void SpinnakerGrabber::stop()
{
    if (!running_.load()) return;
    running_.store(false);

    left_cv_.notify_all();
    right_cv_.notify_all();

    if (left_grab_thread_.joinable())  left_grab_thread_.join();
    if (right_grab_thread_.joinable()) right_grab_thread_.join();
    if (sync_thread_.joinable())       sync_thread_.join();

    try {
        if (cam_left_  && cam_left_->IsStreaming())  cam_left_->EndAcquisition();
        if (cam_right_ && cam_right_->IsStreaming()) cam_right_->EndAcquisition();
        if (cam_left_)  cam_left_->DeInit();
        if (cam_right_) cam_right_->DeInit();
    } catch (const Spinnaker::Exception & e) {
        std::cerr << "SpinnakerGrabber::stop exception: " << e.what() << "\n";
    }

    system_->ReleaseInstance();
}

void SpinnakerGrabber::grabThread(CameraPtr cam, bool is_left)
{
    // Spinnaker ImageProcessor for Bayer -> RGB8 conversion
    ImageProcessor proc;
    proc.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);

    while (running_.load()) {
        try {
            ImagePtr raw = cam->GetNextImage(cfg_.acquire_timeout_ms);
            if (raw->IsIncomplete()) {
                raw->Release();
                continue;
            }

            // Convert BayerRG8 -> RGB8 using Spinnaker processor
            ImagePtr rgb = proc.Convert(raw, PixelFormat_RGB8);
            raw->Release();

            // Wrap pixel data into cv::Mat (deep copy, since rgb will be released)
            int w = static_cast<int>(rgb->GetWidth());
            int h = static_cast<int>(rgb->GetHeight());
            cv::Mat mat(h, w, CV_8UC3);
            std::memcpy(mat.data, rgb->GetData(), static_cast<size_t>(w * h * 3));

            // Chunk data
            ChunkData chunk = rgb->GetChunkData();
            uint64_t fid    = static_cast<uint64_t>(chunk.GetFrameID());
            // Spinnaker timestamp in nanoseconds
            double ts_sec   = static_cast<double>(chunk.GetTimestamp()) * 1e-9;
            rgb->Release();

            RawFrame frame{std::move(mat), fid, ts_sec};

            if (is_left) {
                std::lock_guard<std::mutex> lock(left_mtx_);
                if (left_queue_.size() >= kQueueDepth) left_queue_.pop();
                left_queue_.push(std::move(frame));
                left_cv_.notify_one();
            } else {
                std::lock_guard<std::mutex> lock(right_mtx_);
                if (right_queue_.size() >= kQueueDepth) right_queue_.pop();
                right_queue_.push(std::move(frame));
                right_cv_.notify_one();
            }

        } catch (const Spinnaker::Exception & e) {
            if (running_.load()) {
                std::cerr << "Grab error (" << (is_left ? "left" : "right")
                          << "): " << e.what() << "\n";
            }
        }
    }
}

void SpinnakerGrabber::syncThread()
{
    while (running_.load()) {
        // Wait for left frame
        RawFrame left_frame;
        {
            std::unique_lock<std::mutex> lock(left_mtx_);
            left_cv_.wait_for(lock, std::chrono::milliseconds(500),
                [this] { return !left_queue_.empty() || !running_.load(); });
            if (!running_.load() && left_queue_.empty()) break;
            if (left_queue_.empty()) continue;
            left_frame = std::move(left_queue_.front());
            left_queue_.pop();
        }

        // Find matching right frame by FrameID
        RawFrame right_frame;
        bool matched = false;
        for (int attempt = 0; attempt < 20 && running_.load(); ++attempt) {
            std::unique_lock<std::mutex> lock(right_mtx_);
            right_cv_.wait_for(lock, std::chrono::milliseconds(100),
                [this] { return !right_queue_.empty() || !running_.load(); });

            while (!right_queue_.empty()) {
                auto & front = right_queue_.front();
                int64_t diff = static_cast<int64_t>(front.frame_id) -
                               static_cast<int64_t>(left_frame.frame_id);

                if (std::abs(diff) <= cfg_.frame_id_sync_tolerance) {
                    right_frame = std::move(front);
                    right_queue_.pop();
                    matched = true;
                    break;
                } else if (diff < 0) {
                    // Right is behind: discard right and fetch newer
                    right_queue_.pop();
                } else {
                    // Right is ahead: left was probably dropped, skip left
                    break;
                }
            }
            if (matched) break;
        }

        if (matched && callback_) {
            callback_(left_frame, right_frame);
        }
    }
}

}  // namespace passive_stereo_capture
