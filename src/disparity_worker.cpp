#include "disparity_worker.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


namespace passive_stereo_capture
{

DisparityWorker::DisparityWorker(
    PointCloud2Pub pub_cloud,
    CompressedImagePub pub_disp_img,
    const StereoCalib & calib,
    const Config & cfg)
: pub_cloud_(std::move(pub_cloud)),
  pub_disp_img_(std::move(pub_disp_img)),
  cfg_(cfg),
  calib_(calib)
{
    if (cfg_.backend != "retinify") {
        if (!calib_.isLoaded() || std::abs(calib_.P2().at<double>(0, 3)) < 1e-12 ||
            std::abs(calib_.P2().at<double>(1, 3)) > 1e-12)
            throw std::invalid_argument("Conventional stereo requires loaded horizontal stereo calibration");
        conventional_ = std::make_unique<ConventionalStereo>(cfg_.backend, cfg_.stereo);
    }
}

DisparityWorker::~DisparityWorker()
{
    stop();
    if (h_pinned_disp_)      { cudaFreeHost(h_pinned_disp_); }
    if (h_pinned_xyz_)       { cudaFreeHost(h_pinned_xyz_);  }
    if (h_pinned_rect_left_) { cudaFreeHost(h_pinned_rect_left_); }
}

void DisparityWorker::push(StereoFramePtr frame) { queue_.push(std::move(frame)); }

void DisparityWorker::start()
{
    if (running_.load()) return;
    running_.store(true);
    thread_ = std::thread(&DisparityWorker::run, this);
}

void DisparityWorker::stop()
{
    running_.store(false);
    queue_.shutdown();
    if (thread_.joinable()) thread_.join();
}

bool DisparityWorker::initPipeline(uint32_t W, uint32_t H)
{
    retinify::DepthMode mode = retinify::DepthMode::ACCURATE;
    if      (cfg_.depth_mode == "fast")     mode = retinify::DepthMode::FAST;
    else if (cfg_.depth_mode == "balanced") mode = retinify::DepthMode::BALANCED;

    retinify::CalibrationParameters calib{};
    calib.imageWidth  = W;
    calib.imageHeight = H;

    // Scale calibration intrinsics if actual image resolution differs from calib file
    double sx = (calib_.width() > 0) ? (static_cast<double>(W) / calib_.width()) : 1.0;
    double sy = (calib_.height() > 0) ? (static_cast<double>(H) / calib_.height()) : 1.0;

    if (std::abs(sx - 1.0) > 1e-3 || std::abs(sy - 1.0) > 1e-3) {
        std::cout << "[DisparityWorker] Scaling calibration parameters by ("
                  << sx << ", " << sy << ") to match frame size " << W << "x" << H << std::endl;
    }
    else std::cout << "[DisparityWorker] No scaling factor: " << sx << ", " << sy << std::endl;
    // Retinify owns rectification: give it RAW images and RAW calibration.
    // Never combine raw pixels with rectified P matrices / identity extrinsics.
    calib.leftIntrinsics.fx  = calib_.fx_l() * sx;
    calib.leftIntrinsics.fy  = calib_.fy_l() * sy;
    calib.leftIntrinsics.cx  = (calib_.cx_l() + 0.5) * sx - 0.5;
    calib.leftIntrinsics.cy  = (calib_.cy_l() + 0.5) * sy - 0.5;
    calib.rightIntrinsics.fx = calib_.fx_r() * sx;
    calib.rightIntrinsics.fy = calib_.fy_r() * sy;
    calib.rightIntrinsics.cx = (calib_.cx_r() + 0.5) * sx - 0.5;
    calib.rightIntrinsics.cy = (calib_.cy_r() + 0.5) * sy - 0.5;
    calib.leftDistortion = calib_.toRetinifyDistortion(calib_.leftDistortions());
    calib.rightDistortion = calib_.toRetinifyDistortion(calib_.rightDistortions());
    calib.rotation = calib_.rot();
    calib.translation = calib_.trans();

    if (cfg_.rectify_on_cpu) {
        // Inputs were already rectified by OpenCV. Preserve those intrinsics;
        // zero distortion and identity rotation prevent a second geometric warp.
        const auto & P1 = calib_.P1();
        const auto & P2 = calib_.P2();
        calib.leftIntrinsics = {P1.at<double>(0,0)*sx, P1.at<double>(1,1)*sy,
            (P1.at<double>(0,2)+0.5)*sx-0.5, (P1.at<double>(1,2)+0.5)*sy-0.5};
        calib.rightIntrinsics = {P2.at<double>(0,0)*sx, P2.at<double>(1,1)*sy,
            (P2.at<double>(0,2)+0.5)*sx-0.5, (P2.at<double>(1,2)+0.5)*sy-0.5};
        calib.leftDistortion = {}; calib.rightDistortion = {};
        calib.rotation = retinify::Identity();
        calib.translation = {P2.at<double>(0,3)/P2.at<double>(0,0), 0, 0};
    }

    auto status = pipeline_.Initialize(W, H, retinify::PixelFormat::RGB8, mode, calib);
    if (!status.IsOK()) {
        std::cerr << "[DisparityWorker] Retinify Initialize failed with code "
                  << static_cast<int>(status.Code()) << std::endl;
        return false;
    }

    // ── Disparity pinned buffer ───────────────────────────────────────────────
    size_t disp_bytes = static_cast<size_t>(W) * H * sizeof(float);
    if (pinned_disp_bytes_ < disp_bytes) {
        if (h_pinned_disp_) { cudaFreeHost(h_pinned_disp_); h_pinned_disp_ = nullptr; }
        if (cudaHostAlloc(&h_pinned_disp_, disp_bytes, cudaHostAllocDefault) == cudaSuccess) {
            pinned_disp_bytes_ = disp_bytes;
        } else {
            h_pinned_disp_ = nullptr;
            cpu_disp_buf_.resize(static_cast<size_t>(W) * H);
        }
    }

    if (pub_cloud_) {
        // ── XYZ pinned buffer ─────────────────────────────────────────────────────
        size_t xyz_bytes = static_cast<size_t>(W) * H * 3 * sizeof(float);
        if (pinned_xyz_bytes_ < xyz_bytes) {
            if (h_pinned_xyz_) { cudaFreeHost(h_pinned_xyz_); h_pinned_xyz_ = nullptr; }
            if (cudaHostAlloc(&h_pinned_xyz_, xyz_bytes, cudaHostAllocDefault) != cudaSuccess) {
                h_pinned_xyz_ = nullptr;
                return false;
            }
            pinned_xyz_bytes_ = xyz_bytes;
        }

        size_t pt_size = cfg_.publish_confidence ?
            sizeof(PointXYZRGBConf) : sizeof(PointXYZRGB);
        size_t max_pts = static_cast<size_t>(W) * H;
        cpu_point_buf_.resize(max_pts * pt_size);

        // ── Rectified Left RGB pinned buffer ──────────────────────────────────────
        size_t rect_left_bytes = static_cast<size_t>(W) * H * 3 * sizeof(uint8_t);
        if (pinned_rect_left_bytes_ < rect_left_bytes) {
            if (h_pinned_rect_left_) { cudaFreeHost(h_pinned_rect_left_); h_pinned_rect_left_ = nullptr; }
            if (cudaHostAlloc(&h_pinned_rect_left_, rect_left_bytes, cudaHostAllocDefault) == cudaSuccess) {
                pinned_rect_left_bytes_ = rect_left_bytes;
                rect_left_rgb_ = cv::Mat(static_cast<int>(H), static_cast<int>(W), CV_8UC3, h_pinned_rect_left_);
            } else {
                h_pinned_rect_left_ = nullptr;
                rect_left_rgb_.create(static_cast<int>(H), static_cast<int>(W), CV_8UC3);
            }
        }

        if (h_pinned_rect_left_) {
            rect_left_rgb_ = cv::Mat(static_cast<int>(H), static_cast<int>(W), CV_8UC3, h_pinned_rect_left_);
        }

    } // Cloud-only buffers are unnecessary for compressed disparity output.

    pipeline_W_ = W;
    pipeline_H_ = H;
    pipeline_init_ = true;
    return true;
}

size_t DisparityWorker::compactCloud(
    const float * xyz, const uint8_t * img_rgb,
    uint32_t W, uint32_t H,
    int u0, int v0, int u1, int v1, int step,
    float max_dist_sq, int img_step,
    void * out_buf, bool with_conf,
    const float * disp, int disp_step,
    int conf_radius, float conf_alpha, float min_conf)
{
    auto * dst = static_cast<uint8_t *>(out_buf);
    size_t total_pts = 0;
    for (int v = v0; v < v1; v += step) {
        const float   * xyz_row = xyz + v * static_cast<int>(W) * 3;
        const uint8_t * img_row = img_rgb + v * img_step;

        for (int u = u0; u < u1; u += step) {
            float X = xyz_row[u*3+0], Y = xyz_row[u*3+1], Z = xyz_row[u*3+2];
            if (Z <= 0.05f || !std::isfinite(X) || !std::isfinite(Y) || !std::isfinite(Z)) continue;
            if (max_dist_sq > 0.f && (X*X + Y*Y + Z*Z) > max_dist_sq) continue;

            float conf = 1.f;
            if (with_conf && disp && conf_radius > 0) {
                float sum = 0.f, sum_sq = 0.f; int nb = 0;
                for (int dv = -conf_radius; dv <= conf_radius; ++dv) {
                    int vv = v + dv;
                    if (vv < 0 || vv >= (int)H) continue;
                    const float * nb_row = reinterpret_cast<const float *>(
                        reinterpret_cast<const char *>(disp) + vv * disp_step);
                    for (int du = -conf_radius; du <= conf_radius; ++du) {
                        int uu = u + du;
                        if (uu < 0 || uu >= (int)W) continue;
                        float dn = nb_row[uu];
                        if (std::isfinite(dn) && dn > 0.f) { sum += dn; sum_sq += dn*dn; nb++; }
                    }
                }
                if (nb > 1) {
                    float mean  = sum / nb;
                    float sigma = std::sqrt(std::max((sum_sq/nb) - mean*mean, 0.f));
                    conf = 1.f / (1.f + conf_alpha * sigma);
                    if (conf < min_conf) continue;
                } else {
                    continue;
                }
            }

            uint8_t r = img_row[u*3+2], g = img_row[u*3+1], b = img_row[u*3+0]; // BGR format
            uint32_t rgb = (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);

            if (with_conf) {
                const PointXYZRGBConf point{X, Y, Z, rgb, conf};
                std::memcpy(dst + total_pts * sizeof(point), &point, sizeof(point));
            } else {
                const PointXYZRGB point{X, Y, Z, rgb};
                std::memcpy(dst + total_pts * sizeof(point), &point, sizeof(point));
            }
            ++total_pts;
        }
    }
    return total_pts;
}

void DisparityWorker::run()
{
    std::ofstream trace;
    if (!cfg_.trace_path.empty()) {
        trace.open(cfg_.trace_path);
        if (!trace) std::cerr << "Cannot open disparity trace file; profiling disabled\n";
        trace << "frame_id,stamp_ns,queue_ms,prepare_ms,execute_ms,output_ms,total_ms,points\n";
    }
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;
        // No output consumer: avoid rectification, inference and image enhancement.
        // Capture and SLAM continue independently at their configured rates.
        if ((!pub_cloud_ || pub_cloud_->get_subscription_count() == 0) &&
            (!pub_disp_img_ || pub_disp_img_->get_subscription_count() == 0)) continue;
        if (cfg_.process_hz > 0.0 && !process_limiter_.ready(frame->stamp.seconds(), cfg_.process_hz)) continue;
        const auto processing_start = std::chrono::steady_clock::now();
        cv::Mat input_left, input_right;
        if (conventional_ || cfg_.rectify_on_cpu) {
            // Rectification maps use calibration dimensions, including with camera binning.
            cv::Mat native_left = frame->left_rgb, native_right = frame->right_rgb;
            const cv::Size native_size(calib_.width(), calib_.height());
            if (native_left.size() != native_size) {
                cv::resize(frame->left_rgb, native_left, native_size);
                cv::resize(frame->right_rgb, native_right, native_size);
            }
            calib_.rectify(native_left, native_right, input_left, input_right);
        } else {
            input_left = frame->left_rgb;
            input_right = frame->right_rgb;
        }
        if (cfg_.width > 0 && cfg_.height > 0 &&
            (input_left.cols != cfg_.width || input_left.rows != cfg_.height)) {
            const auto interpolation = cfg_.width < input_left.cols && cfg_.height < input_left.rows
                ? cv::INTER_AREA : cv::INTER_LINEAR;
            cv::resize(input_left, input_left, cv::Size(cfg_.width, cfg_.height), 0, 0, interpolation);
            cv::resize(input_right, input_right, cv::Size(cfg_.width, cfg_.height), 0, 0, interpolation);
        }
        uint32_t W = static_cast<uint32_t>(input_left.cols);
        uint32_t H = static_cast<uint32_t>(input_left.rows);

        if (!conventional_ && (!pipeline_init_ || pipeline_W_ != W || pipeline_H_ != H)) {
            if (!initPipeline(W, H)) {
                std::cerr << "[DisparityWorker] Failed to initialize Retinify pipeline for "
                          << W << "x" << H << "!\n";
                continue;
            }
        }

        cv::Mat left_rgb, right_rgb;
        if (cfg_.clahe_enabled) {
            left_rgb  = applyClaheRGB(input_left, clahe_);
            right_rgb = applyClaheRGB(input_right, clahe_);
        } else {
            left_rgb  = input_left;
            right_rgb = input_right;
        }

        const auto prepared = std::chrono::steady_clock::now();
        if (conventional_) {
            try {
                conventional_->compute(left_rgb, right_rgb, conventional_disp_);
            } catch (const std::exception & e) {
                std::cerr << "[DisparityWorker] " << e.what() << std::endl;
                continue;
            }
        } else {
            auto status = pipeline_.Execute(
                left_rgb.ptr<uint8_t>(), left_rgb.step[0],
                right_rgb.ptr<uint8_t>(), right_rgb.step[0]);
            if (!status.IsOK()) continue;
        }

        const auto executed = std::chrono::steady_clock::now();
        size_t cloud_points = 0;
        processed_frames_++;
        metrics_.record(processing_start, frame->received_at);

        bool has_cloud_sub = (pub_cloud_ && pub_cloud_->get_subscription_count() > 0);
        bool has_img_sub   = (pub_disp_img_ && pub_disp_img_->get_subscription_count() > 0);

        const double stamp_sec = frame->stamp.seconds();
        const bool pub_cloud_now = has_cloud_sub && cloud_limiter_.ready(stamp_sec, cfg_.cloud_hz);
        const bool pub_img_now = has_img_sub && image_limiter_.ready(stamp_sec, cfg_.image_hz);

        // Retrieve disparity if needed for pointcloud confidence gating OR for image publishing
        float * disp_ptr = conventional_ ? conventional_disp_.ptr<float>() : h_pinned_disp_ ? h_pinned_disp_ : cpu_disp_buf_.data();
        bool disp_retrieved = static_cast<bool>(conventional_);
        if (!conventional_ && (pub_img_now || (pub_cloud_now && cfg_.publish_confidence))) {
            auto disp_status = pipeline_.RetrieveDisparity(disp_ptr, W * sizeof(float));
            disp_retrieved = disp_status.IsOK();
        }

        // 1. Lightweight Disparity Image Visualizer (Colormap Jet)
        if (pub_img_now && disp_retrieved) {
            cv::Mat disp_colored(static_cast<size_t>(H), static_cast<size_t>(W), CV_8UC3);
            bool colorized = false;
            if (conventional_) {
                cv::Mat normalized;
                conventional_disp_.convertTo(normalized, CV_8U, 255.0 / cfg_.stereo.num_disparities,
                    -255.0 * cfg_.stereo.min_disparity / cfg_.stereo.num_disparities);
                cv::applyColorMap(normalized, disp_colored, cv::COLORMAP_JET);
                disp_colored.setTo(cv::Scalar::all(0), conventional_disp_ != conventional_disp_);
                colorized = true;
            } else {
                auto col_status = retinify::ColorizeDisparity(disp_ptr, W*sizeof(float), disp_colored.ptr<uint8_t>(), disp_colored.step[0], W, H, 256.0f);
                colorized = col_status.IsOK();
                if (colorized) cv::cvtColor(disp_colored, disp_colored, cv::COLOR_RGB2BGR);
            }
            if (colorized) {
                cv::imencode(".jpg", disp_colored, disp_jpeg_buf_, {cv::IMWRITE_JPEG_QUALITY, 20});

                sensor_msgs::msg::CompressedImage img_msg;
                img_msg.header.stamp    = frame->stamp;
                img_msg.header.frame_id = cfg_.frame_id;
                img_msg.format          = "jpeg";
                img_msg.data            = disp_jpeg_buf_;
                pub_disp_img_->publish(img_msg);
            }
        }

        // 2. Heavy Dense PointCloud2 (only computed when someone is subscribed and throttled)
        if (pub_cloud_now && (conventional_ || h_pinned_xyz_)) {
            const float * xyz_ptr = h_pinned_xyz_;
            if (conventional_) {
                cv::reprojectImageTo3D(conventional_disp_, conventional_xyz_,
                    ConventionalStereo::scaledQ(calib_.Q(), cv::Size(calib_.width(), calib_.height()),
                        input_left.size()), false, CV_32F);
                xyz_ptr = conventional_xyz_.ptr<float>();
                cpu_point_buf_.resize(static_cast<size_t>(W) * H *
                    (cfg_.publish_confidence ? sizeof(PointXYZRGBConf) : sizeof(PointXYZRGB)));
            } else {
                auto pc_status = pipeline_.RetrievePointCloud(
                    h_pinned_xyz_, static_cast<size_t>(W) * 3 * sizeof(float));
                if (!pc_status.IsOK()) {
                    std::cerr << "[DisparityWorker] RetrievePointCloud failed: code "
                              << static_cast<int>(pc_status.Code()) << std::endl;
                    continue;
                }
            }
            uint8_t * color_ptr;
            std::size_t color_stride;
            if(!conventional_){
                color_ptr = rect_left_rgb_.ptr<uint8_t>();
                color_stride = rect_left_rgb_.step[0];
                auto rect_status = pipeline_.RetrieveRectifiedLeftImage(color_ptr, color_stride);
                if (!rect_status.IsOK()) {
                    // Unrectified color would not align with rectified XYZ.
                    continue;
                }
            }
            else{ color_ptr = input_left.ptr<uint8_t>(); color_stride = input_left.step[0];}


            float  sampling = static_cast<float>(std::clamp(cfg_.sampling_factor, 0.01, 1.0));
            int    step_px  = std::max(1, static_cast<int>(1.f / sampling));
            double crop     = std::clamp(cfg_.crop_factor, 0.01, 1.0);
            int cw = static_cast<int>(W * crop), ch = static_cast<int>(H * crop);
            int u0 = (static_cast<int>(W) - cw) / 2, v0 = (static_cast<int>(H) - ch) / 2;
            int u1 = u0 + cw, v1 = v0 + ch;
            float mdsq = (cfg_.max_dist > 0) ?
                static_cast<float>(cfg_.max_dist * cfg_.max_dist) : -1.f;

            bool wconf = cfg_.publish_confidence && disp_retrieved;

            size_t valid = compactCloud(
                xyz_ptr, color_ptr,
                W, H, u0, v0, u1, v1, step_px,
                mdsq, static_cast<int>(color_stride),
                cpu_point_buf_.data(), wconf,
                disp_ptr, static_cast<int>(W * sizeof(float)),
                cfg_.confidence_radius,
                static_cast<float>(cfg_.confidence_alpha),
                static_cast<float>(cfg_.min_confidence));

            cloud_points = valid;
            if (valid > 0) {
                uint32_t pt_step = wconf ? sizeof(PointXYZRGBConf) : sizeof(PointXYZRGB);
                size_t   data_sz = valid * pt_step;

                auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
                cloud->header.stamp    = frame->stamp;
                cloud->header.frame_id = cfg_.frame_id;
                cloud->height          = 1;
                cloud->width           = static_cast<uint32_t>(valid);
                cloud->is_dense        = false;
                cloud->is_bigendian    = false;

                sensor_msgs::PointCloud2Modifier mod(*cloud);
                if (wconf) {
                    mod.setPointCloud2Fields(5,
                        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "rgb", 1, sensor_msgs::msg::PointField::UINT32,
                        "confidence", 1, sensor_msgs::msg::PointField::FLOAT32);
                } else {
                    mod.setPointCloud2Fields(4,
                        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                        "rgb", 1, sensor_msgs::msg::PointField::UINT32);
                }
                cloud->point_step = pt_step;
                cloud->row_step   = cloud->width * pt_step;

                cloud->data.resize(data_sz);
                std::memcpy(cloud->data.data(), cpu_point_buf_.data(), data_sz);

                pub_cloud_->publish(std::move(cloud));
                published_clouds_++;
            } else {
                empty_clouds_++;
            }
        }
        if (trace) {
            const auto finish = std::chrono::steady_clock::now();
            const auto ms = [](auto a, auto b) { return std::chrono::duration<double, std::milli>(b-a).count(); };
            trace << frame->frame_id << ',' << frame->stamp.nanoseconds() << ','
                  << ms(frame->received_at, processing_start) << ','
                  << ms(processing_start, prepared) << ',' << ms(prepared, executed) << ','
                  << ms(executed, finish) << ',' << ms(processing_start, finish) << ',' << cloud_points << '\n';
        }
        metrics_.record(processing_start, frame->received_at);
    }
}

}  // namespace passive_stereo_capture
