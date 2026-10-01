#include "disparity_worker.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace passive_stereo_capture
{

DisparityWorker::DisparityWorker(
    PointCloud2Pub pub_cloud,
    const StereoRectifier & rect,
    const Config & cfg)
: pub_cloud_(std::move(pub_cloud)),
  cfg_(cfg),
  fx_(rect.fx()), fy_(rect.fy()),
  cx_(rect.cx()), cy_(rect.cy()),
  baseline_m_(rect.baseline())
{}

DisparityWorker::~DisparityWorker()
{
    stop();
    if (h_pinned_disp_) { cudaFreeHost(h_pinned_disp_); }
    if (h_pinned_xyz_)  { cudaFreeHost(h_pinned_xyz_);  }
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
    // Rectified intrinsics
    calib.leftIntrinsics.fx  = fx_;
    calib.leftIntrinsics.fy  = fy_;
    calib.leftIntrinsics.cx  = cx_;
    calib.leftIntrinsics.cy  = cy_;
    calib.rightIntrinsics    = calib.leftIntrinsics;
    calib.rotation           = retinify::Identity();
    calib.translation        = {-std::abs(baseline_m_), 0.0, 0.0};

    auto status = pipeline_.Initialize(W, H, retinify::PixelFormat::RGB8, mode, calib);
    if (!status.IsOK()) return false;

    size_t disp_bytes = W * H * sizeof(float);
    if (pinned_disp_bytes_ < disp_bytes) {
        if (h_pinned_disp_) cudaFreeHost(h_pinned_disp_);
        if (cudaHostAlloc(&h_pinned_disp_, disp_bytes, cudaHostAllocDefault) != cudaSuccess) {
            h_pinned_disp_ = nullptr;
            cpu_disp_buf_.resize(W * H);
        } else {
            pinned_disp_bytes_ = disp_bytes;
        }
    }

    size_t xyz_bytes = W * H * 3 * sizeof(float);
    if (pinned_xyz_bytes_ < xyz_bytes) {
        if (h_pinned_xyz_) cudaFreeHost(h_pinned_xyz_);
        if (cudaHostAlloc(&h_pinned_xyz_, xyz_bytes, cudaHostAllocDefault) != cudaSuccess) {
            h_pinned_xyz_ = nullptr;
            return false;
        }
        pinned_xyz_bytes_ = xyz_bytes;
    }

    size_t pt_size = cfg_.publish_confidence ?
        sizeof(PointXYZRGBConf) : sizeof(PointXYZRGB);
    cpu_point_buf_.resize(static_cast<size_t>(W) * H * pt_size);

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
    size_t count = 0;
    auto * dst_rgb  = reinterpret_cast<PointXYZRGB *>(out_buf);
    auto * dst_conf = reinterpret_cast<PointXYZRGBConf *>(out_buf);

    for (int v = v0; v < v1; v += step) {
        const float   * xyz_row = xyz + v * static_cast<int>(W) * 3;
        const uint8_t * img_row = img_rgb + v * img_step;

        for (int u = u0; u < u1; u += step) {
            float X = xyz_row[u*3+0], Y = xyz_row[u*3+1], Z = xyz_row[u*3+2];
            if (Z <= 0.f) continue;

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
                        if (dn > 0.f) { sum += dn; sum_sq += dn*dn; nb++; }
                    }
                }
                if (nb > 1) {
                    float mean = sum / nb;
                    float sigma = std::sqrt(std::max((sum_sq/nb) - mean*mean, 0.f));
                    conf = 1.f / (1.f + conf_alpha * sigma);
                    if (conf < min_conf) continue;
                } else { continue; }
            }

            uint8_t r = img_row[u*3+0], g = img_row[u*3+1], b = img_row[u*3+2];
            uint32_t rgb = (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);

            if (with_conf) dst_conf[count++] = {X, Y, Z, rgb, conf};
            else           dst_rgb [count++] = {X, Y, Z, rgb};
        }
    }
    return count;
}

void DisparityWorker::run()
{
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;

        uint32_t W = static_cast<uint32_t>(frame->left.cols);
        uint32_t H = static_cast<uint32_t>(frame->left.rows);

        if (!pipeline_init_) {
            if (!initPipeline(W, H)) {
                RCLCPP_ERROR(rclcpp::get_logger("disparity_worker"),
                             "Failed to initialise Retinify pipeline!");
                continue;
            }
        }

        auto status = pipeline_.Execute(
            frame->left.ptr<uint8_t>(),  frame->left.step[0],
            frame->right.ptr<uint8_t>(), frame->right.step[0]);
        if (!status.IsOK()) continue;

        // Retrieve disparity for confidence gating
        float * disp_ptr = h_pinned_disp_ ? h_pinned_disp_ : cpu_disp_buf_.data();
        auto disp_status = pipeline_.RetrieveDisparity(disp_ptr, W * sizeof(float));
        if (!disp_status.IsOK()) continue;

        // Retrieve dense XYZ point cloud from GPU
        if (!h_pinned_xyz_) continue;
        auto pc_status = pipeline_.RetrievePointCloud(h_pinned_xyz_, W * 3 * sizeof(float));
        if (!pc_status.IsOK()) continue;

        // Sampling / crop parameters
        float sampling = static_cast<float>(std::clamp(cfg_.sampling_factor, 0.01, 1.0));
        int   step_px  = std::max(1, static_cast<int>(1.f / sampling));
        double crop    = std::clamp(cfg_.crop_factor, 0.01, 1.0);
        int cw = static_cast<int>(W * crop), ch = static_cast<int>(H * crop);
        int u0 = (static_cast<int>(W) - cw) / 2, v0 = (static_cast<int>(H) - ch) / 2;
        int u1 = u0 + cw, v1 = v0 + ch;
        float mdsq = (cfg_.max_dist > 0) ?
            static_cast<float>(cfg_.max_dist * cfg_.max_dist) : -1.f;

        bool wconf = cfg_.publish_confidence;
        uint32_t pt_step = wconf ? sizeof(PointXYZRGBConf) : sizeof(PointXYZRGB);

        size_t valid = compactCloud(
            h_pinned_xyz_, frame->left.ptr<uint8_t>(),
            W, H, u0, v0, u1, v1, step_px,
            mdsq, static_cast<int>(frame->left.step[0]),
            cpu_point_buf_.data(), wconf,
            disp_ptr, static_cast<int>(W * sizeof(float)),
            cfg_.confidence_radius,
            static_cast<float>(cfg_.confidence_alpha),
            static_cast<float>(cfg_.min_confidence));

        // Build and publish PointCloud2
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
        cloud->data.resize(cloud->row_step);
        if (valid > 0)
            std::memcpy(cloud->data.data(), cpu_point_buf_.data(), cloud->row_step);

        pub_cloud_->publish(std::move(cloud));
    }
}

}  // namespace passive_stereo_capture
