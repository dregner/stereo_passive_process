#pragma once

#include <chrono>
#include <filesystem>

#include <opencv2/core/mat.hpp>

namespace passive_stereo_capture
{

std::filesystem::path saveRawSnapshot(const std::filesystem::path & output_dir,
    std::chrono::system_clock::time_point requested_at,
    const cv::Mat & left_bayer, const cv::Mat & right_bayer);

}  // namespace passive_stereo_capture
