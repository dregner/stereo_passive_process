#pragma once

#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{

/// Apply CLAHE to the L (luminance) channel of an RGB8 image in CIE Lab space.
/// Both input and output are RGB8.
inline cv::Mat applyClaheRGB(const cv::Mat & rgb, cv::Ptr<cv::CLAHE> & clahe)
{
    cv::Mat lab;
    cv::cvtColor(rgb, lab, cv::COLOR_RGB2Lab);

    std::vector<cv::Mat> channels(3);
    cv::split(lab, channels);
    clahe->apply(channels[0], channels[0]);

    cv::Mat merged_lab;
    cv::merge(channels, merged_lab);

    cv::Mat out_rgb;
    cv::cvtColor(merged_lab, out_rgb, cv::COLOR_Lab2RGB);
    return out_rgb;
}
inline cv::Mat applyClaheBGR(const cv::Mat & rgb, cv::Ptr<cv::CLAHE> & clahe)
{
    cv::Mat lab;
    cv::cvtColor(rgb, lab, cv::COLOR_RGB2Lab);

    std::vector<cv::Mat> channels(3);
    cv::split(lab, channels);
    clahe->apply(channels[0], channels[0]);

    cv::Mat merged_lab;
    cv::merge(channels, merged_lab);

    cv::Mat out_bgr;
    cv::cvtColor(merged_lab, out_bgr, cv::COLOR_Lab2BGR);
    return out_bgr;
}

}  // namespace passive_stereo_capture
