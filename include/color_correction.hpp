#pragma once
#include <cmath>
#include <stdexcept>
#include <vector>
#include <opencv2/core.hpp>

namespace passive_stereo_capture {
// Gainij maps input channel j to output channel i; channels are RGB.
inline cv::Matx33f colorCorrectionMatrix(const std::vector<double> & gains) {
    if (gains.size() != 9)
        throw std::invalid_argument("Color correction requires nine row-major RGB gains");
    cv::Matx33f matrix;
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(gains[i]) || std::abs(gains[i]) > 1e6)
            throw std::invalid_argument("Invalid color correction gain");
        matrix.val[i] = static_cast<float>(gains[i]);
    }
    return matrix;
}
inline void correctRgb(cv::Mat & rgb, const cv::Matx33f & matrix) {
    // Float intermediate preserves negative cross-channel contributions before
    // final rounding/clipping to the output's 8-bit [0,255] range.
    cv::Mat corrected;
    rgb.convertTo(corrected, CV_32F);
    cv::transform(corrected, corrected, matrix);
    corrected.convertTo(rgb, CV_8U);
}
}
