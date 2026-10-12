#pragma once
#include "color_correction.hpp"
#include <Spinnaker.h>
#include <ImageUtilityCCM.h>
#include <iomanip>
#include <locale>
#include <sstream>

namespace passive_stereo_capture {
inline Spinnaker::CCMSettings spinnakerCcmSettings(const std::vector<double> & gains) {
    colorCorrectionMatrix(gains); // Validate before handing coefficients to the SDK.
    std::ostringstream entries;
    entries.imbue(std::locale::classic());
    entries << std::setprecision(17);
    for (size_t i = 0; i < gains.size(); ++i) {
        if (i) entries << ',';
        entries << gains[i];
    }
    Spinnaker::CCMSettings settings;
    // The calibration already maps source RGB to the target: no extra gamma.
    settings.ColorSpace = Spinnaker::SPINNAKER_CCM_COLOR_SPACE_OFF;
    settings.CustomCCMCode = Spinnaker::ImageUtilityCCM::EncryptColorCorrectionMatrix(entries.str());
    return settings;
}
inline void correctRgbSpinnaker(cv::Mat & rgb, const Spinnaker::CCMSettings & settings) {
    if (rgb.type() != CV_8UC3 || rgb.empty())
        throw std::invalid_argument("Spinnaker CCM requires a nonempty RGB8 image");
    cv::Mat source = rgb.isContinuous() ? rgb : rgb.clone();
    auto image = Spinnaker::Image::Create(source.cols, source.rows, 0, 0,
        Spinnaker::PixelFormat_RGB8, source.data);
    auto corrected = Spinnaker::ImageUtilityCCM::CreateColorCorrected(image, settings);
    // Own the result independently of the SDK ImagePtr lifetime and worker queues.
    rgb = cv::Mat(corrected->GetHeight(), corrected->GetWidth(), CV_8UC3,
        corrected->GetData(), corrected->GetStride()).clone();
}
}
