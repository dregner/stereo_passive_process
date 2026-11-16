#include "raw_snapshot.hpp"

#include <csetjmp>
#include <cstdio>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>

#include <jpeglib.h>
#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{
namespace
{
namespace fs = std::filesystem;

struct JpegError
{
    jpeg_error_mgr manager{};
    std::jmp_buf jump;
    char message[JMSG_LENGTH_MAX]{};
};

void onJpegError(j_common_ptr codec)
{
    auto * error = reinterpret_cast<JpegError *>(codec->err);
    error->manager.format_message(codec, error->message);
    std::longjmp(error->jump, 1);
}

// Keep the mutable libjpeg state in the caller so it remains valid after longjmp.
bool encodeJpeg(jpeg_compress_struct & codec, JpegError & error,
                std::FILE * file, const cv::Mat & color, J_COLOR_SPACE input_colors)
{
    if (setjmp(error.jump)) {
        jpeg_destroy_compress(&codec);
        return false;
    }

    jpeg_create_compress(&codec);
    jpeg_stdio_dest(&codec, file);
    codec.image_width = color.cols;
    codec.image_height = color.rows;
    codec.input_components = 3;
    codec.in_color_space = input_colors;
    jpeg_set_defaults(&codec);
    jpeg_set_quality(&codec, 100, TRUE);
    for (int component = 0; component < codec.num_components; ++component) {
        codec.comp_info[component].h_samp_factor = 1;
        codec.comp_info[component].v_samp_factor = 1;
    }
    jpeg_start_compress(&codec, TRUE);
    while (codec.next_scanline < codec.image_height) {
        auto * row = const_cast<JSAMPLE *>(color.ptr<JSAMPLE>(codec.next_scanline));
        jpeg_write_scanlines(&codec, &row, 1);
    }
    jpeg_finish_compress(&codec);
    jpeg_destroy_compress(&codec);
    return true;
}

void writeRawJpeg(const fs::path & path, const cv::Mat & bayer)
{
    if (bayer.empty() || bayer.type() != CV_8UC1)
        throw std::runtime_error("Expected a BayerRG8 frame for " + path.string());

    cv::Mat color;
#ifdef JCS_EXTENSIONS
    cv::cvtColor(bayer, color, cv::COLOR_BayerRGGB2BGR);
    const auto input_colors = JCS_EXT_BGR;
#else
    cv::cvtColor(bayer, color, cv::COLOR_BayerRGGB2RGB);
    const auto input_colors = JCS_RGB;
#endif

    const auto close_file = [](std::FILE * file) { std::fclose(file); };
    std::unique_ptr<std::FILE, decltype(close_file)> file(std::fopen(path.c_str(), "wb"));
    if (!file)
        throw std::runtime_error("Could not open " + path.string());

    JpegError error;
    jpeg_compress_struct codec{};
    codec.err = jpeg_std_error(&error.manager);
    error.manager.error_exit = onJpegError;
    if (!encodeJpeg(codec, error, file.get(), color, input_colors))
        throw std::runtime_error("Could not encode " + path.string() + ": " + error.message);
    if (std::fclose(file.release()) != 0)
        throw std::runtime_error("Could not finish writing " + path.string());
}

fs::path reserveSnapshotDir(const fs::path & parent, const std::string & timestamp)
{
    auto dir = parent / timestamp;
    for (unsigned int suffix = 1; ; ++suffix) {
        std::error_code error;
        if (fs::create_directory(dir, error))
            return dir;
        if (error && error != std::errc::file_exists)
            throw fs::filesystem_error("Could not create snapshot directory", dir, error);
        dir = parent / (timestamp + "_" + std::to_string(suffix));
    }
}
}  // namespace

fs::path saveRawSnapshot(const fs::path & output_dir,
    std::chrono::system_clock::time_point requested_at,
    const cv::Mat & left_bayer, const cv::Mat & right_bayer)
{
    if (output_dir.empty())
        throw std::invalid_argument("raw_image_output_dir must be set");

    const auto time = std::chrono::system_clock::to_time_t(requested_at);
    std::tm local_time{};
    char timestamp[32]{};
    if (!localtime_r(&time, &local_time) ||
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M", &local_time) == 0)
        throw std::runtime_error("Could not format snapshot timestamp");

    const auto parent = fs::absolute(output_dir);
    fs::create_directories(parent);
    const auto dir = reserveSnapshotDir(parent, timestamp);
    try {
        fs::create_directory(dir / "left");
        fs::create_directory(dir / "right");
        writeRawJpeg(dir / "left" / "image.jpeg", left_bayer);
        writeRawJpeg(dir / "right" / "image.jpeg", right_bayer);
    } catch (...) {
        std::error_code cleanup_error;
        fs::remove_all(dir, cleanup_error);
        if (cleanup_error)
            throw fs::filesystem_error("Could not clean up failed snapshot", dir, cleanup_error);
        throw;
    }
    return dir;
}

}  // namespace passive_stereo_capture
