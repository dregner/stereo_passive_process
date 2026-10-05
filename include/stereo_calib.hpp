    #pragma once

    #include <string>
    #include <opencv2/core.hpp>
    #include <opencv2/calib3d.hpp>
    #include <retinify/geometry.hpp>

    namespace passive_stereo_capture
    {

    /// Loads stereo calibration from an OpenCV FileStorage YAML and pre-computes
    /// rectification maps for both cameras using cv::stereoRectify.
    ///
    /// The YAML file must contain:
    ///   image_width, image_height — native (unrectified) image dimensions
    ///   K1, D1 — left camera 3x3 intrinsic matrix and distortion vector
    ///   K2, D2 — right camera 3x3 intrinsic matrix and distortion vector
    ///   R       — rotation of right w.r.t. left (3x3)
    ///   T       — translation of right w.r.t. left (3x1, metres)
    class StereoCalib
    {
    public:
        StereoCalib() = default;

        /// Load calibration and compute maps.
        /// @throws std::runtime_error if the file is missing or malformed.
        void load(const std::string & calib_yaml_path);

        bool isLoaded() const { return loaded_; }

        // Accessors for downstream consumers (Retinify, SLAM config)
        int width()      const { return img_size_.width;  }
        int height()     const { return img_size_.height; }
        double fx_l()      const { return K1_.at<double>(0, 0); }
        double fy_l()      const { return K1_.at<double>(1, 1); }
        double cx_l()      const { return K1_.at<double>(0, 2); }
        double cy_l()      const { return K1_.at<double>(1, 2); }
        double fx_r()      const { return K2_.at<double>(0, 0); }
        double fy_r()      const { return K2_.at<double>(1, 1); }
        double cx_r()      const { return K2_.at<double>(0, 2); }
        double cy_r()      const { return K2_.at<double>(1, 2); }
        cv::Mat leftDistortions()  const { return D1_; }
        cv::Mat rightDistortions() const { return D2_; }
        retinify::DistortionCoefficients toRetinifyDistortion(const cv::Mat & dist_mat);
        std::array<std::array<double, 3>, 3> rot() const {
            return {{
                {R_.at<double>(0, 0), R_.at<double>(0, 1), R_.at<double>(0, 2)},
                {R_.at<double>(1, 0), R_.at<double>(1, 1), R_.at<double>(1, 2)},
                {R_.at<double>(2, 0), R_.at<double>(2, 1), R_.at<double>(2, 2)}
            }};
        }
        std::array<double, 3> trans() const {
            const double * t = T_.ptr<double>();
            return {t[0], t[1], t[2]};
        }
        double baseline() const {
            const double * t = T_.ptr<double>();
            return std::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
        }
        


    private:
        cv::Mat K1_, D1_, K2_, D2_, R_, T_;  ///< Raw calibration matrices from YAML
        cv::Size img_size_;    
        bool loaded_{false};
    };

    }  // namespace passive_stereo_capture
