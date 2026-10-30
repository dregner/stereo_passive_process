#include <rclcpp/rclcpp.hpp>
#include <exception>
#include "passive_stereo_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<passive_stereo_capture::PassiveStereoNode>(
            rclcpp::NodeOptions{});
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("passive_stereo_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
