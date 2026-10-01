#include <rclcpp/rclcpp.hpp>
#include "passive_stereo_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<passive_stereo_capture::PassiveStereoNode>(
        rclcpp::NodeOptions{});

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
