#include <rclcpp/rclcpp.hpp>
#include "wiln.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto wiln_node = std::make_shared<WilnNode>();
    executor.add_node(wiln_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
