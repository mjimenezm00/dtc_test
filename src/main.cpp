#include <dtc_test/dynamic_task_planner.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create ROS node
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.start_parameter_services(false);

    auto node = std::make_shared<dtc_test::DynamicTaskPlanner>("dtc_test", node_options);

    // Create an executor to spin the node
    std::thread t(
        [node]
        {
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(node);
            executor.spin();
        });

    node->run_test();

    // Shutdown
    t.join();
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
