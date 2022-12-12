//Main function
//Gets the ROS2 nodes for left, mid and right camera spinning 
//and calls constructor declared in FusionHandler.hpp and defined in early_fusion.cpp

#include "early_fusion.cpp"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" 

/**
 * TODO BEFORE PRODUCTION : 
 *  1) DELETE EVERY LOGGER 
 *  2) DELETE EVERY STD::COUT
 */

int main(int argc, char** argv)
{

    //Standard ros2 proccedure. Creates an excecutor
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    //Creates 3 nodes for the 3 cameras using the constructor from FusionHandler.hpp and early_fusion.cpp
    rclcpp::Node::SharedPtr node_left = std::make_shared<FusionHandler>(0);
    rclcpp::Node::SharedPtr node_center = std::make_shared<FusionHandler>(1);
    rclcpp::Node::SharedPtr node_right = std::make_shared<FusionHandler>(2);

    //Adds nodes to the executor
    executor.add_node(node_left);
    executor.add_node(node_center);
    executor.add_node(node_right);

    //Spins executor
    executor.spin();
    // rclcpp::spin(std::make_shared<FusionHandler>());

    //Stops ros2 when program is finished
    rclcpp::shutdown();
    return 0;
}