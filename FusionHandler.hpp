//std libs
#include <memory>

//Ros2 lib
#include <rclcpp/rclcpp.hpp>

//Messages we use
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "turtle_interfaces/msg/bounding_boxes.hpp"

//Used to protect shared data from being simultaneously accessed by multiple threads
#include <shared_mutex>

//Not sure
#include "ament_index_cpp/get_package_share_directory.hpp"

//Fusion algorithms
#include "turtle_fusion/Fusion.hpp"

//Number of cameras
#ifndef CAMERA_N
#define CAMERA_N 3
#endif

//PI
#ifndef PI
#define PI 3.14159
#endif


class FusionHandler : public rclcpp::Node, private Fusion
{
private:

    //Ros2 topics that are used
    struct Topics{
        std::string pcl_subscriber_topic;
        std::string left_pcl_publisher_topic;
        std::string center_pcl_publisher_topic;
        std::string right_pcl_publisher_topic;
        std::string bb_pcl_publisher_topic;
        std::string jai_left_topic;
        std::string jai_center_topic;
        std::string jai_right_topic;
    };

    Topics t;

    //Create ROS2 variables that will be used in early_fusion.cpp

    rclcpp::SubscriptionOptions options;
    rclcpp::CallbackGroup::SharedPtr camera_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> *pcl_subscriber;

    rclcpp::Subscription<turtle_interfaces::msg::BoundingBoxes>::SharedPtr jai_subscriber;

    //Debugging
    // message_filters::Subscriber<turtle_interfaces::msg::BoundingBoxes> *jai_left_subscriber;
    // message_filters::Subscriber<turtle_interfaces::msg::BoundingBoxes> *jai_center_subscriber;
    // message_filters::Subscriber<turtle_interfaces::msg::BoundingBoxes> *jai_right_subscriber;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;
    sensor_msgs::msg::PointCloud2 coneDistancesMsg; // x y z rgb t 

    bool publish_bounding_boxes;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bb_pcl_publisher;
    sensor_msgs::msg::PointCloud2 boundingBoxPclMsg; // x y z rgb t 

    std::shared_timed_mutex fusion_mutex;
    bool lidar_flag;
    sensor_msgs::msg::PointCloud2 latest_pcl;



public:
    //Constuctor and deconstructor
    FusionHandler(int);
    ~FusionHandler();

    //Ros2 functions to define topics and subsribers
    void def_topics();
    void init_publishers(int);
    void init_subscribers(int);

    //What to do when you get the lidar pointcloud and the camera bounding boxxes
    void lidarMsgCallback(const sensor_msgs::msg::PointCloud2);
    void cameraCallback(const turtle_interfaces::msg::BoundingBoxes);

    //Ros2 publishers
    void publishCones();
    void publishBBPcl();

};