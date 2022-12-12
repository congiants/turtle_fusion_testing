//std libraries
#include <memory>
#include <stdio.h>

//External libraries
#include "eigen3/Eigen/Core"
#include "yaml-cpp/yaml.h"

//boost library
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//All functions and non-eigen variables are defined at FusionHandler.hpp
#include "turtle_fusion/FusionHandler.hpp"

using namespace Eigen;

Matrix3f camera_matrix;
Matrix<float, 3, 4> transformation_matrix;

bool processing_flag;

//Constructor used in main. Camera id is 0 left, 1 middle, 2 right.
//Done this way as cameras are unsyncronized. Pseudo-thereading
//Calls def_topics, init_subscribers, init_subsribers functions
//Main file calls manually the constructor 3 times. One time for each camera.
FusionHandler::FusionHandler(int camera_id) : Node("early_fusion_handler" + std::to_string(camera_id))
{
    RCLCPP_INFO(this->get_logger(), "Spinning Node");
    def_topics();
    RCLCPP_INFO(this->get_logger(), "Initialized topics");
    init_subscribers(camera_id);
    init_publishers(camera_id);
    RCLCPP_INFO(this->get_logger(), "Initialized publisers/subscribers, waiting for callbacks");
}

//Deconstructor
FusionHandler::~FusionHandler()
{
}

//Defines topics. Called in FusionHandler constructor. Uses topics t struct defined in FusionHandler.hpp
void FusionHandler::def_topics()
{
    //Gets file name where ini settings are stored.
    std::string filename = ament_index_cpp::get_package_share_directory("turtle_fusion")
                           + "/settings/topic_settings.ini";

    //Reads ini file
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    //Goes into the ini settings and gets what topics the code will be subscribed to and what topics it will publish on
    //Subsribers
    t.pcl_subscriber_topic = pt.get<std::string>("Subscribers.pcl_sub");
    t.jai_left_topic = pt.get<std::string>("Subscribers.jai_left_sub");
    t.jai_center_topic = pt.get<std::string>("Subscribers.jai_center_sub");
    t.jai_right_topic = pt.get<std::string>("Subscribers.jai_right_sub");

    //Publishers
    t.left_pcl_publisher_topic = pt.get<std::string>("Publishers.left_pcl_pub");
    t.center_pcl_publisher_topic = pt.get<std::string>("Publishers.center_pcl_pub");
    t.right_pcl_publisher_topic = pt.get<std::string>("Publishers.right_pcl_pub");
    t.bb_pcl_publisher_topic = pt.get<std::string>("Publishers.bb_pcl_pub");

    publish_bounding_boxes = pt.get<bool>("Settings.publish_bounding_boxes");

}

//Initializes ROS2 subscribers.
//Main file calls manually the constructor, thus also this function 3 times. One time for each camera.
void FusionHandler::init_subscribers(int camera_id)
{
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    
    options.callback_group = lidar_callback_group;

    pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(t.pcl_subscriber_topic, qos, std::bind(&FusionHandler::lidarMsgCallback, this, std::placeholders::_1), options);

    options.callback_group = camera_callback_group;

    //Subscribes to bounding boxes of either left, middle or right camera topics. Left = 0, Mid = 1, Right = 2.
    switch(camera_id){
        case 0 :
            jai_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_left_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1), options);
            break;
        case 1 :
            jai_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_center_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1), options);
            break;
        case 2 :
            jai_subscriber = this->create_subscription<turtle_interfaces::msg::BoundingBoxes>(t.jai_right_topic, qos, std::bind(&FusionHandler::cameraCallback,this,std::placeholders::_1), options);
            break;
        default : 
            RCLCPP_ERROR(this->get_logger(), "Non valid camera ID, Shuting Down...");
            break;
    }
    
}

//Initializes ROS2 publishers.
//Main file calls manually the constructor, thus also this function 3 times. One time for each camera.
void FusionHandler::init_publishers(int camera_id)
{
    rclcpp::SensorDataQoS Qos;

     //Publishes cone distances of either left, middle or right camera. Left = 0, Mid = 1, Right = 2.
    switch(camera_id){
        case 0:
            pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.left_pcl_publisher_topic, Qos);
            break;
        case 1:
            pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.center_pcl_publisher_topic, Qos);
            break;
        case 2:
            pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.right_pcl_publisher_topic, Qos);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Non valid camera ID, Shuting Down...");
    }
    bb_pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(t.bb_pcl_publisher_topic,Qos);

    //Sets up message that will be published 

    //Sets up message id (Bounding box pcl msg is for debbuging)
    coneDistancesMsg.header.frame_id = "os1";
    boundingBoxPclMsg.header.frame_id = "os1";

    //Sets up message height (Bounding box pcl msg is for debbuging)
    coneDistancesMsg.height = 1;
    boundingBoxPclMsg.height = 1;

    //Sets up if message will be big endian. 
    //(A big-endian system stores the most significant byte of a word at the smallest memory address and the least significant byte at the largest.)
    coneDistancesMsg.is_bigendian = false;
    boundingBoxPclMsg.is_bigendian = false;

    //Not sure exactly
    coneDistancesMsg.point_step = 12;
    boundingBoxPclMsg.point_step = 12;
    coneDistancesMsg.point_step = 16;

    coneDistancesMsg.fields.resize(4); //x, y, z, rgb
     
    // coneDistancesMsg.fields.resize(3);
    boundingBoxPclMsg.fields.resize(3);

    //Determines where on the message x,y,z,color will be 
    coneDistancesMsg.fields[0].name = "x";
    coneDistancesMsg.fields[0].offset = 0;
    coneDistancesMsg.fields[0].datatype = 7;
    coneDistancesMsg.fields[0].count = 1;

    coneDistancesMsg.fields[1].name = "y";
    coneDistancesMsg.fields[1].offset = 4;
    coneDistancesMsg.fields[1].datatype = 7;
    coneDistancesMsg.fields[1].count = 1;

    coneDistancesMsg.fields[2].name = "z";
    coneDistancesMsg.fields[2].offset = 8;
    coneDistancesMsg.fields[2].datatype = 7;
    coneDistancesMsg.fields[2].count = 1;

    coneDistancesMsg.fields[3].name = "rgb";
    coneDistancesMsg.fields[3].offset = 12;
    coneDistancesMsg.fields[3].datatype = 7;
    coneDistancesMsg.fields[3].count = 1;

/////////////////////////////DEBUGGING (I think)/////////////////////////////////////
    boundingBoxPclMsg.fields[0].name = "x";
    boundingBoxPclMsg.fields[0].offset = 0;
    boundingBoxPclMsg.fields[0].datatype = 7;
    boundingBoxPclMsg.fields[0].count = 1;

    boundingBoxPclMsg.fields[1].name = "y";
    boundingBoxPclMsg.fields[1].offset = 4;
    boundingBoxPclMsg.fields[1].datatype = 7;
    boundingBoxPclMsg.fields[1].count = 1;

    boundingBoxPclMsg.fields[2].name = "z";
    boundingBoxPclMsg.fields[2].offset = 8;
    boundingBoxPclMsg.fields[2].datatype = 7;
    boundingBoxPclMsg.fields[2].count = 1;
////////////////////////////////////////////////////////////////////////////    
}

//What to do when it gets a pointcloud from the lidar
void FusionHandler::lidarMsgCallback(sensor_msgs::msg::PointCloud2 pcl_msg)
{
    //Saves last pcl message and waits for the camera to send bboxes
    fusion_mutex.lock();
    this->latest_pcl = pcl_msg;
    fusion_mutex.unlock();

    // std::cout << "Save latest PointCloud message"<<std::endl;

}

//What to do when camera sends bboxes
//fusion & publishCones functions are called here
void FusionHandler::cameraCallback(const turtle_interfaces::msg::BoundingBoxes cam_msg)
{
    // std::cout<<"Inside Camera Callback"<<std::endl;
    // std::cout<<"Camera identifier : "<<(int)cam_msg.camera<<std::endl;

    //If latest_pcl from lidarMsgCallback isn't empty processing_flag is set to true
    this->lidar_flag = !(latest_pcl.data.size() == 0);
    processing_flag = lidar_flag && !(cam_msg.x.size() == 0);

    //If processing flag is true then
    if(processing_flag){
        fusion_mutex.lock_shared();

        sensor_msgs::msg::PointCloud2 fusion_pcl = this->latest_pcl;
        
        //Fusion algorithms are executed here with inputs the lidar pcl and the camera bboxes
        fusion(fusion_pcl, cam_msg);
        //Results are published to slam
        publishCones();

        //If results are published succesfully then unlock
        if(publish_bounding_boxes){publishBBPcl();}
        
        fusion_mutex.unlock_shared();
    }

}

//Actual fusion algorithms from the fusion.hpp file
//Goal is to calibrate lidar pointcloud with camera bboxes. Once that is done find the 25th percentile of the average distances of the lidar laser points inside the bboxes
//Called in cameraCallback function
void Fusion::fusion(sensor_msgs::msg::PointCloud2 pcl_msg , turtle_interfaces::msg::BoundingBoxes cam_msg)
{

    int camera_id = (int)cam_msg.camera;
    set_lidar_XYZ(pcl_msg);
    read_intrinsic_params(camera_id);
    calculate_transformation_matrix(camera_id);
    calculate_pixel_points();
    find_inside_bounding_boxes(cam_msg);
}

//Publishes cone
//Called in cameraCallback function
void FusionHandler::publishCones()
{
    
    Matrix4Xf pcl_msg = get_pcl_xyz();
    // Matrix3Xf pcl_msg ;

    pcl_msg = get_pcl_xyz();

    coneDistancesMsg.width = pcl_msg.cols();
    coneDistancesMsg.row_step = coneDistancesMsg.width * coneDistancesMsg.point_step;

    coneDistancesMsg.data.resize(coneDistancesMsg.row_step);

    
    uint8_t* ptr = coneDistancesMsg.data.data();
    coneDistancesMsg.is_dense = false;
   
    for (int i = 0; i < pcl_msg.cols(); i++){

        *((float*)(ptr + i*coneDistancesMsg.point_step)) = pcl_msg(0,i);

        *((float*)(ptr + i*coneDistancesMsg.point_step + 4)) = pcl_msg(1,i);

        *((float*)(ptr + i*coneDistancesMsg.point_step + 8)) = pcl_msg(2,i);

        *((float*)(ptr + i*coneDistancesMsg.point_step + 12)) = pcl_msg(3,i);
    }
    
    pcl_publisher->publish(coneDistancesMsg);
    // std::cout<<"MESSAGE PUBLISHED!!!!"<<std::endl;
}

//Debbugging I think
void FusionHandler::publishBBPcl(){

    Matrix3Xf bb_pcl_msg;


    bb_pcl_msg = get_bb_pcl();


    boundingBoxPclMsg.width = bb_pcl_msg.cols();
    boundingBoxPclMsg.row_step = boundingBoxPclMsg.width * boundingBoxPclMsg.point_step;

    boundingBoxPclMsg.data.resize(boundingBoxPclMsg.row_step);

    
    uint8_t* ptr = boundingBoxPclMsg.data.data();
    // boundingBoxPclMsg.is_dense = false;
   
    for (int i = 0; i < bb_pcl_msg.cols(); i++){

        *((float*)(ptr + i*boundingBoxPclMsg.point_step)) = bb_pcl_msg(0,i);

        *((float*)(ptr + i*boundingBoxPclMsg.point_step + 4)) = bb_pcl_msg(1,i);

        *((float*)(ptr + i*boundingBoxPclMsg.point_step + 8)) = bb_pcl_msg(2,i);

        // *((float*)(ptr + i*coneDistancesMsg.point_step + 12)) = pcl_msg(3,i);
    }
    
    bb_pcl_publisher->publish(boundingBoxPclMsg);
    // std::cout<<"MESSAGE PUBLISHED!!!!"<<std::endl;

}