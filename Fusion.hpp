//Actual fusion algorithms.
//General goal is to calibrate the pointcloud of the lidar on to the 2D plane of the cameras and find the points inside the camera bounding boxes.
//Steps:
//1. Get the cartesian coordinates of the lidar pointcloud (Spherical to cartesian coordinates). Pointcloud has already been filtered (points behind and far away removed)
//2. Get the intrinsic parameters of the camera
//3. Create a transformation matrix to transfrom the 3D points on to the camera 2D plane.
//4. Transform the points from the pointcloud
//5. Find points inside bounding boxxes
//6. Use 25th percentile to calculate the distance of the cone center from the car

//std libraries
#include <stdio.h>
#include <iostream>

//eigen library
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/Core"

//Yaml library
#include "yaml-cpp/yaml.h"

//Custom messages used
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "turtle_interfaces/msg/bounding_boxes.hpp"

//Not sure
#include "ament_index_cpp/get_package_share_directory.hpp"

//Boost library
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//Opencv library
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

//Ros2
#include "rclcpp/rclcpp.hpp"


using namespace Eigen;

#define IMAGE_WIDTH 2560
#define IMAGE_HEIGHT 2048

//Class declecation
class Fusion
{
private:
MatrixXf lidar_xyz;                             // LiDAR 3D points

Matrix<float, 3, 4> transformation_matrix;      // 3x4 [R | t] matrix

std::vector<cv::Point2f> px;                    // image frame pixel points 

Matrix3f intrinsic_K;                           // Camera Matrix 
Matrix<float, 1, 5> intrinsic_D;                // Distortion Coefficients    

Matrix4Xf pcl_xyz;                              // Pcl message to be pushed
// Matrix3Xf pcl_xyz;
float cone_color;
Matrix3Xf bb_pcl;

//Decleration of functions we will use
public:
    void set_lidar_XYZ(sensor_msgs::msg::PointCloud2);
    void read_intrinsic_params(int);
    void calculate_transformation_matrix(int);

    void calculate_pixel_points();
    void find_inside_bounding_boxes(turtle_interfaces::msg::BoundingBoxes);
    void extract_distance(std::vector<float>, std::vector<float>, std::vector<float>, int);
    
    void assign_bb_pcl(std::vector<float>);

    void fusion(sensor_msgs::msg::PointCloud2, turtle_interfaces::msg::BoundingBoxes);

    MatrixXf get_lidar_xyz(){return lidar_xyz;}
    std::vector<cv::Point2f> get_px(){return px;}
    Matrix4Xf get_pcl_xyz(){return pcl_xyz;}
    // Matrix3Xf get_pcl_xyz(){return pcl_xyz;}
    Matrix3Xf get_bb_pcl(){return bb_pcl;}
};

//Gets base filtered pcl message from lidar and extract xyz coordinates of the points. XYZ from points are saved in eigen::lidar_xyz matrix.
//lidar_xyz matrix structure:
//X1 X2 ... Xn
//Y1 Y2 ... Yn
//Z1 Z2 ... Zn
//1  1 ...  1
void Fusion::set_lidar_XYZ(sensor_msgs::msg::PointCloud2 pcl_msg)
{

    uint8_t* ptr = pcl_msg.data.data();
    int pcl_size = pcl_msg.data.size()/pcl_msg.point_step; //Number of pointcloud points

    // std::cout<<"PCL SIZE = "<<pcl_size<<std::endl<<std::endl;intrinisic_D
    // resize 4xN (x(i) y(i) z(i) 1)
    
    lidar_xyz.resize(4,pcl_size); // 4 x number of pointcloud points

    //Fills up lidar_xyz with the pointcloud points
    for(int i=0; i<pcl_size; i++){
        lidar_xyz(0,i) = *((float*)(ptr + i*pcl_msg.point_step));       // X
        lidar_xyz(1,i) = *((float*)(ptr + i*pcl_msg.point_step + 4));   // Y
        lidar_xyz(2,i) = *((float*)(ptr + i*pcl_msg.point_step + 8));   // Z
        lidar_xyz(3,i) = 1;                                             // Homogenous 1 
    }
}

//Reads intrinsic parameters of the camera
void Fusion::read_intrinsic_params(int camera_id)
{
    //Location of the intrinsic parameters of the camera 
    std::string package_share_path = ament_index_cpp::get_package_share_directory("turtle_calibration");
    std::string intrinsic_filename = package_share_path + "/settings/intrinsic_params" + std::to_string(camera_id) + ".yaml";

    YAML::Node yaml_root = YAML::LoadFile(intrinsic_filename);

    //Fills up intrinsic_K and intrinsic_D with camera intriniscs and distortion coefficients
    for(YAML::const_iterator it = yaml_root.begin(); it!=yaml_root.end(); it++){
        
        const std::string &key = it->first.as<std::string>();

        YAML::Node attributes = it->second;
        int rows = attributes["rows"].as<int>();
        int cols = attributes["cols"].as<int>();

        const std::vector<float> data = attributes["data"].as<std::vector<float>>();

        for(int i=0; i<rows; i++){
            for(int j=0; j<cols; j++){
                if(key =="K"){
                    intrinsic_K(i,j) = data[i*cols + j];
                } else if(key == "D"){
                    intrinsic_D(j) = data[j];
                }else{
                    std::cout<<"Invalid Yaml Key"<<std::endl;
                    break;
                }
            }
        }

    }
}

//Creates a transformation matrix
void Fusion::calculate_transformation_matrix(int camera_id){
    float roll, pitch, yaw;
    float t_x, t_y, t_z;

    //Filename where the results of the lidar_camera calibration are stored
    std::string filename = ament_index_cpp::get_package_share_directory("turtle_calibration") + 
                           "/settings/lidar_camera_extrinsic.ini";

    Matrix3f rotation_matrix; // 3 x N rotation matrix
    Vector3f translation_vector; //3 x N translation vector

    //Uses boost library to get the calibration results
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    //Creates a string that is used by boost to get the calibration results
    //Will be used for rotation matrix
    std::string label_roll = "Camera"+std::to_string(camera_id)+".roll";
    std::string label_pitch = "Camera"+std::to_string(camera_id)+".pitch";
    std::string label_yaw = "Camera"+std::to_string(camera_id)+".yaw";

    //Will be used for translation vector
    std::string label_t_x = "Camera"+std::to_string(camera_id)+".t_x";
    std::string label_t_y = "Camera"+std::to_string(camera_id)+".t_y";
    std::string label_t_z = "Camera"+std::to_string(camera_id)+".t_z";
    
    //Fills up roll, pitch, yaw varaibles to create the rotation matrix
    roll = pt.get<float>(label_roll);
    pitch = pt.get<float>(label_pitch);
    yaw = pt.get<float>(label_yaw);

    //Fills up t_x, t_y, t_z for the translation vector
    t_x = pt.get<float>(label_t_x);
    t_y = pt.get<float>(label_t_y);
    t_z = pt.get<float>(label_t_z);

    //Fills up actual translation_vector matrix
    translation_vector << t_x, 
                          t_y, 
                          t_z;

    //Fills up actual rotation_matrix matrix
    rotation_matrix = AngleAxisf(roll, Vector3f::UnitX())
                      *AngleAxisf(pitch, Vector3f::UnitY())
                      *AngleAxisf(yaw, Vector3f::UnitZ());

    //Fills up actual transformation matrix
    transformation_matrix.resize(3,4); //Why 3 x 4 and not 4 x 4?
    transformation_matrix << rotation_matrix, translation_vector;

    /*
    Manual matrixes used for debugging
    
    switch(camera_id){
        case 0: 
            transformation_matrix << 0.574656, -0.810698, -0.111978, 0.532918,
                            -0.254605, -0.0470594, -0.965899, 0.2155,
                            0.777784, 0.58357, -0.233451, 1.05934;
            break;
        case 1:
            transformation_matrix << 0.00513168, -0.999857, 0.016079, -0.0121403,
                            -0.242234, -0.0168433, -0.970071, -0.0280563,
                            0.970205, 0.00108329, -0.242286, 1.01903; 
            break;
        case 2: 
            transformation_matrix << -0.551352, -0.826806, 0.111365, -0.605246,
                            -0.183028, -0.010359, -0.983053, 0.193978,
                            0.813948, -0.562392, -0.145618, 0.766961;

     }*/
    
}

//Finds projection of 3D LiDAR points on to 2D camera image frame
//We have the transformation matrix needed, the intrinsic camera matrix and the LiDAR points ready.
//So we do:
//1. Multiply LiDAR points with the transformation matrix and the intrinsic camera matrix. Result is a 3 x LiDAR points matrix
//2. In each column divide the first two lines with the third line. Now two first lines of each column contain x, y of each 3D point in the camera frame. (x,y is o,o at the top left corner of the image)
//3. Get the first two lines inside a std::vector that contains cv:2d vectors   
void Fusion::calculate_pixel_points()
{

    MatrixXf pixel_homeogenous_points; //Auxiliary matrix used for required math before px matrix (2 x LiDAR points) is filled up with the actual projection of the 3D points on to 2D image 
    pixel_homeogenous_points.resize(3, lidar_xyz.cols()); //3 x LiDAR points matrix
    pixel_homeogenous_points = intrinsic_K * transformation_matrix * lidar_xyz; //Auxiliary matrix is filled up with matrix multiplication result. 3D to 2D projection is almost ready

    //Math done here to get the first two lines of each column containing x, y of each point on to the 2D image frame.
    px.resize(pixel_homeogenous_points.cols());
    for(int i=0; i<pixel_homeogenous_points.cols(); i++){
        pixel_homeogenous_points(0,i) = pixel_homeogenous_points(0,i)/pixel_homeogenous_points(2,i);
        pixel_homeogenous_points(1,i) = pixel_homeogenous_points(1,i)/pixel_homeogenous_points(2,i);
        
        //Fills up std::vector px containing cv::vector with x, y of each lidar point on the 2D image plane. (x,y are 0,0 at the top left corner)
        px[i].x = pixel_homeogenous_points(0,i);
        px[i].y = pixel_homeogenous_points(1,i);

    }
}

//Finds points inside the bounding boxxes
//Calls extract_distance, assign_bb_pcl functions

//Personal note:
//Here the packages gets slow. Maybe split 2D image and depending where bounding box is go through half the LiDAR points?
void Fusion::find_inside_bounding_boxes(turtle_interfaces::msg::BoundingBoxes cam_msg)
{
    std::vector<float> x_buf, y_buf, z_buf, indexes;
    int counter = 0;
    pcl_xyz.resize(4,cam_msg.x.size());
    // pcl_xyz.resize(3,cam_msg.x.size());

    // std::cout<<"FOUND "<<cam_msg.x.size()<< " BOUNDING BOXES "<<std::endl;

    //It goes through all the bounding boxxes
    for(int i=0; i<cam_msg.x.size(); i++){

        //Sets cone color from the camera message
        cone_color = cam_msg.color[i];
        //Using cv we declare a rectanle whose dimensions are determined by the camera bounding box i
        cv::Rect2f bounding_box(cam_msg.x[i] * IMAGE_WIDTH, cam_msg.y[i] * IMAGE_HEIGHT, cam_msg.w[i] * IMAGE_WIDTH, cam_msg.h[i] * IMAGE_HEIGHT);

        //We go through all the 2D lidar points and using the .inside function from cv we get which points are inside the rectangle (bounding box i)
        for(int j=0; j<px.size(); j++){
            //If 2D lidar point is inside the bounding box, get its equevelant 3D coordinates and push them back inside the std::vectors x_buff,y_buff,z_buff. Push back the index of the point
            if(px[j].inside(bounding_box)){
                x_buf.push_back(lidar_xyz(0,j));    //x(j)
                y_buf.push_back(lidar_xyz(1,j));    //y(j)
                z_buf.push_back(lidar_xyz(2,j));    //z(j) 

                counter++;
                indexes.push_back(j);
                // bb_pcl.resize(3,counter);
                // bb_pcl(0,counter) = lidar_xyz(0,j);
                // bb_pcl(1,counter) = lidar_xyz(1,j);
                // bb_pcl(2,counter) = lidar_xyz(2,j);
            }


        }
        if(x_buf.size() == 0){
            // std::cout<<"NO POINTS FOUND FOR BOUNDING BOX "<<i<<std::endl;   
            continue;
        }
        extract_distance(x_buf, y_buf, z_buf, i);

        x_buf.clear();
        y_buf.clear();
        z_buf.clear();
    }
    assign_bb_pcl(indexes);
    indexes.clear();
}

//Fills up bb_pcl with the points inside the bounding boxes using the index
void Fusion::assign_bb_pcl(std::vector<float> id){
    bb_pcl.resize(3,id.size());
    for (int i=0; i<id.size(); i++){
        bb_pcl(0,i) = lidar_xyz(0,id[i]);
        bb_pcl(1,i) = lidar_xyz(1,id[i]);
        bb_pcl(2,i) = lidar_xyz(2,id[i]);
    }
}

//Finds distance of the points inside the bounding boxxes
//Using the 25th percentile of the average distance
void Fusion::extract_distance(std::vector<float> v_x, std::vector<float> v_y, std::vector<float> v_z, int bounding_box_id)
{
    float mean_x = 0;
    float mean_y = 0; 
    float mean_z = 0;

    //Fills up mean_x, mean_y, mean_z with x,y,z of each point
    for(int i=0; i<v_x.size(); i++){
        mean_x = mean_x + v_x[i];
        mean_y = mean_y + v_y[i];
        mean_z = mean_z + v_z[i];
    }
    //And then finds the mean of x,y,z
    mean_x = mean_x / v_x.size();
    mean_y = mean_y / v_y.size();
    mean_z = mean_z / v_z.size();

    //Debugging
    // pcl_xyz(0,bounding_box_id) = mean_x;
    // pcl_xyz(1,bounding_box_id) = mean_y;
    // pcl_xyz(2,bounding_box_id) = mean_z;

    //Fills up the 3rd line of eigen::pcl_xyz
    pcl_xyz(3,bounding_box_id) = cone_color;

    //Finds indext the point closest to the LiDAR
    float r = 10000;
    int flag = 0;
    for(int i=0; i<v_x.size(); i++){
        float tmp = sqrt(pow(v_x[i],2) + pow(v_y[i],2) + pow(v_z[i],2));
        if(tmp<r){
            r = tmp;
            flag = i;
        }
    }

    //Debugging
    // pcl_xyz(0,bounding_box_id) = v_x[flag];
    // pcl_xyz(1,bounding_box_id) = v_y[flag];
    // pcl_xyz(2,bounding_box_id) = v_z[flag];

    //Replaces first 3 lines of pcl_x, pcl_y, pcl_z with the 25th percentile of the average distance 
    //using the index of the point closest to the lidar
    pcl_xyz(0,bounding_box_id) = (mean_x + v_x[flag]) / 2;
    pcl_xyz(1,bounding_box_id) = (mean_y + v_y[flag]) / 2;
    pcl_xyz(2,bounding_box_id) = (mean_z + v_z[flag]) / 2;

}