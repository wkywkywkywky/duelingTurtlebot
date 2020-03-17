//coded by Kunyu Wang and Cora Coleman for UCSD CSE 276B final project
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <depth_image_proc/depth_traits.h>
#include <sound_play/sound_play.h>
#include <ctime>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

using namespace std;

//finding UI image and refresh UI window every second.
 void UICallBack(const std_msgs::String::ConstPtr& filename){
    cv::Mat image;
    image = cv::imread(filename->data, cv::IMREAD_COLOR);
    if(!image.data){
        std::cout << "Could not open or find the image" << std::endl;
    }
    else{
    	std::cout << "found image" << std::endl;
        cv::imshow("window", image);
        cv::waitKey(1);
    }
 }

int main(int argc, char **argv){
    ros::init(argc, argv, "ui");
    cv::namedWindow("window", cv::WINDOW_NORMAL);
    cv::setWindowProperty("window", 0, 1);
    ros::NodeHandle nh;
    ros::Subscriber UIsub = nh.subscribe("/duelingturtlebotUI", 1000, UICallBack);
    ros::spin();
}
