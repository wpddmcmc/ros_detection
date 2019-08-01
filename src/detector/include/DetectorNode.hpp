/*******************************************************************************************************************
@Copyright 2019 Inspur Co., Ltd
@Filename:  ImageConsumer.hpp
@Author:    Michael.Chen
@Version:   1.0
@Date:      15th/Jul/2018
*******************************************************************************************************************/
#pragma once
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "dnndetector.hpp"
#include <time.h>
#include <sys/timeb.h>
#include <thread> 
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include "detector/Detector_Info.h"

class DetectorNode{
public:
    DetectorNode()
    {
        std::string config_file_name = ros::package::getPath("detector") + "/param/param_config.xml"; // load config file
        cv::FileStorage fs(config_file_name, cv::FileStorage::READ);                                  // initialization config
        if (!fs.isOpened())                                                                           //open xml config file
        {
            std::cout << "Could not open the configuration file: param_config.xml " << std::endl;
            exit(-1);
        }

        fs["debug_mode"] >> debug_mode;
        fs["video_file"] >> video_file;
        fs["show_fps"] >> show_fps;
        fs["show_time"] >> show_time;
        fs["use_camera"] >> use_camera;
        fs["camera_index"] >> camera_index;
        fs["window_width"] >> window_width;
        fs["window_height"] >> window_height;

        ros_nh_ = ros::NodeHandle();
        detect_pub_ = ros_nh_.advertise<detector::Detector_Info>("vision_data", 1000);
    }
    void ImageReader();
    void ImageProcesser();

    private:
        int debug_mode;
        std::string video_file;
        int show_fps;
        int show_time;
        int use_camera;
        int camera_index;
        int window_width;
        int window_height;

        ros::NodeHandle ros_nh_;
        ros::Publisher detect_pub_;
        detector::Detector_Info ros_info_;
};