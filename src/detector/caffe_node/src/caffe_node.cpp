/**********************************************************
 * Copyright (c) 2019 Michael.Chen. All rights reserved.
 * File name: main.cpp
 * Created on 24th/Jul/2019
 * Author: Michael.Chen
 * Follow me: https://www.tgeek.tech
 * ********************************************************/
#include "detection.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
	Detector detector;
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/video", 1);

    cv::VideoCapture cap;
    cap.open(0);
    cv::Mat frame;
	sensor_msgs::ImagePtr msg;
	struct timeb tb;
	char process_time[30];
	long fps =0;
    ros::Rate loop_rate(5);
    while (ros::ok()) 
    {
        ftime(&tb);                         // get time
        struct tm *t = localtime(&tb.time); // get time with format
		fps = tb.time * 1000 + tb.millitm;		
        cap>>frame; 
        detector.GetResult(frame);
		detector.DrawResult(frame,detector.Class_names,detector.Boxes,detector.Confidences,detector.Centers);
		ftime(&tb);
        sprintf(process_time, "FPS: %.2f", double(1000 / ((tb.time * 1000 + tb.millitm) - fps)));
        cv::putText(frame, process_time, cv::Point(15, 20), 0, 0.8, cv::Scalar(0, 255, 100), 1);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}