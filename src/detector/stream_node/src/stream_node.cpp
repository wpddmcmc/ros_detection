#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat frame;
    cv::namedWindow("view",false);
    cv::resizeWindow("view",cv::Size(1280,960));
    try
    {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        resize(frame,frame,cv::Size(1280,960));
        cv::imshow("view", frame);
        char key = cv::waitKey(10);
        if (key == 27)
            exit(0);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    cv::namedWindow("view", 0);
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/video", 1, imageCallback);

    ros::spin();
    cv::destroyWindow("view");
}