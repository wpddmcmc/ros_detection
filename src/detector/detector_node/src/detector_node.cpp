#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dnndetector.hpp>
#include "detector/Detector_Info.h"

DnnDetector dnndetector;
cv::dnn::Net net = dnndetector.net;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/video", 1);

    ros::NodeHandle ros_nh_;
    ros::Publisher detect_pub_;
    detector::Detector_Info ros_info_;
    ros_nh_ = ros::NodeHandle();
    detect_pub_ = ros_nh_.advertise<detector::Detector_Info>("detection_data", 1000);

    cv::VideoCapture cap;
    cap.open(0);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        cap >> frame;
        dnndetector.thePredictor(frame, net); // do detection
        dnndetector.drawResult(frame, dnndetector.out_names, dnndetector.out_boxes, dnndetector.out_confidences, dnndetector.out_centers, true);
        
        ros_info_.names.clear();
		ros_info_.confidences.clear();
		ros_info_.centers_x.clear();
		ros_info_.centers_y.clear();
		for(int n=0; n<dnndetector.out_names.size(); n++)
		{
			ros_info_.names.push_back(dnndetector.out_names[n]);
			ros_info_.confidences.push_back(dnndetector.out_confidences[n]);
			ros_info_.centers_x.push_back(dnndetector.out_centers[n].x);
			ros_info_.centers_y.push_back(dnndetector.out_centers[n].y);
		}
        detect_pub_.publish(ros_info_);

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

