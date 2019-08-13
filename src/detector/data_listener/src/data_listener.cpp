#include "ros/ros.h"
#include "detector/Detector_Info.h"
#include <iostream>
#include <iomanip>

void chatterCallback(const detector::Detector_Info & msg)
{
 
    for(int i=0 ; i<end( msg.names)-begin( msg.names);i++)
    {
        if(i==0)
        {
            std::cout<<"[Detection]"<<std::endl;
        }
        std::cout<<msg.names[i].c_str()<<" "<<std::setprecision(4)<<msg.confidences[i]*100
        <<"% ("<<msg.centers_x[i]<<","<<msg.centers_y[i]<<")"<<std::endl;
    }     
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("detection_data", 1000, chatterCallback);
    ROS_INFO("Listener Start to Monitor");
    ros::spin();

    return 0;
}
