/*******************************************************************************************************************
@Copyright 2019 Inspur Co., Ltd
@Filename:  ImageConsumer.cpp
@Author:    Michael.Chen
@Version:   1.0
@Date:      15th/Jul/2018
*******************************************************************************************************************/
#include "DetectorNode.hpp"

#define VIDEO_WIDTH  640
#define VIDEO_HEIGHT 480
#define BUFFER_SIZE 3

// #define USE_CAMERA	// If wanna to use camera, uncommit this line plz!!!!!

volatile unsigned int prdIdx = 0;		// index of reading image
volatile unsigned int csmIdx = 0;		// index of processing image

struct ImageData {
	cv::Mat img;             // data come from camera
	unsigned int frame;  	// index of img
};

ImageData capturedata[BUFFER_SIZE];   // Buffer of capture

/************************************************* 
    Function:      	ImageReader
    Description:    Image read
    Input:          video file or camare image 
    Output:         one frame of reading iamge
    Return:         void
    Others:         none
    *************************************************/
void DetectorNode::ImageReader() 
{ 									
	cv::VideoCapture cap;
	if(!use_camera)
	{
		// use video file
		// read video file
		std::string video_name = ros::package::getPath("detector") + "/video/" + video_file;

		cap.open(video_name); // try to open video file
		if (!cap.isOpened())
		{
			std::cout << "ERROR: Cannot find file \"" << video_name << "\"" << std::endl;
			return;
		}
		else
		{
			std::cout << "INFO: File \"" << video_name << "\" load successfully..." << std::endl;
		}
	}

	else
	{
			// use camera
		  cap.open(camera_index); 					// watch out camera index!!!
	}

	while (true) {
		while(prdIdx - csmIdx >= BUFFER_SIZE);				// wait for image producing
		cap >> capturedata[prdIdx % BUFFER_SIZE].img;		// image reading
		capturedata[prdIdx % BUFFER_SIZE].frame++;

		++prdIdx;											// procecc next frame
	}
}

/************************************************* 
    Function:       ImageProcesser 
    Description:    Image process
    Input:          one frame of reading iamge
    Output:         frame after processing display
    Return:         void
    Others:         none
    *************************************************/
void DetectorNode::ImageProcesser() {
	cv::Mat frame,output;								// source image
	DnnDetector dnndetector;
	cv::dnn::Net net = dnndetector.net;

	cv::namedWindow("Detection",false);
	cv::resizeWindow("Detection",cv::Size(window_width,window_height));

	// for processing speed calculating
	struct timeb tb;
	char process_time[30];
	char timenow[20];
	long fps =0;
    while(true){
		if(show_fps||show_time){			// if fps display or time display
			ftime(&tb);										// get time 
			struct tm *t = localtime(&tb.time);				// get time with format
			fps = tb.time*1000+tb.millitm;

			sprintf(timenow, "%d-%02d-%02d-%02d:%02d:%02d",
				t->tm_year + 1900,
				t->tm_mon + 1,
				t->tm_mday,
				t->tm_hour,
				t->tm_min,
				t->tm_sec);
			ros_info_.time = timenow;
		}
		
		while (prdIdx - csmIdx == 0);						// wait for image producing
		capturedata[csmIdx % BUFFER_SIZE].img.copyTo(frame);
		++csmIdx;											// load next frame
		dnndetector.thePredictor(frame, net);
		frame.copyTo(output);
		dnndetector.drawResult(output,dnndetector.out_names,dnndetector.out_boxes,dnndetector.out_confidences,dnndetector.out_centers,true);

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

		if(show_time){								// display time on image
			cv::putText(output,timenow,cv::Point(output.cols-250,20),0 , 0.6, cv::Scalar(255, 0, 128), 1);
		}

		if(show_fps){								// display fps on imagerows
			ftime(&tb);
			sprintf(process_time,"FPS: %.2f",double(1000/((tb.time*1000+tb.millitm)-fps)));
			cv::putText(output,process_time,cv::Point(15,20),0 , 0.8, cv::Scalar(0, 0, 0), 1);
		}
		
		try {
			cv::imshow("Detection", output);	
			if(debug_mode<1){						// debug mode, wait for keyboard to continu
				char key = cv::waitKey(30);
				if(key == 27)								// if 'esc' pressed, programe close 
				{
					exit(0);
				}	
			}

			else{											// undebug mode, continu processing
				char key = cv::waitKey(0);
				if(key == 27)								// if 'esc' pressed, programe close 
				{
					exit(0);
				}	
			}

		}
		catch (cv::Exception e) {
			std::cout<<"Capture Error"<<std::endl;
		}			
    }
}

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "detector");

    DetectorNode image_consumer;
    
	std::thread task0(&DetectorNode::ImageReader, image_consumer);  	// add image reading thread
	std::thread task1(&DetectorNode::ImageProcesser, image_consumer);  // add image processing thread

	// thread joined
	task0.join();
	task1.join();
	
	ros::waitForShutdown();
	
	return EXIT_SUCCESS;
}