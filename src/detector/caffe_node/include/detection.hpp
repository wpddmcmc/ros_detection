/*********************************************************************************
 * Copyright (c) 2019 Michael.Chen. All rights reserved.
 * File name: detection.hpp
 * Created on 24th/Jul/2019
 * Author: Michael.Chen
 * Follow me: https://www.tgeek.tech
 * ******************************************************************************/

#include "caffe/caffe.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include <ros/package.h>
#include <time.h>
#include <sys/timeb.h>

/************************************************* 
    Class:       Detector
    Description:    class for prediction
*************************************************/
class Detector {
public:
  /************************************************* 
    Function:       Detector
    Description:    initialization of detector
    Input:          None 
    Output:         None 
  *************************************************/
  Detector(){
    // Read configration file
    cv::FileStorage setting_fs(ros::package::getPath("detector") + "/caffe_node/param/param.xml", cv::FileStorage::READ);
    if(!setting_fs.isOpened()){
      std::cout<<"ERROR: Open config file failed"<<std::endl;
      exit(-1);
    }
    setting_fs["device"] >> device_type;
    setting_fs["thresh"] >> thresh;
    setting_fs["model_file"] >> model_file;
    setting_fs["weights_file"] >> weights_file;
    setting_fs["mean_file"] >> mean_file;
    setting_fs["mean_value"] >> mean_value;
    setting_fs["name_file"] >> name_file;

    // device selection
    if (device_type)
      caffe::Caffe::set_mode(caffe::Caffe::GPU);
    else
      caffe::Caffe::set_mode(caffe::Caffe::CPU);

    model_file =  ros::package::getPath("detector") + model_file;
    weights_file =  ros::package::getPath("detector") + weights_file;
    name_file =  ros::package::getPath("detector") + name_file;
    // Load the network
    net_.reset(new caffe::Net<float>(model_file, caffe::TEST));
    net_->CopyTrainedLayersFrom(weights_file);
    if(net_->num_inputs()!=1) 
      std::cout<<"ERROR: Network should have exactly one input."<<std::endl;
      
    caffe::Blob<float> *input_layer = net_->input_blobs()[0];
    num_channels_ = input_layer->channels();
    if(num_channels_ != 3 && num_channels_ != 1)
      std::cout<< "ERROR: Input layer should have 1 or 3 channels."<<std::endl;

    input_geometry_ = cv::Size(input_layer->width(), input_layer->height());
    // Load the binaryproto mean file. 
    SetMean(mean_file, mean_value);

    // Read label file
    std::ifstream ifs(name_file.c_str());
    if(!ifs.is_open()){
      std::cout<<"ERROR: Cannot find labels file in \""<<name_file<<"\""<<std::endl;
    }
    std::string line;
    while (std::getline(ifs, line)) labels.push_back(line);
  }

  // Get output results
  void GetResult(cv::Mat frame);
  // Draw output to image
  void DrawResult(cv::Mat& frame, std::vector<std::string> Class_names, 
                            std::vector<cv::Rect> Boxes,std::vector<float> Confidences,std::vector<cv::Point> Centers);
public: 
  // public output results
  std::vector<cv::Rect> Boxes;          // bounding boxes of detected objects
  std::vector<std::string> Class_names; // classes of detected objects
  std::vector<float> Confidences;       // confidences of detected objects
  std::vector<cv::Point> Centers;       // center position of detected objects

private:
  // Load mean value
  void SetMean(const std::string &mean_file, const std::string &mean_value);
  // Wrap the input layer of the network in separate cv::Mat objects
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);
  // Convert the input image to the input image format of the network.
  void Preprocess(const cv::Mat& img,std::vector<cv::Mat>* input_channels);
  // Get outputs of output layers of network
  std::vector<std::vector<float> > Detect(const cv::Mat& img);

private:
  caffe::shared_ptr<caffe::Net<float> > net_;    // Caffe network
  cv::Size input_geometry_;               // input size
  int num_channels_;                      // number of input channels
  cv::Mat mean_;                          // mean value

  std::string name_file;                  // label file
  int device_type;                        // device type: cpu/gpu
  std::string model_file;                 // network struture file
  std::string weights_file;               // pretrained model file
  std::string mean_file;                  // binary mean file
  std::string mean_value;                 // mean value
  std::vector<std::string> labels;        // classes
  float thresh;                           // confidence threshold
};

