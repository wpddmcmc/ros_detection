/*********************************************************************************
 * Copyright (c) 2019 Michael.Chen. All rights reserved.
 * File name: detect.hpp
 * Created on 19th/Jul/2019
 * Author: Michael.Chen
 * Github: https://github.com/wpddmcmc/dnnDetector
 * Follow me: https://www.tgeek.tech
 * ******************************************************************************/
#pragma once
#include "DetectorNode.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

/************************************************* 
    Class:       Detector
    Description:    class for prediction
    *************************************************/
class DnnDetector
{
public:
    DnnDetector();
    cv::dnn::Net net;

    // output results
    cv::Mat outputimage;
    std::vector<cv::Rect> out_boxes;
    std::vector<cv::Point> out_centers;
    std::vector<std::string> out_names;
    std::vector<float> out_confidences;

    // predictation
    void thePredictor(cv::Mat frame, cv::dnn::Net net);  
    // draw prediction results   
    void drawResult(cv::Mat& frame, std::vector<std::string> out_names, std::vector<cv::Rect> out_boxes,std::vector<float> confidences,std::vector<cv::Point> out_centers,bool if_fps);   

private:
    // network structure
    int net_type;                   
    std::string net_structure;
    std::string model;
    std::string name_file;

    // network configrations
    double thresh;          // threshold for confidence
    double nms_thresh;      // threshold for nms
    size_t width;
    size_t height;
    float meanVal;
    float scaleFactor;
    std::vector<std::string> classes;

    double fps;
    // get the names of output layers
    std::vector<std::string> getOutputsNames(cv::dnn::Net &net);      
    // draw prediction results (Not Used)
    void drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame,std::vector<std::string> classes);
    // yolo detection
    void yoloProcess(cv::Mat frame, const std::vector<cv::Mat> &outs, float confThreshold, float nmsThreshold);
    // ssd detection
    void ssdPropcess(cv::Mat frame, std::vector<cv::Mat> outs);
};
