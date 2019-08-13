/********************************************************************************
 * Copyright (c) 2019 Michael.Chen. All rights reserved.
 * File name: main.cpp
 * Created on 19th/Jul/2019
 * Author: Michael.Chen
 * Github: https://github.com/wpddmcmc/dnnDetector
 * Follow me: https://www.tgeek.tech
 * ******************************************************************************/

#include "dnndetector.hpp"

/************************************************* 
    Function:       Detector
    Description:    initialization of detector
    Input:          None 
    Output:         None 
    *************************************************/
DnnDetector::DnnDetector()
{
    // Read configration file
    cv::FileStorage setting_fs(ros::package::getPath("detector") + "/detector_node/param/dnn_param.xml", cv::FileStorage::READ);
    setting_fs["net_type"] >> net_type;
    setting_fs["thresh"] >> thresh;
    setting_fs["nms_thresh"] >> nms_thresh;

    // If use YoloV3
    if (net_type)
    {
        std::cout << "INFO: Found \"net_type==1\", using **YoloV3** network" << std::endl;
        width = 416;
        height = 416;
        setting_fs["Yolo_config"] >> net_structure;
        setting_fs["Yolo_model"] >> model;
        setting_fs["coco_name"] >> name_file;
        setting_fs["Yolo_scaleFactor"] >> scaleFactor;
        setting_fs["Yolo_meanVal"] >> meanVal;
    }

    // If use SSD
    else
    {
        std::cout << "INFO: Found \"net_type==0\", using **SSD** network" << std::endl;
        width = 300;
        height = 300;
        setting_fs["ssd_config"] >> net_structure;
        setting_fs["ssd_model"] >> model;
        setting_fs["ssd_name"] >> name_file;
        setting_fs["ssd_scaleFactor"] >> scaleFactor;
        setting_fs["ssd_meanVal"] >> meanVal;
    }
    net_structure = ros::package::getPath("detector") + net_structure;
    model = ros::package::getPath("detector") + model;
    name_file =  ros::package::getPath("detector") + name_file;
    std::cout<<"Cfg: "<<net_structure<<std::endl;
    //if (net_type) net = cv::dnn::readNetFromDarknet(net_structure, model);
    //else net = cv::dnn::readNetFromCaffe(net_structure, model);
    net = cv::dnn::readNet(net_structure, model);
    if (net.empty())
    {
        std::cerr << "ERROR: Can't load network by using the following files: " << std::endl;
        exit(-1);
    }
    else
        std::cout << "INFO: Load network sucessfully" << std::endl;

    // Read lable file
    std::ifstream ifs(name_file.c_str());
    std::string line;
    while (std::getline(ifs, line))
        classes.push_back(line);
}

/****************************************************************************** 
    Function:       Detector::getOutputsNames
    Description:    Get the names of network output layers
    Input:          Net &net                    - the dnn network
    Output:         vector<std::string> names   - the names of output layers
    Return:         vector<std::string> names
    ****************************************************************************/
std::vector<cv::String> DnnDetector::getOutputsNames(cv::dnn::Net &net)
{
    static std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers
        std::vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

/****************************************************************************** 
    Function:       Detector::drawPred (*Not Used in the Project*)
    Description:    Draw the predictions
    ****************************************************************************/
void DnnDetector::drawPred(int classId, float conf, int left, int top, int right, int bottom, cv::Mat& frame,std::vector<std::string> classes)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    std::string label = cv::format("%.5f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId]  + ":" + label;
    }
    //Display the label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    rectangle(frame, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
}

/****************************************************************************** 
    Function:       DnnDetector::drawResult (*Not Used in the Project*)
    Description:    Draw the predictions
    Input:          cv::Mat& frame                          - image to draw
                    std::vector<std::string> out_names      - names of objects
                    std::vector<cv::Rect> out_boxes         - positions of objects
                    std::vector<float> confidences          - confidences of objects
                    std::vector<cv::Point> out_centers      - centers of objects
                    bool if_fps                             - whether draw FPS
    Output:         image with results drawn
    Return:         void
    ****************************************************************************/
void DnnDetector::drawResult(cv::Mat& frame, std::vector<std::string> out_names, std::vector<cv::Rect> out_boxes,std::vector<float> confidences,std::vector<cv::Point> out_centers,bool if_fps)
{
    //Draw a rectangle displaying the bounding box
    for (int n = 0; n < out_boxes.size(); n++)
    {
        rectangle(frame,out_boxes[n].tl(), out_boxes[n].br(), cv::Scalar(255, 178, 50), 3);
        std::string label = cv::format("%.2f%%", 100*confidences[n]);
        label = out_names[n]  + ":" + label;
        out_names.push_back(classes[n]);
        //Display the label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        int top = std::max(out_boxes[n].y, labelSize.height);
        int left = out_boxes[n].x;
        rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        putText(frame, label, cv::Point(left,top ), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
        putText(frame,cv::format("* (%d,%d)",out_centers[n].x,out_centers[n].y),out_centers[n],cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    }
    if(if_fps){
        // display fps at top left of image
        std::string display = cv::format("FPS: %.2f", fps);
	    cv::putText(frame, display, cv::Point(0, 15), 0, 0.5, cv::Scalar(0, 0, 255));
    }
}

/****************************************************************************** 
    Function:       DnnDetector::yoloProcess
    Description:    Detection using Yolo
    Input:          cv::Mat& frame,                         - image need to be detected
                    const std::vector<cv::Mat>& outs        - output layers
                    float confThreshold                     - confidence threshold
                    float nmsThreshold                      - non maximum suppression threshold
    Output:         std::vector<cv::Rect> out_boxes;        - positions of objects
                    std::vector<cv::Point> out_centers;     - centers of objects
                    std::vector<std::string> out_names;     - names of objects
                    std::vector<float> out_confidences;     - confidences of objects
    Return:         void
    ****************************************************************************/
void DnnDetector::yoloProcess(cv::Mat frame, const std::vector<cv::Mat>& outs, float confThreshold, float nmsThreshold)
{
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    // for yolo3-tiny, 2 scalars(13*13 and 26*26)
    // for yolo3, 3 scalars(13*13, 26*26, 52*52)
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        // the data is [x, y, w, h, confidence,witch class*80]
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);      // the propabilities of 80 classes
            cv::Point classIdPoint;
            double confidence;
            // Get the value(confidence) and location(classIdPoint) of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            // get all the bboxes of the most possible class
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);  
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }
    
    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        out_boxes.push_back(box);
        out_confidences.push_back(confidences[idx]);
        out_names.push_back(classes[classIds[idx]]);
        out_centers.push_back(cv::Point(box.x+box.width/2,box.y+box.height/2));
        //rectangle(frame,box, Scalar(0, 0, 255), 2, 8, 0);
        //drawPred(classIds[idx], confidences[idx], box.x, box.y,
        //    box.x + box.width, box.y + box.height, frame,classes);
    }
}

/****************************************************************************** 
    Function:       DnnDetector::thePredictor
    Description:    Load and run the dnn network 
    Input:          cv::Mat frame               - image need to be detected
                    std::vector<cv::Mat> outs   - output layers of network
    Output:         std::vector<cv::Rect> out_boxes;        - positions of objects
                    std::vector<cv::Point> out_centers;     - centers of objects
                    std::vector<std::string> out_names;     - names of objects
                    std::vector<float> out_confidences;     - confidences of objects
    Return:         void
    ****************************************************************************/
void DnnDetector::ssdPropcess(cv::Mat frame, std::vector<cv::Mat> outs)
{
    // ssd, only one output layer, so get the first layer
    // for ssd output layer:
    // [image_id, label, confidence, xmin, ymin, xmax, ymax]
    cv::Mat detectionMat(outs[0].size[2], outs[0].size[3], CV_32F, outs[0].ptr<float>());
    for (int i = 0; i < detectionMat.rows; i++) {               // go through all classes
			float confidence = detectionMat.at<float>(i, 2);
			if (confidence > thresh) {
				size_t objIndex = (size_t)(detectionMat.at<float>(i, 1));
				float tl_x = detectionMat.ptr<float>(i) [3] * frame.cols;
				float tl_y = detectionMat.ptr<float>(i) [4] * frame.rows;
				float br_x = detectionMat.ptr<float>(i) [5] * frame.cols;
                float br_y = detectionMat.ptr<float>(i) [6] * frame.rows;
                
                out_confidences.push_back(confidence);
                out_boxes.push_back(cv::Rect((int)tl_x, (int)tl_y, (int)(br_x - tl_x), (int)(br_y - tl_y)));
                out_centers.push_back(cv::Point((int)(br_x + tl_x)/2, (int)(br_y + tl_y)/2));
                out_names.push_back(classes[objIndex]);
			}
	}
}

/****************************************************************************** 
    Function:       DnnDetector::thePredictor
    Description:    Load and run the dnn network 
    Input:          cv::Mat frame               - image need to be detected
                    cv::dnn::Net net            - the dnn network
    Output:         detection results
    Return:         void
    ****************************************************************************/
void DnnDetector::thePredictor(cv::Mat frame,cv::dnn::Net net)
{
    // clear public variables
    out_boxes.clear();
    out_names.clear();
    out_confidences.clear();
    out_centers.clear();

    frame.copyTo(outputimage);
    // Creates 4-dimensional blob from image
    cv::Mat inputblob = cv::dnn::blobFromImage(outputimage, scaleFactor, cv::Size(width, height), meanVal, false);   
	net.setInput(inputblob);            // Set the input value for the network. 
    std::vector<cv::Mat> outs;          //	contains all output blobs for specified layer

    net.forward(outs, getOutputsNames(net));    // network forward
    // results output
    if(net_type)    
        yoloProcess(outputimage, outs, thresh, nms_thresh);
    else    
        ssdPropcess(outputimage,outs);

    // calculate FPS
    std::vector<double> layersTimes;
    fps = 1000/(net.getPerfProfile(layersTimes) / ( cv::getTickFrequency() / 1000));
}
