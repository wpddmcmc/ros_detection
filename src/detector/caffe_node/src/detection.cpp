/*********************************************************************************
 * Copyright (c) 2019 Michael.Chen. All rights reserved.
 * File name: detection.cpp
 * Created on 24th/Jul/2019
 * Author: Michael.Chen
 * Follow me: https://www.tgeek.tech
 * ******************************************************************************/
#include "detection.hpp"

/****************************************************************************** 
    Function:       Detector::Detect
    Description:    Get the outputs of output layers of network
    Input:          cv::Mat &img                -the image need to be detected
    Output:         std::vector<std::vector<float>> detections     -the outputs of output layers
    Return:         std::vector<std::vector<float>> detections 
    ****************************************************************************/
std::vector<std::vector<float>> Detector::Detect(const cv::Mat &img)
{
  std::vector<std::vector<float>> detections;
  caffe::Blob<float> *input_layer = net_->input_blobs()[0];

  input_layer->Reshape(1, num_channels_,input_geometry_.height, input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  // Wrap the input layer of the network in separate cv::Mat objects
  std::vector<cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  // Convert the input image to the input image format of the network.
  Preprocess(img, &input_channels);
  // run forward progress
  net_->Forward();

  // Copy the output layer to a std::vector
  caffe::Blob<float> *result_blob = net_->output_blobs()[0];
  const float *result = result_blob->cpu_data();
  const int num_det = result_blob->height();
  for (int k = 0; k < num_det; ++k)
  {
    if (result[0] == -1)
    {
      // Skip invalid detection.
      result += 7;
      continue;
    }
    std::vector<float> detection(result, result + 7);
    detections.push_back(detection);
    result += 7;
  }
  return detections;
}

/****************************************************************************** 
    Function:       Detector::SetMean
    Description:    Load the mean file in binaryproto format
    Input:          const std::string& mean_file        -binary file of mean 
                    const std::string& mean_value       -value of mean (1 or 3 channels)
    Output:         cv::Mat mean_                       -the mean image
    Return:         void
****************************************************************************/
void Detector::SetMean(const std::string& mean_file, const std::string& mean_value) {
  cv::Scalar channel_mean;
  // load mean file
  if (!mean_file.empty()) {
    CHECK(mean_value.empty()) <<
      "Cannot specify mean_file and mean_value at the same time";
    caffe::BlobProto blob_proto;
    ReadProtoFromBinaryFileOrDie(mean_file.c_str(), &blob_proto);

    // Convert from BlobProto to Blob<float> 
    caffe::Blob<float> mean_blob;
    mean_blob.FromProto(blob_proto);
    CHECK_EQ(mean_blob.channels(), num_channels_)
      << "Number of channels of mean file doesn't match input layer.";

    // The format of the mean file is planar 32-bit float BGR or grayscale.
    std::vector<cv::Mat> channels;
    float* data = mean_blob.mutable_cpu_data();
    for (int i = 0; i < num_channels_; ++i) {
      // Extract an individual channel. 
      cv::Mat channel(mean_blob.height(), mean_blob.width(), CV_32FC1, data);
      channels.push_back(channel);
      data += mean_blob.height() * mean_blob.width();
    }

    // Merge the separate channels into a single image.
    cv::Mat mean;
    cv::merge(channels, mean);

    // Compute the global mean pixel value and create a mean image
    // filled with this value. 
    channel_mean = cv::mean(mean);
    mean_ = cv::Mat(input_geometry_, mean.type(), channel_mean);
  }
  // use mean value
  if (!mean_value.empty()) {
    CHECK(mean_file.empty()) <<
      "Cannot specify mean_file and mean_value at the same time";
    std::stringstream ss(mean_value);
    std::vector<float> values;
    std::string item;
    while (getline(ss, item, ',')) {
      float value = std::atof(item.c_str());
      values.push_back(value);
    }
    CHECK(values.size() == 1 || values.size() == num_channels_) <<
      "Specify either 1 mean_value or as many as channels: " << num_channels_;

    std::vector<cv::Mat> channels;
    for (int i = 0; i < num_channels_; ++i) {
      /* Extract an individual channel. */
      cv::Mat channel(input_geometry_.height, input_geometry_.width, CV_32FC1,
          cv::Scalar(values[i]));
      channels.push_back(channel);
    }
    cv::merge(channels, mean_);
  }
}

/****************************************************************************** 
    Function:       Detector::WrapInputLayer
    Description:    Wrap the input layer of the network in separate cv::Mat objects,
                    The last preprocessing operation will write the separate channels 
                    directly to the input layer
    Input:          std::vector<cv::Mat>* input_channels      
    Output:         std::vector<cv::Mat>* input_channels      -input layer
    Return:         void
****************************************************************************/
void Detector::WrapInputLayer(std::vector<cv::Mat>* input_channels) {
  caffe::Blob<float>* input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float* input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i) {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

/****************************************************************************** 
    Function:       Detector::Preprocess
    Description:    Convert the input image to the input image format of the network
    Input:          const cv::Mat& img                        -image need to be detected
                    std::vector<cv::Mat>* input_channels      -input layer
    Output:          const cv::Mat& img                       -image need to be detected
    Return:         void
****************************************************************************/
void Detector::Preprocess(const cv::Mat& img,
                            std::vector<cv::Mat>* input_channels) {
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  if (num_channels_ == 3)
    sample_resized.convertTo(sample_float, CV_32FC3);
  else
    sample_resized.convertTo(sample_float, CV_32FC1);

  cv::Mat sample_normalized;
  cv::subtract(sample_float, mean_, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
  }

  /****************************************************************************** 
    Function:       Detector::GetResult
    Description:    Run prediction
    Input:          cv::Mat frame                               -image need to be detected
    Output:         std::vector<cv::Rect> Boxes;                -bounding boxes of detected objects
                    std::vector<std::string> Class_names;       -classes of detected objects
                    std::vector<float> Confidences;             -confidences of detected objects
                    std::vector<cv::Point> Centers;             -center position of detected objects
    Return:         void
****************************************************************************/
  void Detector::GetResult(cv::Mat frame)
  {
    // clear outputs
    Class_names.clear();
    Confidences.clear();
    Boxes.clear();
    Centers.clear();

    std::vector<std::vector<float>> detections = Detect(frame);
    for (int i = 0; i < detections.size(); ++i)
    {
      std::vector<float> &d = detections[i];
      // Detection format: [image_id, label, score, xmin, ymin, xmax, ymax].
      const float score = d[2];
      if (score >= thresh)
      {
        Class_names.push_back(labels[int(d[1])]);
        Confidences.push_back(score);
        Boxes.push_back(cv::Rect(int(d[3] * frame.cols), int(d[4] * frame.rows),
                                int((d[5] - d[3]) * frame.cols), int((d[6] - d[4]) * frame.rows)));
        Centers.push_back(cv::Point(int(((d[5] + d[3]) * frame.cols)/2),int(((d[6] + d[4]) * frame.rows)/2)));
      }
    }
  }

/****************************************************************************** 
    Function:       Detector::GetResult
    Description:    Run prediction
    Input:          cv::Mat frame                               -image need to draw
                    std::vector<cv::Rect> Boxes;                -bounding boxes of detected objects
                    std::vector<std::string> Class_names;       -classes of detected objects
                    std::vector<float> Confidences;             -confidences of detected objects
                    std::vector<cv::Point> Centers;             -center position of detected objects
    Outputs:        cv::Mat frame                               -image drawn results
    Return:         void
****************************************************************************/
  void Detector::DrawResult(cv::Mat &frame, std::vector<std::string> Class_names,
                            std::vector<cv::Rect> Boxes, std::vector<float> Confidences, std::vector<cv::Point> Centers)
  {
    for(int n=0; n<Class_names.size(); n++){
      cv::rectangle(frame,Boxes[n],cv::Scalar(255, 178, 50), 3);
      std::string label = cv::format("%.2f%%", 100*Confidences[n]);
      label = Class_names[n] + ":" + label;
      //Display the label at the top of the bounding box
      int baseLine;
      cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
      int top = std::max(Boxes[n].y, labelSize.height);
      int left = Boxes[n].x;
      rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
      putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1);
      putText(frame, cv::format("* (%d,%d)", Centers[n].x, Centers[n].y), Centers[n], cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 0, 255), 1);
    }
  }