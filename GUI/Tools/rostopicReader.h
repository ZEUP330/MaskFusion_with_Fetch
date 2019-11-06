#pragma once

#include <Utils/Resolution.h>
#include <Utils/Stopwatch.h>
#include <pangolin/utils/file_utils.h>
#include "../Core/FrameData.h"

#include "LogReader.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <cassert>
#include <zlib.h>
#include <iostream>
#include <string>
#include <stack>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include<opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
void rgb_read(const sensor_msgs::ImageConstPtr& msg);
void dep_read(const sensor_msgs::ImageConstPtr& msg);
cv::Mat read_rgb();
cv::Mat read_dep();



class rosropicLogReader : public LogReader {
 public:
  rosropicLogReader(std::string file, bool flipColors);

  virtual ~rosropicLogReader();



void convert_callback(const sensor_msgs::ImageConstPtr& msg);

void depth_callback(const sensor_msgs::ImageConstPtr& msg);

  void getNext();

  void getPrevious();

  int getNumFrames();

  bool hasMore();

  bool rewind();

  void fastForward(int frame);

  const std::string getFile();

  FrameDataPointer getFrameData();

  void setAuto(bool value);

  std::stack<int> filePointers;

  cv::Mat rgb, dep;
 private:
  FrameDataPointer readFrame();
  FrameDataPointer currentDataPointer;

  cv::Mat depthDecompressionBuffer;
  cv::Mat rgbDecompressionBuffer;
  cv::Mat depthBuffer;
  cv::Mat rgbBuffer;
  ros::NodeHandle nh_;
  image_transport::Subscriber image_sub_, depth_sub_;
};
