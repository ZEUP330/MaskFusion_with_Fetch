
#include "../Core/Utils/Macros.h"
#include "rostopicReader.h"

void rosropicLogReader::convert_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image;
    try
    {
        rgb =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) -> image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::circle(rgb, cvPoint(image.cols / 2, image.rows / 2), 2, cv::Scalar(255, 0, 0), -1);

}

void rosropicLogReader::depth_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image;
    try
    {
        dep =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1) -> image; //image.type==CV32FC1   5
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // ROS_INFO("%f", dep.at<float>(dep.rows / 2, dep.cols / 2));

}

rosropicLogReader::rosropicLogReader(std::string file, bool flipColors) : LogReader(file, flipColors)  {

  currentFrame = 0;
  image_transport::ImageTransport it_(nh_);
  depthBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_16UC1);
  rgbBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_8UC3);
  depthDecompressionBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_16UC1);
  rgbDecompressionBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_8UC3);
  image_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 1, &rosropicLogReader::convert_callback, this);
  depth_sub_ = it_.subscribe("/head_camera/depth_registered/image_raw", 1, &rosropicLogReader::depth_callback, this);
  numFrames = 100000000;
  std::cout << "Successful subcribe ros topic;" << std::endl;
}

rosropicLogReader::~rosropicLogReader() {}
// important function
void rosropicLogReader::getNext() {
  currentDataPointer = readFrame();
}
// important function
void rosropicLogReader::getPrevious() {
  currentDataPointer = readFrame();
}

FrameDataPointer rosropicLogReader::readFrame() {
  FrameDataPointer result = std::make_shared<FrameData>();
  result->allocateRGBD(rgbBuffer.cols, rgbBuffer.rows);

  struct timeval tv;
  gettimeofday(&tv,NULL);
  result->timestamp = tv.tv_sec*1000 + tv.tv_usec/1000;
  result->rgb = rgb;
  result->depth = dep;
  cv::imwrite("rgb.jpg", rgb);
  currentFrame++;

  return result;
}
// important function
void rosropicLogReader::fastForward(int frame) {
    assert(0 && "not yet implemented");
}

int rosropicLogReader::getNumFrames() { return numFrames; }
// important function
bool rosropicLogReader::hasMore() { return currentFrame + 1 < numFrames; }
// important function
bool rosropicLogReader::rewind() {
  return false;
}

const std::string rosropicLogReader::getFile() { return file; }
// important function
FrameDataPointer rosropicLogReader::getFrameData() { return currentDataPointer; }

void rosropicLogReader::setAuto(bool value) {}
