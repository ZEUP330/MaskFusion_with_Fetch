/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 *
 * The use of the code within this file and all code within files that
 * make up the software that is ElasticFusion is permitted for
 * non-commercial purposes only.  The full terms and conditions that
 * apply to the code within this file are detailed within the LICENSE.txt
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/>
 * unless explicitly stated.  By downloading this file you agree to
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#pragma once

#include <Utils/Resolution.h>
#include <Utils/Stopwatch.h>
#include <pangolin/utils/file_utils.h>
#include "../Core/FrameData.h"

#include "LogReader.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <stack>



#include "mujoco.h"
#include "stdlib.h"
#include "string.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
// select EGL, OSMESA or GLFW
#if defined(MJ_EGL)
    #include <EGL/egl.h>
#elif defined(MJ_OSMESA)
    #include <GL/osmesa.h>
    OSMesaContext ctx;
    unsigned char buffer[10000000];
#else
    #include "glfw3.h"
#endif




class MujocoReader : public LogReader {
 public:

  MujocoReader(std::string file, bool flipColors);

  virtual ~MujocoReader();
  
  // load model, init simulation and rendering
  void initMuJoCo(const char* filename);
  // deallocate everything and deactivate
  void closeMuJoCo(void);
  // create OpenGL context/window
  void initOpenGL(void);
  // close OpenGL context/window
  void closeOpenGL(void);

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

 private:
  FrameDataPointer readFrame();
  FrameDataPointer currentDataPointer;

  cv::Mat depthDecompressionBuffer;
  cv::Mat rgbDecompressionBuffer;
  cv::Mat depthBuffer;
  cv::Mat rgbBuffer;
  unsigned char* rgb;
  float* depth;
  int W, H;
  double frametime;
  int framecount;
  double duration, fps;
  mjrRect viewport ;
    // MuJoCo model and data
    mjModel* m;
    mjData* d;
    // MuJoCo visualization
    mjvScene scn;
    mjvCamera cam;
    mjvOption opt;
    mjrContext con;
    GLFWwindow* window ;
};
