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

#include "../Core/Utils/Macros.h"
#include "MujocoReader.h"
#include <unistd.h>
//-------------------------------- global data ------------------------------------------


//-------------------------------- utility functions ------------------------------------
// load model, init simulation and rendering
void MujocoReader::initMuJoCo(const char* filename)
{
    // activate
    mj_activate("mjkey.txt");

    // load and compile
    char error[1000] = "Could not load binary model";
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
        m = mj_loadModel(filename, 0);
    else
        m = mj_loadXML(filename, 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data, run one computation to initialize all fields
    d = mj_makeData(m);
    mj_forward(m, d);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, 200);

    // center and scale view
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
}
// deallocate everything and deactivate
void MujocoReader::closeMuJoCo(void)
{
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    mj_deactivate();
}
// create OpenGL context/window
void MujocoReader::initOpenGL(void)
{
    //------------------------ EGL
#if defined(MJ_EGL)
    // desired config
    const EGLint configAttribs[] ={
        EGL_RED_SIZE,           8,
        EGL_GREEN_SIZE,         8,
        EGL_BLUE_SIZE,          8,
        EGL_ALPHA_SIZE,         8,
        EGL_DEPTH_SIZE,         24,
        EGL_STENCIL_SIZE,       8,
        EGL_COLOR_BUFFER_TYPE,  EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE,       EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE,    EGL_OPENGL_BIT,
        EGL_NONE
    };

    // get default display
    EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if( eglDpy==EGL_NO_DISPLAY )
        mju_error_i("Could not get EGL display, error 0x%x\n", eglGetError());

    // initialize
    EGLint major, minor;
    if( eglInitialize(eglDpy, &major, &minor)!=EGL_TRUE )
        mju_error_i("Could not initialize EGL, error 0x%x\n", eglGetError());

    // choose config
    EGLint numConfigs;
    EGLConfig eglCfg;
    if( eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs)!=EGL_TRUE )
        mju_error_i("Could not choose EGL config, error 0x%x\n", eglGetError());

    // bind OpenGL API
    if( eglBindAPI(EGL_OPENGL_API)!=EGL_TRUE )
        mju_error_i("Could not bind EGL OpenGL API, error 0x%x\n", eglGetError());

    // create context
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, NULL);
    if( eglCtx==EGL_NO_CONTEXT )
        mju_error_i("Could not create EGL context, error 0x%x\n", eglGetError());

    // make context current, no surface (let OpenGL handle FBO)
    if( eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, eglCtx)!=EGL_TRUE )
        mju_error_i("Could not make EGL context current, error 0x%x\n", eglGetError());

    //------------------------ OSMESA
#elif defined(MJ_OSMESA)
    // create context
    ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, 0);
    if( !ctx )
        mju_error("OSMesa context creation failed");

    // make current
    if( !OSMesaMakeCurrent(ctx, buffer, GL_UNSIGNED_BYTE, 800, 800) )
        mju_error("OSMesa make current failed");

    //------------------------ GLFW
#else
    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create invisible window, single-buffered
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
    if( !window )
        mju_error("Could not create GLFW window");

    // make context current
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
#endif
}
// close OpenGL context/window
void MujocoReader::closeOpenGL(void)
{
    //------------------------ EGL
#if defined(MJ_EGL)
    // get current display
    EGLDisplay eglDpy = eglGetCurrentDisplay();
    if( eglDpy==EGL_NO_DISPLAY )
        return;

    // get current context
    EGLContext eglCtx = eglGetCurrentContext();

    // release context
    eglMakeCurrent(eglDpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    // destroy context if valid
    if( eglCtx!=EGL_NO_CONTEXT )
        eglDestroyContext(eglDpy, eglCtx);

    // terminate display
    eglTerminate(eglDpy);

    //------------------------ OSMESA
#elif defined(MJ_OSMESA)
    OSMesaDestroyContext(ctx);

    //------------------------ GLFW
#else
    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif
#endif
}
MujocoReader::MujocoReader(std::string file, bool flipColors) : LogReader(file, flipColors) {
  depthBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_16UC1);
  rgbBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_8UC3);
  depthDecompressionBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_16UC1);
  rgbDecompressionBuffer = cv::Mat(Resolution::getInstance().height(), Resolution::getInstance().width(), CV_8UC3);
  currentFrame = 0;

    frametime = 0;
    framecount = 0;
    duration = 10;
    fps = 30;
    m = 0;
    d = 0;
    // initialize OpenGL and MuJoCo
    // sleep(1);
    initOpenGL();
    initMuJoCo("/home/vcc/.mujoco/mujoco200/model/arm26.xml");

    // set rendering to offscreen buffer
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if( con.currentBuffer!=mjFB_OFFSCREEN )
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");

    // get size of active renderbuffer
    viewport =  mjr_maxViewport(&con);
    W = viewport.width;
    H = viewport.height;

    // allocate rgb and depth buffers
    rgb = (unsigned char*)malloc(3*W*H);
    depth = (float*)malloc(sizeof(float)*W*H);
        
    // main loop


  std::cout << "Reading log file: " << file << " which has " << numFrames << " frames. " << std::endl;
}

MujocoReader::~MujocoReader() {
    
    // close file, free buffers
    fclose(fp);
    free(rgb);
    free(depth);

    // close MuJoCo and OpenGL
    closeMuJoCo();
    closeOpenGL();

}

void MujocoReader::getNext() {
  currentDataPointer = readFrame();
}

void MujocoReader::getPrevious() {
  currentDataPointer = readFrame();
}

FrameDataPointer MujocoReader::readFrame() {

// update abstract scene
        printf("%f %f",d->time-frametime, 1/fps);
        if( (d->time-frametime)>1/fps || frametime==0 )
        {

            // usleep(1);
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
            // mj_setBuffer(mjFB_WINDOW, &con);

            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            // render scene in offscreen buffer
            mjr_render(viewport, &scn, &con);

            // add time stamp in upper-left corner
            char stamp[50];
            sprintf(stamp, "Time = %.3f", d->time);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

            // read rgb and depth buffers
            mjr_readPixels(rgb, depth, viewport, &con);
            // show the image in here
            printf("%d, %d\n", W, H);
            Mat cvRawImg,cvDepthImg;
            Mat src_rgb = Mat(H, W, CV_8UC3, rgb);
            // src_rgb.convertTo(src_rgb, CV_8FC3, 255);
            cvtColor(src_rgb, cvRawImg, COLOR_RGB2BGR);
            flip(cvRawImg,cvRawImg, 0);

            float Bigest = 0, smallest = 100;
            for(int k=0; k<H*W; k++) {
                Bigest = Bigest > depth[k]?Bigest:depth[k];
                smallest = smallest < depth[k]?smallest:depth[k];
            }
            printf("%f %f\n", Bigest, smallest);
            /**/for(int k=0; k<H; k++)
                for(int k1=0; k1<W; k1++){
                    //printf("%f ", depth[k*W+k1]);
                    depth[k*W+k1]=(depth[k*W+k1]-smallest)/(Bigest-smallest);
                }
            Mat cvRawdep16U = Mat(H, W, CV_32FC1, depth);
            // cvRawdep16U.convertTo(cvDepthImg, CV_16U, 255);
            flip(cvRawdep16U,cvRawdep16U, 0);

            rgbBuffer = cvRawImg;
            depthBuffer = cvRawdep16U;
            cv::imwrite("rgbBuffer.jpg", cvRawImg);
            cv::imwrite("depthBuffer.jpg", cvRawdep16U);
            framecount++;
            frametime = d->time;
        }

  FrameDataPointer result = std::make_shared<FrameData>();
  result->allocateRGBD(W, H);

  //depthBuffer.convertTo(result->depth, CV_32FC1, 0.001);
  result->depth = depthBuffer;
  result->rgb = rgbBuffer;
  if (flipColors) result->flipColors();
  currentFrame++;


  mj_step(m, d);

  return result;
}

void MujocoReader::fastForward(int frame) {
}

int MujocoReader::getNumFrames() { return numFrames; }

bool MujocoReader::hasMore() { return  d->time<duration; }

bool MujocoReader::rewind() {
}

const std::string MujocoReader::getFile() { return file; }

FrameDataPointer MujocoReader::getFrameData() { return currentDataPointer; }

void MujocoReader::setAuto(bool value) {}
