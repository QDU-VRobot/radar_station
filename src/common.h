#ifndef _COMMON_H_
#define _COMMON_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <chrono>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "camera.h"
#include "aravis.h"
extern "C" //Just for shorter symbols :)
{
enum mode_flag : uint8_t {
    MODE_USE_CAR=0,
    MODE_USE_ARMOR=1,
    MODE_DISPLAY=2,
    MODE_SERIAL=3,
    MODE_RECORD_RAW=4,
    MODE_RECORD_PROC=5,
    MODE_LOOP=6,
    MODE_PERSISTENT=7
};
inline bool file_readable(const char* filename)
{
    FILE* test=fopen(filename,"rb");
    if(test)
    {
        fclose(test);
        return true;
    }
    return false;
}
inline size_t getsavename(char* savename, const char* pathname, const char* saveext) //Generate savename based on filename (include path) and saveext
{
    const char* fileext=strrchr(pathname,'/');
    fileext=strrchr(fileext?fileext+1:pathname,'.');
    size_t extlen=sizeof(saveext);
    size_t savelen=strlen(pathname)+extlen-(fileext?strlen(fileext):0);
    if(savename)
    {
        strncpy(savename,pathname,savelen);
        strcpy(savename+savelen-extlen,saveext);
    }
    return savelen;
}
inline bool start_camera(camera::camera& cam)
{
    if(cam.isopened())
    {
        if(cam.start())
        {
            fprintf(stderr,"An error occurred while starting video [%d]\n",camera::aravis::last_error?camera::aravis::last_error->code:0);
            return false;
        }
        return true;
    }
    return false;
}
inline std::pair<cv::Mat,int64_t> grabMatTime(camera::camera& cam,unsigned int ms=-1)
{
    int64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto [data, fmt, width, height] = cam.grab(ms);
    if(data)
    {
        cv::Mat img = cv::Mat(height, width, CV_8UC1, const_cast<void*>(data)).clone();
        cam.free();
        return std::make_pair(img, timestamp);
    }
    return std::make_pair(cv::Mat(),timestamp);
}
inline cv::Mat calculateCameraMatrix(cv::Size imgsize, double fov) //Returns an ideal camera matrix with a given FOV
{
    double f=0.5/tan(fov/2);
    return cv::Mat_<double>(3,3)<<imgsize.width*f,0,imgsize.width/2,0,imgsize.height*f,imgsize.height/2,0,0,1;
}
inline cv::Point2f backProjectPoint(cv::Point3d imagePoint, const cv::Mat& m1, const cv::Mat& m2) //imagePoint=video (x,y) and realistic z, m1=R^-1*K^-1, m2=R^-1*T
{
    cv::Mat tmp=m1*cv::Mat(cv::Point3d(imagePoint.x,imagePoint.y,1));
    tmp*=(imagePoint.z+m2.at<double>(2))/tmp.at<double>(2);
    tmp-=m2;
    return cv::Point2f(tmp.at<double>(0),tmp.at<double>(1));
}
inline cv::Vec2f getPerpDirection(cv::Point2f left, cv::Point2f right) {
    cv::Vec2f d = right - left;
    float len = cv::norm(d);
    return cv::Vec2f(-d[1], d[0]) / len;
}
}

#endif
