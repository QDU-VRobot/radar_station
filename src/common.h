#ifndef _COMMON_H_
#define _COMMON_H_

#include <string.h>
#include <stdio.h>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
extern "C" //Just for shorter symbols :)
{
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
}

#endif
