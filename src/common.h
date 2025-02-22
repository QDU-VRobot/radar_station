#ifndef _COMMON_H_
#define _COMMON_H_

#include <string.h>
#include <stdio.h>
#include <utility>
#include "opencv2/videoio.hpp"
#include "hikcamera.h"
int run(const char* car_engine, const char* armor_engine, const std::pair<const std::pair<int,int>,int>& image_arg, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec, bool wait_forever=false);
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
static size_t getsavename(char* savename, const char* pathname, const char* saveext) //Generate savename based on filename (include path) and saveext
{
	const char* fileext=strrchr(pathname,'/');
	fileext=strrchr(fileext?fileext+1:pathname,'.');
	size_t extlen=sizeof(saveext);
	size_t savelen=strlen(pathname)+extlen-(fileext?strlen(fileext):0);
	if(!savename)
		return savelen;
	strncpy(savename,pathname,savelen);
	strcpy(savename+savelen-extlen,saveext);
	return 0;
}

#endif
