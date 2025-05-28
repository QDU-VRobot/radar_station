#ifndef _COMMON_H_
#define _COMMON_H_

#include <string.h>
#include <stdio.h>
#include <tuple>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "hikcamera.h"
#include "serial.h"
#define SERIAL_SPEED B115200
extern "C"
{
extern uint8_t mode;
extern volatile struct serial_info //See RoboMaster doc
{
	uint16_t stage_remain_time;
	uint8_t robot_id;
	uint8_t mark_progress:5; //Robot 12347
	uint8_t game_progress:3;
	uint8_t more_damage:2;
	uint8_t damaging:1;
} serial_info;
int run(const char* car_engine, const char* armor_engine, const std::tuple<int,int,float>& image_arg, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec, serial::terminal& sport, bool wait_forever=false, const std::tuple<cv::Mat,cv::Mat,std::vector<std::pair<std::vector<cv::Point2f>,float>>>& pose_points=std::make_tuple(cv::Mat(),cv::Mat(),std::vector<std::pair<std::vector<cv::Point2f>,float>>()), const cv::Mat& cam_matrix=cv::Mat(), const cv::Mat& dist_coeffs=cv::Mat(), const cv::Mat& example_img=cv::Mat(), float scale=1);
void receive(serial::terminal& sport);
uint8_t send_pos(serial::terminal& sport,const std::vector<cv::Point2f>& center_points,const std::vector<uint8_t>& ids,bool* lock=0);
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
	if(!savename)
		return savelen;
	strncpy(savename,pathname,savelen);
	strcpy(savename+savelen-extlen,saveext);
	return 0;
}
inline bool open_serial(serial::terminal& port, const char* filename=0)
{
	if(filename)
	{
		if(port.open(filename)<0)
			return false;
	}
	else if(port.open()<0)
		return false;
	port.set_rate(SERIAL_SPEED);
	port.options.c_cflag&=~(CSIZE|CSTOPB|PARENB);
	port.options.c_cflag|=(CS8|CREAD|CLOCAL);
	port.options.c_iflag&=~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
	port.options.c_oflag&=~OPOST;
	port.options.c_lflag&=~(ICANON|ECHO|ISIG);
	return port.apply_options()==0;
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
