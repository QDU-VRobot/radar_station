#ifndef _COMMON_H_
#define _COMMON_H_

#include <string.h>
#include <stdio.h>
#include <utility>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "hikcamera.h"
#include "serial.h"
#define SERIAL_SPEED B115200
extern "C"
{
extern uint8_t mode;
extern struct serial_info
{
	uint16_t stage_remain_time;
	uint8_t robot_id;
	uint8_t mark_progress:6;
	uint8_t game_progress:3;
	uint8_t more_damage:2;
	uint8_t damaging:1;
} serial_info;
int run(const char* car_engine, const char* armor_engine, const std::pair<const std::pair<int,int>,float>& image_arg, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec, serial::terminal& sport, bool wait_forever=false, const cv::Mat& trans_matrix=cv::Mat(), const cv::Mat& cam_matrix=cv::Mat(), const cv::Mat& dist_coeffs=cv::Mat(), const cv::Mat& example_img=cv::Mat(), float scale=1);
void receive(serial::terminal* sport,const std::string& filename);
uint8_t send_pos(serial::terminal* sport,const std::vector<cv::Point2f>& center_points,const std::vector<uint8_t>& ids,bool* lock=0);
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
inline bool open_serial(serial::terminal& port, const char* filename)
{
	if(port.open(filename)<0)
		return false;
	port.set_rate(SERIAL_SPEED);
	port.options.c_cflag&=~(CSIZE|CSTOPB|PARENB);
	port.options.c_cflag|=(CS8|CREAD|CLOCAL);
	port.options.c_iflag&=~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
	port.options.c_oflag&=~OPOST;
	port.options.c_lflag&=~(ICANON|ECHO|ISIG);
	return port.apply_options()==0;
}
}

#endif
