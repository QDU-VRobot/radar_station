#include "common.h"
#include "hikcamera.h"
#include "convert.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <utility>
#include "json/json.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
struct model_config
{
	uint32_t builder_flag=0b10000001001; //See NvInfer.h
	const char *car_file="models/car.onnx", *armor_file="models/armor.onnx";
	const char *save_ext=".engine";
	int64_t img_width=640,img_height=640;
	int64_t batch_size=6;
};
struct video_config
{
	uint8_t video_type=2; //0-Video file; 1-Common camera; 2-3rd party camera (hikcamera)
	const char* video_file=0;
	uint8_t video_device=0; //Device number
	float fps=-1; //FPS, -1=auto, 0=keep original
	float exposure_time=-1; //Exposure time, -1=auto, 0=keep original
	float gain=-1; //Gain, -1=auto
};
inline hikcamera::camera init_hikcamera(uint8_t device_num)
{
	hikcamera::CAMERA_INFO_LIST devlist=hikcamera::camera::enum_cameras();
	if(devlist.nDeviceNum<=device_num)
		throw std::runtime_error("Camera not found, check camera connection");
	std::cerr<<"Using camera "<<static_cast<unsigned>(device_num)<<std::endl;
	return hikcamera::camera(devlist.pDeviceInfo[device_num]);
}
uint8_t mode=0b0111; //Bit 0: Use car model; Bit 1: Use armor model (car model must be used); Bit 2: Display video; Bit 3: Record video; All 0: Generate only
size_t threads=4;
std::chrono::milliseconds wait_timeout(2);
int convert(const struct model_config& config, size_t threads, char** car_engine_name=0, char** armor_engine_name=0)
{
	char* car_engine=new char[getsavename(0,config.car_file,config.save_ext)+1];
	getsavename(car_engine,config.car_file,config.save_ext);
	char* armor_engine=new char[getsavename(0,config.armor_file,config.save_ext)+1];
	getsavename(armor_engine,config.armor_file,config.save_ext);
	if(mode==0||!file_readable(car_engine))
	{
		std::cerr<<"Generating car engine..."<<std::endl;
		if(!file_readable(config.car_file)||!convert_onnx(config.car_file,car_engine,config.builder_flag,threads,true,config.img_width,config.img_height,config.batch_size))
		{
			std::cerr<<"Failed to convert car model"<<std::endl;
			if(mode==0)
				return 1;
			std::cerr<<"Warning: Inference not available"<<std::endl;
			mode&=~1;
		}
	}
	if(mode==0||!file_readable(armor_engine))
	{
		std::cerr<<"Generating armor engine..."<<std::endl;
		if(!file_readable(config.armor_file)||!convert_onnx(config.armor_file,armor_engine,config.builder_flag,threads,true,config.img_width,config.img_height,config.batch_size))
		{
			std::cerr<<"Failed to convert armor model"<<std::endl;
			if(mode==0)
				return 1;
			std::cerr<<"Warning: Inference without armor model"<<std::endl;
			mode&=~2;
		}
	}
	if(mode==0)
	{
		std::cerr<<"Convert finished, exiting."<<std::endl;
		delete[] car_engine;
		delete[] armor_engine;
		return 0;
	}
	if(car_engine_name)
		*car_engine_name=car_engine;
	else
		delete[] car_engine;
	if(armor_engine_name)
		*armor_engine_name=armor_engine;
	else
		delete[] armor_engine;
	return -1; //Normal
}
std::pair<int,int> apply_video_config(const struct video_config& config, cv::VideoCapture& cap, hikcamera::camera& hikcam)
{
	int width,height;
	if(cap.isOpened())
	{
		width=static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
		height=static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
		if(config.fps>0)
			cap.set(cv::CAP_PROP_FPS,config.fps);
		if(config.exposure_time!=0)
			cap.set(cv::CAP_PROP_AUTO_EXPOSURE,config.exposure_time<0);
		if(config.exposure_time>0)
			cap.set(cv::CAP_PROP_EXPOSURE,config.exposure_time);
		if(config.gain>0)
			cap.set(cv::CAP_PROP_GAIN,config.gain);
	}
	if(hikcam.get_handle())
	{
		union hikcamera::gettype g;
		hikcam.get_value(hikcamera::INT,"Width",&g);
		width=g.stIntValue.nCurValue;
		hikcam.get_value(hikcamera::INT,"Height",&g);
		height=g.stIntValue.nCurValue;
		union hikcamera::settype s;
		s.eValue=0;
		hikcam.set_value(hikcamera::ENUM,"TriggerMode",s);
		if(config.fps!=0)
		{
			s.bValue=config.fps>0;
			hikcam.set_value(hikcamera::BOOL,"AcquisitionFrameRateEnable",s);
			if(config.fps>0)
			{
				s.fValue=config.fps;
				hikcam.set_value(hikcamera::FLOAT,"AcquisitionFrameRate",s);
			}
		}
		if(config.exposure_time!=0)
		{
			s.eValue=config.exposure_time<0?2:0;
			hikcam.set_value(hikcamera::ENUM,"ExposureAuto",s);
			if(config.exposure_time>0)
			{
				s.fValue=config.exposure_time;
				hikcam.set_value(hikcamera::FLOAT,"ExposureTime",s);
			}
		}
		if(config.gain!=0)
		{
			s.eValue=config.gain<0?2:0;
			hikcam.set_value(hikcamera::ENUM,"GainAuto",s);
			if(config.gain>0)
			{
				s.fValue=config.gain;
				hikcam.set_value(hikcamera::FLOAT,"Gain",s);
			}
		}
	}
	return std::make_pair(width,height);
}
int main(int argc, char* argv[])
{
	int ret;
	//Part 1 Parse config
	struct model_config model_config;
	struct video_config video_config;
	char* car_engine;
	char* armor_engine;
	//TODO: Parse commandline input
	//TODO: Read JSON Config files (and save if not found)
	//Part 2 Convert and prepare model (See convert.h)
	if((ret=convert(model_config,threads,&car_engine,&armor_engine))>=0)
		return ret;
	std::cout<<"Config: "<<std::endl<<" Video type: "<<static_cast<unsigned>(video_config.video_type);
	if(video_config.video_type==0)
		std::cout<<" Video file: "<<video_config.video_file;
	else
		std::cout<<" Video device: "<<static_cast<unsigned>(video_config.video_device);
	std::cout<<" FPS: "<<video_config.fps<<" Exposure time: "<<video_config.exposure_time<<" Gain: "<<video_config.gain<<std::endl;
	//Part 3.1 Open video
	cv::VideoCapture cap;
	hikcamera::camera hikcam;
	switch(video_config.video_type)
	{
		case 0:
			cap.open(video_config.video_file);
			break;
		case 1:
			cap.open(video_config.video_device);
			break;
		case 2:
			hikcam=init_hikcamera(video_config.video_device);
			if(hikcamera::camera::last_error!=MV_OK)
			{
				std::cerr<<"Camera init failed! ["<<hikcamera::camera::last_error<<"]"<<std::endl;
				return 1;
			}
			while(hikcam.open()!=MV_OK)
			{
				std::cerr<<"Cannot open camera, please close other programs ["<<hikcamera::camera::last_error<<"]"<<std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			std::cerr<<"Video handle: "<<hikcam.get_handle()<<std::endl;
			break;
		default:
			std::cerr<<"Wrong video type "<<static_cast<unsigned>(video_config.video_type)<<std::endl;
			return -1;
	}
	if(!cap.isOpened()&&video_config.video_type!=2)
	{
		std::cerr<<"No video handle, check camera connection or filename"<<std::endl;
		delete[] car_engine;
		delete[] armor_engine;
		return 1;
	}
	//Part 3.2 Apply config
	std::pair<int,int> image_arg=apply_video_config(video_config,cap,hikcam);
	//Part 4 Read image, infer & Show (see infer.cpp)
	ret=run(car_engine,armor_engine,image_arg,cap,hikcam,video_config.video_device==0);
	delete[] car_engine;
	delete[] armor_engine;
	return ret;
}
