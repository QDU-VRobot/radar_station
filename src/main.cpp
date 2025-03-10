#include "common.h"
#include "hikcamera.h"
#include "serial.h"
#include "convert.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <utility>
#include <string>
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
struct model_config
{
	uint32_t builder_flag; //See NvInfer.h
	const std::string car_file; //ONNX/engine path
	const std::string armor_file; //ONNX/engine path
	const char* save_ext; //.engine
	int64_t tensor_width;
	int64_t tensor_height;
	int64_t batch_size;
};
struct video_config
{
	uint8_t video_type; //0-Video file; 1-Generic camera; 2-3rd party camera (hikcamera)
	const std::string video_file;
	uint8_t video_device; //Device number
	float fps; //FPS, -1=auto, 0=keep original
	float exposure_time; //Exposure time, -1=auto, 0=keep original
	float gain; //Gain, -1=auto
	const std::string record_file;
	const std::string fourcc; //-1=interactive
	int quality; //-1=keep original
};
static const char* name[2]={"Video","Field"};
static inline int init_hikcamera(hikcamera::camera& hikcam, uint8_t device_num)
{
	hikcamera::CAMERA_INFO_LIST devlist=hikcamera::camera::enum_cameras(); //USB cameras must be enumerated before opening
	if(devlist.nDeviceNum<=device_num)
		throw std::runtime_error("Camera not found, check camera connection");
	std::cerr<<"Using camera "<<static_cast<unsigned>(device_num)<<std::endl;
	return hikcam.open(devlist.pDeviceInfo[device_num]);
}
uint8_t mode; //Bit 0: Use car model; Bit 1: Use armor model (car model must be used); Bit 2: Display video; Bit 3: Enable serial port; All 0: Generate only
size_t threads; //For car_infer
std::chrono::milliseconds wait_timeout; //Timeout for inferring to finish per cycle
std::vector<std::vector<uint8_t>> robot_classes{{1,2,3,4,5,7},{101,102,103,104,105,107}}; //Default is their IDs, configurable
static int convert(const struct model_config& config, size_t threads, const char** car_engine_name=0, const char** armor_engine_name=0)
{
	char* car_engine=new char[getsavename(0,config.car_file.c_str(),config.save_ext)+1];
	getsavename(car_engine,config.car_file.c_str(),config.save_ext);
	char* armor_engine=new char[getsavename(0,config.armor_file.c_str(),config.save_ext)+1];
	getsavename(armor_engine,config.armor_file.c_str(),config.save_ext);
	if(mode==0||!file_readable(car_engine))
	{
		std::cerr<<"Generating car engine..."<<std::endl;
		if(!file_readable(config.car_file.c_str())||!convert_onnx(config.car_file.c_str(),car_engine,config.builder_flag,threads,true,config.tensor_width,config.tensor_height,config.batch_size))
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
		if(!file_readable(config.armor_file.c_str())||!convert_onnx(config.armor_file.c_str(),armor_engine,config.builder_flag,threads,true,config.tensor_width,config.tensor_height,config.batch_size))
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
static std::pair<const std::pair<int,int>,float> apply_video_config(const struct video_config& config, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec)
{
	int width,height;
	float fps;
	if(cap.isOpened())
	{
		if(config.fps>0)
			cap.set(cv::CAP_PROP_FPS,config.fps);
		if(config.exposure_time!=0)
			cap.set(cv::CAP_PROP_AUTO_EXPOSURE,config.exposure_time<0);
		if(config.exposure_time>0)
			cap.set(cv::CAP_PROP_EXPOSURE,config.exposure_time);
		if(config.gain>0)
			cap.set(cv::CAP_PROP_GAIN,config.gain);
		width=static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
		height=static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
		fps=cap.get(cv::CAP_PROP_FPS);
	}
	if(hikcam.isopened())
	{
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
		union hikcamera::gettype g;
		hikcam.get_value(hikcamera::INT,"Width",&g);
		width=g.stIntValue.nCurValue;
		hikcam.get_value(hikcamera::INT,"Height",&g);
		height=g.stIntValue.nCurValue;
		hikcam.get_value(hikcamera::FLOAT,"ResultingFrameRate",&g);
		fps=g.stFloatValue.fCurValue;
	}
	if(config.record_file.size())
	{
		std::vector<int> rec_config;
		if(config.quality>=0)
			rec_config={cv::VIDEOWRITER_PROP_QUALITY,config.quality};
		if(config.fourcc[0]>0)
			rec.open(config.record_file.c_str(),cv::VideoWriter::fourcc(config.fourcc[0],config.fourcc[1],config.fourcc[2],config.fourcc[3]),(mode&48)==48?fps*2:fps,cv::Size(width,height),rec_config);
		else
			rec.open(config.record_file.c_str(),config.fourcc[0],(mode&48)==48?fps*2:fps,cv::Size(width,height),rec_config); //Save pictures?
	}
	return std::make_pair(std::make_pair(width,height),fps);
}
static void mark(int event, int x, int y, int flags, void *userdata)
{
	static cv::Mat marked_img[2];
	std::pair<std::pair<uint8_t&,uint8_t>,std::pair<const cv::Mat&,cv::Point2f(&)[4]>>& arg=*(reinterpret_cast<std::pair<std::pair<uint8_t&,uint8_t>,std::pair<const cv::Mat&,cv::Point2f(&)[4]>>*>(userdata));
	const uint8_t state=arg.first.first>>arg.first.second*3;
	if(marked_img[arg.first.second].empty())
		arg.second.first.copyTo(marked_img[arg.first.second]);
	switch(event)
	{
		case -1:
			break;
		case cv::EVENT_LBUTTONDOWN:
			//Mark
			if(!(state&4))
			{
				arg.second.second[state&3].x=x,arg.second.second[state&3].y=y;
				cv::circle(marked_img[arg.first.second],arg.second.second[state&3],2,cv::Scalar(0,255,0),cv::LineTypes::FILLED);
				arg.first.first+=1<<arg.first.second*3;
			}
			break;
		case cv::EVENT_RBUTTONDOWN:
			//Line & finish
			if((state&7)==4)
			{
				for(size_t i=0;i<4;++i)
					cv::line(marked_img[arg.first.second],arg.second.second[i],arg.second.second[i+1&3],cv::Scalar(0,255,0),2);
				arg.first.first|=7<<arg.first.second*3;
				if(arg.first.first==077)
					std::cout<<"Press any other key to finish"<<std::endl;
			}
			break;
		case cv::EVENT_MBUTTONDOWN:
			//Clear
			arg.first.first&=~(7<<arg.first.second*3);
			marked_img[arg.first.second].release();
			cv::imshow(name[arg.first.second],arg.second.first);
		default:
			return;
	}
	cv::imshow(name[arg.first.second],marked_img[arg.first.second]);
}
static cv::Mat mark_points(const cv::Mat& img, const cv::Mat& minimap, float scale)
{
	cv::Point2f points[2][4];
	uint8_t state=0,key=0; //State bit 0-2: Video marked points counter; bit 3-5: Minimap marked points counter
	cv::namedWindow(name[0]),cv::namedWindow(name[1]);
	std::pair<std::pair<uint8_t&,uint8_t>,std::pair<const cv::Mat&,cv::Point2f(&)[4]>> video_arg=std::make_pair(std::make_pair(std::ref(state),0),std::make_pair(std::ref(img),std::ref(points[0]))),image_arg=std::make_pair(std::make_pair(std::ref(state),1),std::make_pair(std::ref(minimap),std::ref(points[1])));
	cv::setMouseCallback(name[0],mark,&video_arg),cv::setMouseCallback(name[1],mark,&image_arg);
	std::cout<<"Left click on the window anticlockwise to mark points; Right click to confirm; Middle click to reset\nPress 'c' for CLI input"<<std::endl;
	do
	{
		mark(-1,0,0,0,&video_arg),mark(-1,0,0,0,&image_arg); //Manually trigger mouse event to show image
		key=cv::waitKey();
	}
	while(state!=077&&key!=27&&key!='c'&&key!='C');
	cv::destroyAllWindows();
	if(key==27) //ESC
		return cv::Mat();
	if(key=='c'||key=='C')
		for(size_t i=0;i<2;++i)
		{
			std::cout<<"Input 4 points' x and y corresponding to the "<<(i?"map":"video")<<": ";
			for(size_t j=0;j<4;++j)
				if(!(std::cin>>points[i][j].x>>points[i][j].y))
					return cv::Mat();
		}
	else
		for(size_t j=0;j<4;++j)
			points[1][j]/=scale; //Scale to field position
	return cv::getPerspectiveTransform(points[0],points[1]);
}
int main(int argc, char** argv)
{
	int ret;
	//Part 1 Parse config
	cv::CommandLineParser args(argc,argv,"{help ?||Show this help}{C|camera.yml|Parameter file (*.XML, *.YAML or *.JSON)}{t|4|Threads}{m|63|Mode config (Bit 0: Use car model; Bit 1: Use armor model (car model must be used); Bit 2: Display video; Bit 3: Enable serial port; Bit 4: Record original video; Bit 5: Record processed video; All 0: Generate engine only)}{w|2|Wait timeout (ms) per cycle}{I|Field.jpg|Field image (black if not set)}{W|28|Field width}{H|15|Field height}{S|40|Field image scale}{s|/dev/ttyS0|Serial device}"
	"{B|1033|Builder flags (See NvInfer.h for detail)}{c|models/car.onnx|Car model (ONNX format with plugins)}{a|models/armor.onnx|Armor model (ONNX format with plugins)}{n||No conversion, assume model files are already in TensorRT format}{tensor-width|640|Tensor width}{tensor-height|640|Tensor height}{b|6|Batch size (maximum is doubled)}"
	"{V|2|Video type (0=Video file, 1=Generic camera (cv::VideoCapture), 2=hikcamera (MVS))}{f|-1|FPS (-1=auto, 0=keep unchanged)}{e|-1|Exposure time (-1=auto, 0=keep unchanged)}{g|-1|Gain (-1=auto)}{R||Record output file}{fourcc|avc1|Fourcc code}{q|-1|Quality of recorded video (-1=keep default)}{@file|0|Video device ID or video file}");
	args.about("Detect cars and armors and (optionally) send their position to the serial port");
	if(args.has("help"))
	{
		args.printMessage();
		return 0;
	}
	mode=args.get<unsigned>("m"),threads=args.get<size_t>("t"),wait_timeout=std::chrono::milliseconds(args.get<unsigned>("w"));
	struct model_config model_config={args.get<uint32_t>("B"),args.get<std::string>("c"),args.get<std::string>("a"),args.has("n")?0:".engine",args.get<int>("tensor-width"),args.get<int>("tensor-height"),args.get<int>("b")};
	struct video_config video_config={args.get<unsigned>("V"),args.get<std::string>("@file"),args.get<unsigned>("V")!=0?args.get<unsigned>("@file"):0,args.get<float>("f"),args.get<float>("e"),args.get<float>("g"),args.get<std::string>("R"),args.get<std::string>("fourcc"),args.get<int>("q")};
	cv::Mat trans_matrix,cam_matrix,dist_coeffs,example_img;
	if(args.has("C"))
	{
		cv::FileStorage config(args.get<std::string>("C"),cv::FileStorage::Mode::READ);
		if(config.isOpened())
			config["camera_matrix"]>>cam_matrix,config["distortion_coefficients"]>>dist_coeffs,config["perspective_matrix"]>>trans_matrix,config["robot_classes"]>>robot_classes;
		else
			std::cerr<<"Failed to open "<<args.get<std::string>("C")<<", ignoring"<<std::endl;
	}
	if(args.has("I"))
	{
		example_img=cv::imread(args.get<std::string>("I"));
		cv::resize(example_img,example_img,cv::Size(args.get<unsigned>("W")*args.get<float>("S"),args.get<unsigned>("H")*args.get<float>("S")));
	}
	else if((mode&5)==5)
		example_img=cv::Mat::zeros(args.get<unsigned>("H")*args.get<float>("S"),args.get<unsigned>("W")*args.get<float>("S"),CV_8UC3);
	if(!args.check())
	{
		args.printErrors();
		return -1;
	}
	const char* car_engine;
	const char* armor_engine;
	std::cout<<"Global config: "<<std::endl<<"Mode: "<<static_cast<unsigned>(mode)<<" Threads: "<<threads<<" Timeout: "<<wait_timeout.count()<<"ms"<<std::endl;
	//Part 2 Convert and prepare model (See convert.h)
	std::cout<<"Model config: "<<std::endl<<"Car file: "<<model_config.car_file<<" Armor file: "<<model_config.armor_file<<std::endl;
	if(model_config.save_ext)
	{
		if((ret=convert(model_config,threads,&car_engine,&armor_engine))>=0)
			return ret;
	}
	std::cout<<"Video config: "<<std::endl<<"Video type: "<<static_cast<unsigned>(video_config.video_type);
	if(video_config.video_type==0)
		std::cout<<" Video file: "<<video_config.video_file.c_str();
	else
		std::cout<<" Video device: "<<static_cast<unsigned>(video_config.video_device);
	std::cout<<" FPS: "<<video_config.fps<<" Exposure time: "<<video_config.exposure_time<<" Gain: "<<video_config.gain<<std::endl;
	std::cout<<"Camera matrix:\n"<<cam_matrix<<"\nDistortion coefficients:\n"<<dist_coeffs<<std::endl;
	//Part 3.1 Open serial port
	serial::terminal sport;
	//Part 3.2 Set serial config
	if((mode&8)==8)
		if(open_serial(sport,args.get<std::string>("s").c_str()))
			std::thread(receive,&sport,args.get<std::string>("s")).detach();
		else
		{
			std::cerr<<"Serial port is not available"<<std::endl;
			mode&=~8;
		}
	//Part 4.1 Open video
	cv::VideoCapture cap;
	hikcamera::camera hikcam;
	switch(video_config.video_type)
	{
		case 0:
			cap.open(video_config.video_file.c_str());
			break;
		case 1:
			cap.open(video_config.video_device);
			break;
		case 2:
			if(init_hikcamera(hikcam,video_config.video_device)!=hikcamera::OK&&!hikcam.get_handle())
			{
				std::cerr<<"Camera init failed! ["<<hikcamera::camera::last_error<<"]"<<std::endl;
				return 1;
			}
			while(!hikcam.isopened()&&hikcam.open()!=hikcamera::OK)
			{
				std::cerr<<"Cannot open camera ["<<hikcamera::camera::last_error<<"]"<<std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			std::cerr<<"Video handle: "<<hikcam.get_handle()<<std::endl;
			break;
		default:
			std::cerr<<"Wrong video type "<<static_cast<unsigned>(video_config.video_type)<<std::endl;
			return -1;
	}
	if(!cap.isOpened()&&video_config.video_type!=2)
		std::cerr<<"No video handle, check camera connection or filename"<<std::endl;
	else
	{
		cv::VideoWriter rec;
		//Part 4.2 Apply config
		std::pair<std::pair<int,int>,float> image_arg=apply_video_config(video_config,cap,hikcam,rec);
		//Part 4.3 Mark points (if not done)
		if(trans_matrix.empty()&&(mode&5)==5)
		{
			cv::Mat img;
			if(cap.isOpened())
				cap.read(img);
			else if(hikcam.isopened())
			{
				hikcamera::FRAME frame;
				if(hikcam.start()!=hikcamera::OK||hikcam.grab_frame(&frame,-1)!=hikcamera::OK)
				{
					std::cerr<<"An error occurred while starting video ["<<hikcamera::camera::last_error<<"]"<<std::endl;
					return 2;
				}
				img=cv::Mat(frame.stFrameInfo.nHeight,frame.stFrameInfo.nWidth,CV_8UC1,frame.pBufAddr);
				if(!img.empty())
					cv::cvtColor(img,img,cv::COLOR_BayerBG2BGR);
				hikcam.free_frame(&frame);
				hikcam.stop();
			}
			if(!img.empty())
			{
				if(!cam_matrix.empty())
				{
					cv::Mat tmp;
					cv::undistort(img,tmp,cam_matrix,dist_coeffs);
					img=std::move(tmp);
				}
				std::cerr<<"Marking field points"<<std::endl;
				if(!(trans_matrix=mark_points(img,example_img,args.get<float>("S"))).empty())
				{
					if(args.has("C"))
					{
						cv::FileStorage config(args.get<std::string>("C"),cv::FileStorage::Mode::APPEND);
						if(config.isOpened())
						{
							config<<"perspective_matrix"<<trans_matrix;
							std::cout<<"Saved to "<<args.get<std::string>("C")<<std::endl;
						}
					}
				}
				else std::cerr<<"Marking cancelled"<<std::endl;
			}
			else std::cerr<<"Image is empty"<<std::endl;
		}
		std::cout<<"Perspective matrix:\n"<<trans_matrix<<std::endl;
		//Part 5 Read image, infer & Show (see infer.cpp)
		ret=run(model_config.save_ext?car_engine:model_config.car_file.c_str(),model_config.save_ext?armor_engine:model_config.armor_file.c_str(),image_arg,cap,hikcam,rec,sport,video_config.video_device==0,trans_matrix,cam_matrix,dist_coeffs,example_img,args.get<float>("S"));
	}
	if(model_config.save_ext)
	{
		delete[] car_engine;
		delete[] armor_engine;
	}
	return ret;
}
