#include "common.h"
#include "hikcamera.h"
#include "serial.h"
#include "convert.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <tuple>
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
serial::terminal sport;
std::vector<std::vector<uint8_t>> robot_classes{{0,1,2,3,4,5},{6,7,8,9,10,11}}; //Configurable
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
static std::tuple<int,int,float> apply_video_config(const struct video_config& config, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec)
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
	return std::make_tuple(width,height,fps);
}
static void adjust_height(int pos, void *userdata)
{
	*(reinterpret_cast<unsigned int*>(userdata))=pos;
}
struct mark_flag
{
	uint8_t type:1;
	uint8_t step:2; //0-finished; 1-mark points; 2-mark contour
};
static void mark(int event, int x, int y, int flags, void *userdata)
{
	static cv::Mat marked_img[2];
	std::tuple<struct mark_flag&,const cv::Mat&,void*,const unsigned int&,void*>& arg=*(reinterpret_cast<std::tuple<struct mark_flag&,const cv::Mat&,void*,const unsigned int&,void*>*>(userdata));
	if(marked_img[std::get<0>(arg).type].empty())
		std::get<1>(arg).copyTo(marked_img[std::get<0>(arg).type]);
	switch(event)
	{
		case -1:
			break;
		case cv::EVENT_LBUTTONDOWN:
			//Mark
		{
			cv::Point2f point(x,y);
			switch(std::get<0>(arg).step)
			{
				case 1:
					if(std::get<0>(arg).type==0)
						reinterpret_cast<std::vector<cv::Point2f>*>(std::get<2>(arg))->push_back(point);
					else if(std::get<0>(arg).type==1)
						reinterpret_cast<std::vector<cv::Point3f>*>(std::get<2>(arg))->emplace_back(x,y,std::get<3>(arg)*0.01);
				cv::circle(marked_img[std::get<0>(arg).type],point,2,cv::Scalar(0,255,0),cv::LineTypes::FILLED);
				break;
				case 0:
					if(std::get<0>(arg).type==0)
					{
						std::vector<std::pair<std::vector<cv::Point2f>,float>>* contours=reinterpret_cast<std::vector<std::pair<std::vector<cv::Point2f>,float>>*>(std::get<4>(arg));
						contours->resize(contours->size()+1);
						std::get<0>(arg).step=2;
					}
				case 2:
					if(std::get<0>(arg).type==0)
					{
						reinterpret_cast<std::vector<std::pair<std::vector<cv::Point2f>,float>>*>(std::get<4>(arg))->back().first.push_back(point);
						cv::circle(marked_img[std::get<0>(arg).type],point,2,cv::Scalar(0,255,255),cv::LineTypes::FILLED);
					}
					break;
			}
		}
			break;
		case cv::EVENT_RBUTTONDOWN:
			//Line & finish
			switch(std::get<0>(arg).step)
			{
				case 1:
					if(std::get<0>(arg).type==0)
					{
						std::vector<cv::Point2f>* points=reinterpret_cast<std::vector<cv::Point2f>*>(std::get<2>(arg));
						if(points->size()>=4)
						{
							for(auto i=points->begin();i!=points->end();++i)
								cv::line(marked_img[std::get<0>(arg).type],*i,i+1==points->end()?*points->begin():*(i+1),cv::Scalar(0,255,0),2);
							std::cout<<"Marked "<<points->size()<<" points on "<<name[std::get<0>(arg).type]<<std::endl;
							std::get<0>(arg).step=0;
							std::cout<<"Now mark a contour for different heights"<<std::endl;
						}
					}
					else if(std::get<0>(arg).type==1)
					{
						std::vector<cv::Point3f>* points=reinterpret_cast<std::vector<cv::Point3f>*>(std::get<2>(arg));
						if(points->size()>=4)
						{
							for(auto i=points->begin();i!=points->end();++i)
								cv::line(marked_img[std::get<0>(arg).type],cv::Point2f(i->x,i->y),i+1==points->end()?cv::Point2f(points->begin()->x,points->begin()->y):cv::Point2f((i+1)->x,(i+1)->y),cv::Scalar(0,255,0),2);
							std::cout<<"Marked "<<points->size()<<" points on "<<name[std::get<0>(arg).type]<<std::endl;
							std::get<0>(arg).step=0;
						}
					}
					break;
				case 2:
					if(std::get<0>(arg).type==0)
					{
						std::vector<std::pair<std::vector<cv::Point2f>,float>>* contours=reinterpret_cast<std::vector<std::pair<std::vector<cv::Point2f>,float>>*>(std::get<4>(arg));
						if(contours->back().first.size()>=3)
						{
							for(auto i=contours->back().first.begin();i!=contours->back().first.end();++i)
								cv::line(marked_img[std::get<0>(arg).type],*i,i+1==contours->back().first.end()?*contours->back().first.begin():*(i+1),cv::Scalar(0,255,255),2);
							contours->back().second=std::get<3>(arg)*0.01;
							std::cout<<"Marked a contour with "<<contours->back().first.size()<<" points on "<<name[std::get<0>(arg).type]<<std::endl;
							std::get<0>(arg).step=0;
						}
					}
					break;
			}
			break;
		case cv::EVENT_MBUTTONDOWN:
			//Clear
			std::get<0>(arg).step=1;
			if(std::get<0>(arg).type==0)
				reinterpret_cast<std::vector<cv::Point2f>*>(std::get<2>(arg))->resize(0),reinterpret_cast<std::vector<std::pair<std::vector<cv::Point2f>,float>>*>(std::get<4>(arg))->resize(0);
			else if(std::get<0>(arg).type==1)
				reinterpret_cast<std::vector<cv::Point3f>*>(std::get<2>(arg))->resize(0);
			marked_img[std::get<0>(arg).type].release();
			cv::imshow(name[std::get<0>(arg).type],std::get<1>(arg));
		default:
			return;
	}
	cv::imshow(name[std::get<0>(arg).type],marked_img[std::get<0>(arg).type]);
}
static std::tuple<cv::Mat,cv::Mat,std::vector<std::pair<std::vector<cv::Point2f>,float>>> mark_points(const cv::Mat& img, const cv::Mat& minimap, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, float scale)
{
	std::vector<cv::Point2f> video_points;
	std::vector<cv::Point3f> map_points;
	std::vector<std::pair<std::vector<cv::Point2f>,float>> contours;
	uint8_t key=0;
	struct mark_flag state[2]={{0,1},{1,1}};
	unsigned int z=0,contour_height=20;
	cv::namedWindow(name[0]),cv::namedWindow(name[1]);
	std::tuple<struct mark_flag&,const cv::Mat&,void*,const unsigned int&,void*> video_arg=std::make_tuple(std::ref(state[0]),std::ref(img),&video_points,std::ref(contour_height),&contours),image_arg=std::make_tuple(std::ref(state[1]),std::ref(minimap),&map_points,std::ref(z),(void*)0);
	cv::createTrackbar("Height of contour (cm)",name[0],reinterpret_cast<int*>(&contour_height),100,adjust_height,&contour_height);
	cv::createTrackbar("Height of next point (cm)",name[1],reinterpret_cast<int*>(&z),100,adjust_height,&z);
	cv::setMouseCallback(name[0],mark,&video_arg),cv::setMouseCallback(name[1],mark,&image_arg);
	std::cout<<"Left click on the window anticlockwise to mark points; Right click to confirm; Middle click to reset\nPress 'c' for CLI input"<<std::endl;
	do
	{
		mark(-1,0,0,0,&video_arg),mark(-1,0,0,0,&image_arg); //Manually trigger mouse event to show image
		key=cv::waitKey();
	}
	while((state[0].step!=0||state[1].step!=0||map_points.size()!=video_points.size())&&key!=27&&key!='c'&&key!='C');
	cv::destroyAllWindows();
	cv::Mat rvec,tvec;
	if(key!=27) //ESC
	{
		if(key=='c'||key=='C')
		{
			float x,y,z;
			char c;
			std::cout<<"Input points' x, y and z corresponding to the map within a line: ";
			do
			{
				if(!(std::cin>>x>>y>>z))
					return std::make_tuple(rvec,tvec,contours);
				map_points.emplace_back(x,y,z);
			}
			while(std::cin.get(c)&&(map_points.size()<4||c!='\n'));
			std::cout<<"Input points' x and y corresponding to the video within a line: ";
			do
			{
				if(!(std::cin>>x>>y))
					return std::make_tuple(rvec,tvec,contours);
				video_points.emplace_back(x,y);
			}
			while(std::cin.get(c)&&(video_points.size()<4||c!='\n'));
			std::cout<<"Creating contours: "<<std::endl;
			do
			{
				std::vector<cv::Point2f> points;
				std::cout<<"Input points' x and y in the video within a line: "<<std::endl;
				do
				{
					if(std::cin>>x>>y)
						points.emplace_back(x,y);
					else
					{
						points.resize(0);
						break;
					}
				}
				while(std::cin.get(c)&&(points.size()<3||c!='\n'));
				if(points.size())
				{
					std::cout<<"Input height (then input other characters to finish): "<<std::endl;
					if(std::cin>>z)
						contours.emplace_back(std::move(points),z);
				}
				else
					break;
			}
			while(std::cin.get(c)&&c=='\n');
		}
		else
			for(auto i=map_points.begin();i!=map_points.end();++i)
				i->x/=scale,i->y=(minimap.rows-i->y)/scale; //Scale to field position
		cv::solvePnP(map_points,video_points,cam_matrix.empty()?calculateCameraMatrix(img.size(),acos(0.5)):cam_matrix,dist_coeffs,rvec,tvec);
	}
	return std::make_tuple(rvec,tvec,contours);
}
int main(int argc, char** argv)
{
	int ret;
	//Part 1 Parse config
	cv::CommandLineParser args(argc,argv,"{help ?||Show this help}{C|config.yml|Parameter file (*.XML, *.YAML or *.JSON)}{t|4|Threads}{m|63|Mode config (Bit 0: Use car model; Bit 1: Use armor model (car model must be used); Bit 2: Display video; Bit 3: Enable serial port; Bit 4: Record original video; Bit 5: Record processed video; All 0: Generate engine only)}{w|2|Wait timeout (ms) per cycle}{I|Field.jpg|Field image (black if not set)}{W|28|Field width}{H|15|Field height}{S|40|Field image scale}{s|/dev/ttyS0|Serial device}"
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
	cv::Mat cam_matrix,dist_coeffs,example_img;
	std::tuple<cv::Mat,cv::Mat,std::vector<std::pair<std::vector<cv::Point2f>,float>>> pose_points;
	if(args.has("C"))
	{
		cv::FileStorage config(args.get<std::string>("C"),cv::FileStorage::Mode::READ);
		if(config.isOpened())
		{
			std::vector<std::vector<cv::Point2f>> points;
			std::vector<float> heights;
			config["camera_matrix"]>>cam_matrix,config["distortion_coefficients"]>>dist_coeffs;
			config["rvec"]>>std::get<0>(pose_points),config["tvec"]>>std::get<1>(pose_points);
			config["contours"]>>points,config["contour_heights"]>>heights;
			config["robot_classes"]>>robot_classes;
			heights.resize(points.size());
			for(size_t i=0;i<points.size();++i)
				std::get<2>(pose_points).emplace_back(std::move(points[i]),heights[i]);
		}
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
	//Part 3 Open serial port
	if((mode&8)==8)
		if(open_serial(sport,args.get<std::string>("s").c_str()))
			std::thread(receive,std::ref(sport)).detach();
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
		std::tuple<int,int,float> image_arg=apply_video_config(video_config,cap,hikcam,rec);
		//Part 4.3 Mark points (if not done)
		if((std::get<0>(pose_points).empty()||std::get<1>(pose_points).empty())&&(mode&5)==5)
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
				std::cerr<<"Marking field points"<<std::endl;
				pose_points=mark_points(img,example_img,cam_matrix,dist_coeffs,args.get<float>("S"));
				if(args.has("C"))
				{
					cv::FileStorage config(args.get<std::string>("C"),cv::FileStorage::Mode::APPEND);
					if(config.isOpened())
					{
						if(!std::get<0>(pose_points).empty()&&!std::get<1>(pose_points).empty())
						{
							config<<"rvec"<<std::get<0>(pose_points)<<"tvec"<<std::get<1>(pose_points);
							std::cout<<"Saved to "<<args.get<std::string>("C")<<std::endl;
						}
						if(std::get<2>(pose_points).size())
						{
							std::vector<std::vector<cv::Point2f>> points;
							std::vector<float> heights;
							for(auto i=std::get<2>(pose_points).begin();i!=std::get<2>(pose_points).end();++i)
								points.push_back(i->first),heights.push_back(i->second);
							config<<"contours"<<points<<"contour_heights"<<heights;
							std::cout<<"Saved to "<<args.get<std::string>("C")<<std::endl;
						}
					}
				}
				else std::cerr<<"Config not saved!"<<std::endl;
			}
			else std::cerr<<"Image is empty"<<std::endl;
		}
		std::cout<<"rvec:\n"<<std::get<0>(pose_points)<<"\ntvec:\n"<<std::get<1>(pose_points)<<"\ncontours count: "<<std::get<2>(pose_points).size()<<std::endl;
		//Part 5 Read image, infer & Show (see infer.cpp)
		ret=run(model_config.save_ext?car_engine:model_config.car_file.c_str(),model_config.save_ext?armor_engine:model_config.armor_file.c_str(),image_arg,cap,hikcam,rec,sport,video_config.video_device==0,pose_points,cam_matrix,dist_coeffs,example_img,args.get<float>("S"));
	}
	if(model_config.save_ext)
	{
		delete[] car_engine;
		delete[] armor_engine;
	}
	return ret;
}
