#include "common.h"
#include "hikcamera.h"
#include "inferer.h"
#include "messenger.h"
#include <signal.h>
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
#include "infer/trtyolo.hpp"
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
static const char* name[2]={"Video","Field"}; //Just for windows' name
static inline int init_hikcamera(camera::hikcam& cam, uint8_t device_num)
{
	camera::hikcam::CAMERA_INFO_LIST devlist=camera::hikcam::enum_cameras(); //USB cameras must be enumerated before opening
	if(devlist.nDeviceNum<=device_num)
		throw std::runtime_error("Camera not found, check camera connection");
	std::cerr<<"Using camera "<<static_cast<unsigned>(device_num)<<std::endl;
	return cam.open(devlist.pDeviceInfo[device_num]);
}
uint8_t mode; //See help output
size_t threads,rm::inferer::threads; //For car_infer;
std::chrono::milliseconds wait_timeout,rm::inferer::timeout; //Timeout for inferring to finish per cycle
std::vector<std::vector<uint8_t>> rm::msg::robot_classes={{0,1,2,3,254,5},{6,7,8,9,254,11}}; //{Blue{123467},Red{123467}}, Configurable
static bool running=true;
static void handle_exit(int type) //Exit safely or the recorded video will corrupt!
{
	std::cerr<<"Received signal "<<type<<", exiting"<<std::endl;
	running=false;
	signal(type,SIG_DFL); //Press again to force exit
}

static std::tuple<int,int,float> apply_video_config(const struct video_config& config, cv::VideoCapture& cap, camera::hikcam& cam, cv::VideoWriter& rec)
{
	int width,height;
	float fps=0;
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
	if(cam.isopened())
	{
		union camera::hikset s;
		s.eValue=0;
		cam.set(camera::ENUM,"TriggerMode",s);
		if(config.fps!=0)
		{
			s.bValue=config.fps>0;
			cam.set(camera::BOOL,"AcquisitionFrameRateEnable",s);
			if(config.fps>0)
			{
				s.fValue=config.fps;
				cam.set(camera::FLOAT,"AcquisitionFrameRate",s);
			}
		}
		if(config.exposure_time!=0)
		{
			s.eValue=config.exposure_time<0?2:0;
			cam.set(camera::ENUM,"ExposureAuto",s);
			if(config.exposure_time>0)
			{
				s.fValue=config.exposure_time;
				cam.set(camera::FLOAT,"ExposureTime",s);
			}
		}
		if(config.gain!=0)
		{
			s.eValue=config.gain<0?2:0;
			cam.set(camera::ENUM,"GainAuto",s);
			if(config.gain>0)
			{
				s.fValue=config.gain;
				cam.set(camera::FLOAT,"Gain",s);
			}
		}
		union camera::hikget g;
		g=cam.get(camera::INT,"Width");
		width=g.stIntValue.nCurValue;
		g=cam.get(camera::INT,"Height");
		height=g.stIntValue.nCurValue;
		g=cam.get(camera::FLOAT,"ResultingFrameRate");
		fps=g.stFloatValue.fCurValue;
	}
	if(config.record_file.size())
	{
		std::string name=config.record_file;
		if(config.record_file.back()=='/')
			name+=std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+".mp4"; //Use system time if filename is unknown
		std::vector<int> rec_config;
		if(config.quality>=0)
			rec_config={cv::VIDEOWRITER_PROP_QUALITY,config.quality};
		if(config.fourcc.size())
			rec.open(name.c_str(),cv::VideoWriter::fourcc(config.fourcc[0],config.fourcc[1],config.fourcc[2],config.fourcc[3]),(mode&48)==48?fps*2:fps,cv::Size(width,height),rec_config);
		else
			rec.open(name.c_str(),0,(mode&48)==48?fps*2:fps,cv::Size(width,height),rec_config); //Save pictures?
	}
	return std::make_tuple(width,height,fps);
}
static void adjust_height(int pos, void *userdata)
{
	*(reinterpret_cast<unsigned int*>(userdata))=pos;
}
struct mark_flag
{
	uint8_t type:1; //0-video; 1-minimap
	uint8_t step:2; //0-finished; 1-mark PnP points; 2-mark contour
};
static void mark(int event, int x, int y, int flags, void *userdata)
{
	static cv::Mat marked_img[2]; //Video, Minimap
	std::tuple<struct mark_flag&,const cv::Mat&,void*,const unsigned int&,void*>& arg=*(reinterpret_cast<std::tuple<struct mark_flag&,const cv::Mat&,void*,const unsigned int&,void*>*>(userdata));
	if(marked_img[std::get<0>(arg).type].empty()) //First call to this function
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
				case 1: //Mark PnP point
					if(std::get<0>(arg).type==0)
						reinterpret_cast<std::vector<cv::Point2f>*>(std::get<2>(arg))->push_back(point);
					else if(std::get<0>(arg).type==1)
						reinterpret_cast<std::vector<cv::Point3f>*>(std::get<2>(arg))->emplace_back(x,y,std::get<3>(arg)*0.01);
				cv::circle(marked_img[std::get<0>(arg).type],point,2,cv::Scalar(0,255,0),cv::LineTypes::FILLED);
				break;
				case 0: //Start another contour marking
					if(std::get<0>(arg).type==0)
					{
						std::vector<std::pair<std::vector<cv::Point2f>,float>>* contours=reinterpret_cast<std::vector<std::pair<std::vector<cv::Point2f>,float>>*>(std::get<4>(arg));
						contours->resize(contours->size()+1);
						std::get<0>(arg).step=2;
					}
				case 2: //Add a contour point
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
			//Finish
			switch(std::get<0>(arg).step)
			{
				case 1: //Finish PnP marking and go to contour marking
					if(std::get<0>(arg).type==0)
					{
						std::vector<cv::Point2f>* points=reinterpret_cast<std::vector<cv::Point2f>*>(std::get<2>(arg));
						if(points->size()>=4)
						{
							for(auto i=points->begin();i!=points->end();++i)
								cv::line(marked_img[std::get<0>(arg).type],*i,i+1==points->end()?*points->begin():*(i+1),cv::Scalar(0,255,0),2); //Line around points
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
				case 2: //Finished a contour marking
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
			//Clear and reset step to PnP marking
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
	std::vector<std::pair<std::vector<cv::Point2f>,float>> contours; //Different height applied
	uint8_t key=0; //Keyboard input
	struct mark_flag state[2]={{0,1},{1,1}};
	unsigned int z=0,contour_height=20;
	cv::namedWindow(name[0]),cv::namedWindow(name[1]);
	std::tuple<struct mark_flag&,const cv::Mat&,void*,const unsigned int&,void*> video_arg=std::make_tuple(std::ref(state[0]),std::ref(img),&video_points,std::ref(contour_height),&contours),image_arg=std::make_tuple(std::ref(state[1]),std::ref(minimap),&map_points,std::ref(z),(void*)0);
	cv::createTrackbar("Height of contour (cm)",name[0],reinterpret_cast<int*>(&contour_height),100,adjust_height,&contour_height);
	cv::createTrackbar("Height of next point (cm)",name[1],reinterpret_cast<int*>(&z),100,adjust_height,&z);
	cv::setMouseCallback(name[0],mark,&video_arg),cv::setMouseCallback(name[1],mark,&image_arg);
	std::cout<<"Left click on the window in certain sequence to mark points; Right click to confirm; Middle click to reset\nPress 'c' for CLI input"<<std::endl;
	do
	{
		mark(-1,0,0,0,&video_arg),mark(-1,0,0,0,&image_arg); //Manually trigger mouse event to show image
		key=cv::waitKey();
	}
	while((state[0].step!=0||state[1].step!=0||map_points.size()!=video_points.size())&&key!=27&&key!='c'&&key!='C');
	cv::destroyAllWindows();
	cv::Mat rvec,tvec;
	if(key!=27) //ESC to discard markings
	{
		if(key=='c'||key=='C') //Enter CLI mode (without scale)
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
static char output(cv::Mat& img,const std::pair<const std::vector<STrack>, const std::map<int, std::pair<uint8_t,float>>>& result, const cv::Mat& m1, const cv::Mat& m2, const std::vector<std::pair<std::vector<cv::Point2f>,float>>& contours, cv::VideoWriter& rec, rm::msg& msger, cv::Mat&& example_img, float scale, unsigned short latency=0) //Display, send to referee or record
{
	std::vector<cv::Point2f> center_points; //Mark to the minimap and send to server
	if((mode&1)==1&&result.first.size()>0)
	{
		std::vector<uint8_t> ids;
		center_points.reserve(result.first.size());
		ids.reserve(result.first.size());
		for(auto it=result.first.begin();it!=result.first.end();++it)
		{
			cv::Point2f pos2d(it->tlbr[0]+it->tlwh[2]/2,it->tlbr[3]-it->tlwh[2]/6); //May not accurate
			float height=0;
			if(!m1.empty()&&!m2.empty())
			{
				for(auto i=contours.begin();i!=contours.end();++i)
					if(cv::pointPolygonTest(i->first,pos2d,false)>0) //In a contour, which means different height
					{
						height=i->second;
						break;
					}
				pos2d=backProjectPoint(cv::Point3d(pos2d.x,pos2d.y,height),m1,m2);
			}
			center_points.push_back(pos2d);
			std::cout<<it->track_id<<':'<<'['<<pos2d.x<<','<<pos2d.y<<','<<height<<']'<<';';
			if(result.second.count(it->track_id))
			{
				std::cout<<static_cast<unsigned>(result.second.at(it->track_id).first)<<','<<result.second.at(it->track_id).second;
				ids.push_back(result.second.at(it->track_id).first);
			}
			else
				ids.push_back(-1);
			std::cout<<std::endl;
		}
		if((mode&8)==8)
			msger.pos_from_camera=std::make_pair(center_points,ids);
	}
	if(!img.empty()&&((mode&4)==4||(mode&48)&&rec.isOpened()))
	{
		if((mode&16)==16&&rec.isOpened())
			rec.write(img);
		//Draw on output image
		cv::putText(img,std::string("Latency: ")+std::to_string(latency)+"ms",cv::Point(img.cols-255,30),0,1,cv::Scalar(0,255,0),2);
		if((mode&8)==8)
		{
			cv::putText(img,std::string("State: ")+std::to_string(msger.serial_info.game_progress),cv::Point(5,30),0,1,cv::Scalar(0,255,0),2);
			if(msger.serial_info.game_progress==5)
			{
				const char* winner="";
				switch(msger.serial_info.stage_remain_time)
				{
					case 0:
						winner="Draw";
						break;
					case 1:
						winner="Red";
						break;
					case 2:
						winner="Blue";
						break;
				}
				cv::putText(img,std::string("Winner: ")+winner,cv::Point(150,30),0,1,cv::Scalar(0,255,0),2);
			}
			else cv::putText(img,std::string("Timer: ")+std::to_string(msger.serial_info.stage_remain_time/60)+':'+std::to_string(msger.serial_info.stage_remain_time%60),cv::Point(150,30),0,1,cv::Scalar(0,255,0),2);
			if(msger.serial_info.robot_id)
				cv::putText(img,std::string("Team: ")+(msger.serial_info.robot_id/100?"Blue":"Red"),cv::Point(350,30),0,1,cv::Scalar(0,255,0),2);
			{
				std::string marked("Ally: ");
				for(uint8_t i=1;i<=1<<5;i<<=1)
					marked+=msger.serial_info.foe_mark&i?'1':'0';
				cv::putText(img,marked,cv::Point(img.cols-415,img.rows-40),0,1,cv::Scalar(0,255,0),2);
				marked="Foe: ";
				for(uint8_t i=1;i<=1<<5;i<<=1)
					marked+=msger.serial_info.ally_mark&i?'1':'0';
				cv::putText(img,marked,cv::Point(img.cols-420,img.rows-10),0,1,cv::Scalar(0,255,0),2);
			}
			cv::putText(img,std::string("HP: ")+std::to_string(msger.serial_info.ally_outpost_base_HP),cv::Point(5,img.rows-10),0,1,cv::Scalar(0,255,0),2);
			cv::putText(img,std::string("Encrypt: ")+std::to_string(msger.serial_info.encrypt_level)+(msger.password_retries>0?'*':' '),cv::Point(img.cols-185,img.rows-40),0,1,cv::Scalar(0,255,0),2);
			cv::putText(img,std::string("Damages: ")+std::to_string(msger.serial_info.vulnerability)+(msger.serial_info.vulnerating?'*':' '),cv::Point(img.cols-210,img.rows-10),0,1,cv::Scalar(0,255,0),2);
		}
		for(auto it=result.first.begin();it!=result.first.end();++it)
		{
			cv::rectangle(img,cv::Rect(it->tlwh[0],it->tlwh[1],it->tlwh[2],it->tlwh[3]),cv::Scalar(0,255,0),2);
			cv::putText(img,std::to_string(it->track_id)+':'+std::to_string(it->score),cv::Point(it->tlbr[0],it->tlbr[1]),1,1,cv::Scalar(0,255,0),1);
			if((mode&3)==3&&result.second.count(it->track_id))
				cv::putText(img,std::to_string(result.second.at(it->track_id).first)+':'+std::to_string(result.second.at(it->track_id).second),cv::Point(it->tlbr[0],it->tlbr[3]),1,1,cv::Scalar(0,255,0),1);
		}
		if((mode&32)==32&&rec.isOpened())
			rec.write(img);
		if((mode&4)==4)
		{
			if(!example_img.empty())
			{
				for(auto it=center_points.begin();it!=center_points.end();++it)
					cv::circle(example_img,m1.empty()||m2.empty()?cv::Point(it->x*example_img.cols/img.cols,it->y*example_img.rows/img.rows):cv::Point(it->x*scale,example_img.rows-it->y*scale),2,cv::Scalar(0,255,0),cv::LineTypes::FILLED);
				cv::imshow("Minimap",example_img);
			}
			cv::imshow("Radar",img);
			return cv::waitKey(1);
		}
	}
	return -1;
}
int run(rm::inferer& rainfer, cv::VideoCapture& cap, camera::hikcam& cam, cv::VideoWriter& rec, rm::msg& msger, const std::tuple<int,int,float>& image_arg, const std::tuple<cv::Mat,cv::Mat,std::vector<std::pair<std::vector<cv::Point2f>,float>>>& pose_points, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const cv::Mat& example_img, float scale)
{
	unsigned int err_counter=0;
	cv::Mat img,map,m1,m2;
	if(!cam_matrix.empty())
	{
		cv::Mat tmp;
		cv::initUndistortRectifyMap(cam_matrix,dist_coeffs,cv::Mat(),cv::Mat(),cv::Size(std::get<0>(image_arg),std::get<1>(image_arg)),CV_32FC2,map,tmp); //Undistort before backProjectPoint
	}
	if(!std::get<0>(pose_points).empty())
	{
		//Calculate matrices used by backProjectPoint
		cv::Mat rinv,cam_matrix_inv;
		cv::Rodrigues(std::get<0>(pose_points),rinv);
		cv::invert(rinv,rinv);
		cv::invert(cam_matrix.empty()?calculateCameraMatrix(cv::Size(std::get<0>(image_arg),std::get<1>(image_arg)),acos(0.5)):cam_matrix,cam_matrix_inv);
		m1=rinv*cam_matrix_inv,m2=rinv*std::get<1>(pose_points);
	}
	std::pair<vector<STrack>, std::map<int, std::pair<uint8_t,float>>> result;
	std::cout<<"Video starting"<<std::endl;
	if(signal(SIGINT,handle_exit)==SIG_ERR||signal(SIGHUP,handle_exit)==SIG_ERR||signal(SIGTERM,handle_exit)==SIG_ERR)
		std::cerr<<"Signal registration failed"<<std::endl;
	if(cap.isOpened())
	{
		do
			while(!(err_counter=!cap.read(img))&&running)
			{
				static std::chrono::steady_clock::time_point old_timestamp=std::chrono::steady_clock::now();
				std::chrono::steady_clock::time_point now=std::chrono::steady_clock::now();
				if(!cam_matrix.empty())
				{
					cv::Mat tmp;
					cv::remap(img,tmp,map,cv::Mat(),INTER_LINEAR); //Cannot undistort in-place
					img=std::move(tmp);
				}
				if((mode&1)==1)
				{
					trtyolo::DetectRes inferred=rainfer.infer(img);
					if(inferred.num>0)
						result=rainfer.track(inferred,std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp));
				}
				std::cerr<<"Cycle: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp).count()<<"ms"<<std::endl;
				if(output(img,result,m1,m2,std::get<2>(pose_points),rec,msger,example_img.clone(),scale,std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp).count())==27)
					running=false; //ESC to exit
				old_timestamp=now;
			}
			while(running&&(mode&64)==64&&cap.set(cv::CAP_PROP_POS_FRAMES,0));
			std::cout<<"Video stopped"<<std::endl;
	}
	else if(cam.isopened())
	{
		if(cam.start()!=camera::hikcam::OK)
		{
			std::cerr<<"An error occurred while starting video ["<<camera::hikcam::last_error<<"]"<<std::endl;
			if(cam.set(camera::COMMAND,"DeviceReset")==camera::hikcam::OK)
				std::cerr<<"Reset command has been sent, please retry later"<<std::endl;
			return 2;
		}
		camera::hikcam::FRAME frame;
		while(err_counter<MAX(2000/wait_timeout.count(),10000/std::get<2>(image_arg))&&running) //2s or 10 pictures
		{
			frame=cam.grab(wait_timeout.count());
			err_counter=camera::hikcam::last_error==camera::hikcam::OK?0:err_counter+1;
			static int64_t old_timestamp=frame.stFrameInfo.nHostTimeStamp;
			img=cv::Mat(frame.stFrameInfo.nHeight,frame.stFrameInfo.nWidth,CV_8UC1,frame.pBufAddr);
			if(!err_counter)
			{
				cv::Mat tmp;
				cv::cvtColor(img,tmp,cv::COLOR_BayerBG2RGB);
				if(!cam_matrix.empty())
					cv::remap(tmp,img,map,cv::Mat(),INTER_LINEAR);
				else
					img=std::move(tmp);
			}
			if((mode&1)==1)
			{
				trtyolo::DetectRes inferred=rainfer.infer(img);
				if(inferred.num>0)
					result=rainfer.track(inferred,std::chrono::milliseconds(frame.stFrameInfo.nHostTimeStamp-old_timestamp));
			}
			if(!err_counter)
			{
				std::cerr<<"Interval: "<<frame.stFrameInfo.nHostTimeStamp-old_timestamp<<"ms"<<std::endl;
				cv::cvtColor(img,img,cv::COLOR_RGB2BGR);
			}
			if(output(img,result,m1,m2,std::get<2>(pose_points),rec,msger,example_img.clone(),scale,frame.stFrameInfo.nHostTimeStamp-old_timestamp)==27)
				running=false; //ESC to exit
			if(!err_counter)
			{
				old_timestamp=frame.stFrameInfo.nHostTimeStamp;
				cam.free(frame);
			}
		}
		cam.stop();
		std::cout<<"Video stopped ["<<(err_counter=camera::hikcam::last_error)<<"]"<<std::endl;
	}
	cv::destroyAllWindows();
	return err_counter;
}
int main(int argc, char** argv)
{
	int ret;
	//Part 1 Parse config
	cv::CommandLineParser args(argc,argv,"{help ?||Show this help}{C|config.yaml|Parameter file (*.XML, *.YAML or *.JSON)}{M|camera.yaml|Camera config (generated by calibration)}{t|4|Threads}{m|31|Mode (Bit 0: Use car model; Bit 1: Use armor model (car model must be used); Bit 2: Display video; Bit 3: Enable serial port; Bit 4: Record original video; Bit 5: Record processed video; Bit 6: Loop input video; All 0: Generate engine only)}{w|2|Wait timeout (ms) per cycle}{I|Field.jpg|Field image (black if not set)}{W|28|Field width}{H|15|Field height}{S|40|Field image scale}{s|/dev/ttyS0|Serial device}"
	"{B|1033|Builder flags (See NvInfer.h for detail)}{c|models/car.onnx|Car model (ONNX format with plugins)}{a|models/armor.onnx|Armor model (ONNX format with plugins)}{n||No conversion, assume model files are already in TensorRT format}{tensor-width|640|Tensor width}{tensor-height|640|Tensor height}{b|-6|Batch size for armor inference (<0 means dynamic, where maximum is doubled)}"
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
			config["rvec"]>>std::get<0>(pose_points),config["tvec"]>>std::get<1>(pose_points);
			config["contours"]>>points,config["contour_heights"]>>heights;
			if(!config["robot_classes"].empty())
				config["robot_classes"]>>rm::msg::robot_classes;
			heights.resize(points.size());
			for(size_t i=0;i<points.size();++i)
				std::get<2>(pose_points).emplace_back(std::move(points[i]),heights[i]);
		}
		else
			std::cerr<<"Failed to open "<<args.get<std::string>("C")<<", ignoring"<<std::endl;
	}
	if(args.has("M"))
	{
		cv::FileStorage config(args.get<std::string>("M"),cv::FileStorage::Mode::READ);
		if(config.isOpened())
			config["camera_matrix"]>>cam_matrix,config["distortion_coefficients"]>>dist_coeffs;
		else
			std::cerr<<"Failed to open "<<args.get<std::string>("M")<<", ignoring"<<std::endl;
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
	std::string car_engine,armor_engine;
	std::cout<<"Global config: "<<std::endl<<"Mode: "<<static_cast<unsigned>(mode)<<" Threads: "<<threads<<" Timeout: "<<wait_timeout.count()<<"ms"<<std::endl;
	//Part 2 Convert and prepare model (See convert.h)
	std::cout<<"Model config: "<<std::endl<<"Car file: "<<model_config.car_file<<" Armor file: "<<model_config.armor_file<<std::endl;
	if(model_config.save_ext)
	{
		if((ret=rm::convert_to_trt(model_config,threads,mode,car_engine,armor_engine))>=0)
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
	rm::msg msger; //Communicate with referee
	if((mode&8)==8)
		if(rm::msg::serial_quick_setup(msger,args.get<std::string>("s").c_str()))
		{
			std::thread(&rm::msg::receive,&msger).detach();
			std::thread(&rm::msg::poll,&msger).detach(); //Use a new thread for a blocking task
		}
		else
		{
			std::cerr<<"Serial port is not available"<<std::endl;
			mode&=~8;
		}
	//Part 4.1 Open video
	cv::VideoCapture cap;
	camera::hikcam cam;
	switch(video_config.video_type)
	{
		case 0:
			cap.open(video_config.video_file.c_str());
			break;
		case 1:
			cap.open(video_config.video_device);
			break;
		case 2:
			if(init_hikcamera(cam,video_config.video_device)!=camera::hikcam::OK&&!cam.get_handle())
			{
				std::cerr<<"Camera init failed! ["<<camera::hikcam::last_error<<"]"<<std::endl;
				return 1;
			}
			while(!cam.isopened()&&cam.open()!=camera::hikcam::OK)
			{
				std::cerr<<"Cannot open camera ["<<camera::hikcam::last_error<<"]"<<std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			std::cerr<<"Video handle: "<<cam.get_handle()<<std::endl;
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
		std::tuple<int,int,float> image_arg=apply_video_config(video_config,cap,cam,rec);
		//Part 4.3 Mark points (if not done)
		if((std::get<0>(pose_points).empty()||std::get<1>(pose_points).empty())&&(mode&5)==5)
		{
			cv::Mat img;
			if(cam.isopened()&&cam.start()!=camera::hikcam::OK)
			{
				std::cerr<<"An error occurred while starting video ["<<camera::hikcam::last_error<<"]"<<std::endl;
				return 2;
			}
			std::cout<<"Press any key to mark points"<<std::endl;
			do //Find a best view to mark
			{
				if(cap.isOpened())
					cap.read(img);
				else if(cam.isopened())
				{
					camera::hikcam::FRAME frame=cam.grab();
					if(camera::hikcam::last_error!=camera::hikcam::OK)
					{
						std::cerr<<"An error occurred while capturing video ["<<camera::hikcam::last_error<<"]"<<std::endl;
						break;
					}
					img=cv::Mat(frame.stFrameInfo.nHeight,frame.stFrameInfo.nWidth,CV_8UC1,frame.pBufAddr);
					if(!img.empty())
						cv::cvtColor(img,img,cv::COLOR_BayerBG2BGR);
					cam.free(frame);
				}
			}
			while(!img.empty()&&(cv::imshow(name[0],img),cv::waitKey(wait_timeout.count()*100)==-1));
			if(cam.isopened())
				cam.stop();
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
		//Part 5 Read image, infer & Show
		rm::inferer::threads=threads>0?threads:std::thread::hardware_concurrency(),rm::inferer::timeout=video_config.video_device==0?std::chrono::milliseconds(0):wait_timeout;
		rm::inferer rainfer(model_config.save_ext?car_engine.c_str():model_config.car_file.c_str(),true,image_arg,model_config.save_ext?armor_engine.c_str():model_config.armor_file.c_str(),true);
		ret=run(rainfer,cap,cam,rec,msger,image_arg,pose_points,cam_matrix,dist_coeffs,example_img,args.get<float>("S"));
	}
	return ret;
}
