#include "common.h"
#include "hikcamera.h"
#include "serial.h"
#include <signal.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <future>
#include <deque>
#include <map>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <algorithm>
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "deploy/option.hpp"
#include "deploy/model.hpp"
#include "deploy/result.hpp"
#include "BYTETracker.h"
#include "STrack.h"
extern size_t threads;
extern std::chrono::milliseconds wait_timeout;
static bool running=true;
static void handle_exit(int type) //Exit safely or the recorded video will corrupt!
{
	std::cerr<<"Received signal "<<type<<", exiting"<<std::endl;
	running=false;
	signal(type,SIG_DFL); //Press again to force exit
}
static deploy::DetectRes car_infer(const cv::Mat& img, deploy::DetectModel* car_model)
{
	return car_model->predict(deploy::Image(img.data,img.cols,img.rows));
}
static std::map<int, std::pair<uint8_t,float>> armor_infer(const std::vector<cv::Mat>& armors, std::vector<int>& ids, deploy::DetectModel* armor_model, float fps)
{
	static std::map<int,std::pair<uint8_t,float>> tracking; //Save tracked robots
	std::vector<deploy::Image> imgs; //For inference
	for(auto it=armors.begin();it!=armors.end();++it)
		imgs.emplace_back(it->data,it->cols,it->rows); //Convert to yolo-trt format
	std::vector<deploy::DetectRes> result=armor_model->predict(imgs);
	std::map<uint8_t,std::pair<size_t,float>> deduplicated;
	//Deduplicate by robot id, sorted by confidence
	for(size_t i=0;i<result.size();++i)
		for(size_t j=0;j<result[i].classes.size();++j)
			if(deduplicated.count(result[i].classes[j]))
			{
				if(deduplicated[result[i].classes[j]].second<result[i].scores[j])
				{
					deduplicated[result[i].classes[j]].second=result[i].scores[j];
					std::swap(ids[i],ids[deduplicated[result[i].classes[j]].first]);
					std::swap(result[i],result[deduplicated[result[i].classes[j]].first]);
					--i;
					break;
				}
			}
			else
			{
				deduplicated[result[i].classes[j]]=std::make_pair(i,result[i].scores[j]);
				break;
			}
	//Decrease confidence gradually for untracked robots
	auto i=tracking.begin();
	while(i!=tracking.end())
	{
		if(std::find(ids.begin(),ids.end(),i->first)==ids.end())
		{
			i->second.second=MIN(i->second.second-1.0/fps,1);
			if(i->second.second<=0)
			{
				i=tracking.erase(i);
				continue;
			}
		}
		++i;
	}
	//Update tracking result if the confidence is higher, for each robot and each tracking id
	for(auto it=deduplicated.begin();it!=deduplicated.end();++it)
	{
		bool exist=false;
		for(auto i=tracking.begin();i!=tracking.end();++i)
			if(it->first==i->second.first)
			{
				if(it->second.second>i->second.second)
				{
					if(ids[it->second.first]!=i->first)
						tracking.erase(i);
				}
				else
					exist=true;
				break;
			}
		if(!exist&&!(tracking.count(ids[it->second.first])&&tracking[ids[it->second.first]].second>=it->second.second))
			tracking[ids[it->second.first]]=std::make_pair(it->first,it->second.second);
	}
	return tracking;
}
static std::pair<const vector<STrack>, const std::map<int, std::pair<uint8_t,float>>> threaded_infer(const cv::Mat& img, const deploy::DetectModel* car_model, deploy::DetectModel* armor_model=0, BYTETracker* tracker=0, bool wait_forever=false, std::chrono::milliseconds delay=std::chrono::milliseconds(0))
{
	static std::deque<std::future<deploy::DetectRes>> thread_pool;
	static std::vector<std::unique_ptr<deploy::DetectModel>> car_model_pool;
	static auto it=car_model_pool.insert(car_model_pool.begin(),car_model->clone());
	vector<STrack> tracked;
	static std::map<int,std::pair<uint8_t,float>> track_id;
	static cv::Mat old_img; //Sometimes img may be empty, so use the old one instead
	if(thread_pool.size()>0&&(wait_forever&&thread_pool.size()==threads||thread_pool.front().wait_for(wait_timeout)!=std::future_status::timeout)) //Get a inferred result if available
	{
		deploy::DetectRes inferred=thread_pool.front().get();
		thread_pool.pop_front();
		//Track
		std::vector<Object> objects;
		for(size_t i=0;i<inferred.num;++i)
			objects.push_back({cv::Rect2f(inferred.boxes[i].left,inferred.boxes[i].top,inferred.boxes[i].right-inferred.boxes[i].left,inferred.boxes[i].bottom-inferred.boxes[i].top),inferred.classes[i],inferred.scores[i]});
		tracked=tracker->update(objects);
		if((mode&3)==3&&tracked.size())
		{
			std::vector<cv::Mat> armors;
			std::vector<int> ids;
			for(auto i=tracked.begin();i!=tracked.end();++i)
			{
				float left=MAX(i->tlbr[0],0),top=MAX(i->tlbr[1],0),width=MIN(i->tlbr[2],old_img.cols)-left,height=MIN(i->tlbr[3],old_img.rows)-top; //tlbr=top, left, bottom and right edge
				armors.push_back(old_img(cv::Rect2f(left,top,width,height)).clone()); //Do not overflow
				ids.push_back(i->track_id);
			}
			track_id=armor_infer(armors,ids,armor_model,1000.0/delay.count());
		}
	}
	if(!img.empty())
	{
		//Detect
		old_img=img;
		if(thread_pool.size()<threads)
		{
			if(thread_pool.size()==car_model_pool.size())
				it=car_model_pool.insert(it,car_model->clone());
			thread_pool.push_back(std::async(car_infer,old_img,it->get()));
			++it;
			if(it==car_model_pool.end())
				it=car_model_pool.begin();
		}
		else
			std::cerr<<"Warning: Dropped a new frame due to full thread pool"<<std::endl;
	}
	std::cerr<<"Threads (Using/Pool/Max): "<<thread_pool.size()<<'/'<<car_model_pool.size()<<'/'<<threads<<std::endl;
	return std::make_pair(tracked,track_id);
}
static char output(cv::Mat& img,const std::pair<const vector<STrack>, const std::map<int, std::pair<uint8_t,float>>>& result, const cv::Mat& m1, const cv::Mat& m2, const std::vector<std::pair<std::vector<cv::Point2f>,float>>& contours, cv::VideoWriter& rec, serial::terminal& sport, cv::Mat&& example_img, float scale, unsigned short latency=0) //Display, send to judge or record
{
	static bool serial_busy=false; //Prevent infinite thread creation
	std::vector<cv::Point2f> center_points; //Mark to the minimap and send to server
	if((mode&1)==1&&result.first.size()>0)
	{
		std::vector<uint8_t> ids;
		for(auto it=result.first.begin();it!=result.first.end();++it)
		{
			cv::Point2f pos2d(it->tlbr[0]+it->tlwh[2]/2,it->tlbr[3]-it->tlwh[2]/4); //May not accurate
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
		if((mode&11)==11&&!serial_busy&&serial_info.robot_id)
			serial_busy=true,std::thread(send_pos,std::ref(sport),center_points,ids,&serial_busy).detach(); //Use a new thread for a blocking task
	}
	if(!img.empty()&&((mode&4)==4||(mode&48)&&rec.isOpened()))
	{
		if((mode&16)==16&&rec.isOpened())
			rec.write(img);
		//Draw on output image
		cv::putText(img,std::string("Latency: ")+std::to_string(latency)+"ms",cv::Point(img.cols-255,30),0,1,cv::Scalar(0,255,0),2);
		if((mode&8)==8)
		{
			cv::putText(img,std::string("State: ")+std::to_string(serial_info.game_progress),cv::Point(5,30),0,1,cv::Scalar(0,255,0),2);
			cv::putText(img,std::string("Timer: ")+std::to_string(serial_info.stage_remain_time/60)+':'+std::to_string(serial_info.stage_remain_time%60),cv::Point(150,30),0,1,cv::Scalar(0,255,0),2);
			if(serial_info.robot_id)
				cv::putText(img,std::string("Team: ")+(serial_info.robot_id/100?"Blue":"Red"),cv::Point(350,30),0,1,cv::Scalar(0,255,0),2);
			{
				std::string marked("Marked: ");
				for(uint8_t i=1;i<=1<<4;i<<=1)
					marked+=serial_info.mark_progress&i?'1':'0';
				cv::putText(img,marked,cv::Point(5,img.rows-10),0,1,cv::Scalar(0,255,0),2);
			}
			cv::putText(img,std::string("Damages: ")+std::to_string(serial_info.more_damage)+(serial_info.damaging?'*':' '),cv::Point(img.cols-210,img.rows-10),0,1,cv::Scalar(0,255,0),2);
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
int run(const char* car_engine, const char* armor_engine, const std::tuple<int,int,float>& image_arg, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec, serial::terminal& sport, bool wait_forever, const std::tuple<cv::Mat,cv::Mat,std::vector<std::pair<std::vector<cv::Point2f>,float>>>& pose_points, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const cv::Mat& example_img, float scale)
{
	unsigned int err_counter=0;
	deploy::DetectModel *car_model=0,*armor_model=0;
	BYTETracker *tracker=0;
	if((mode&1)==1)
	{
		std::cout<<"Loading car engine..."<<std::endl;
		deploy::InferOption car_option;
		if(cap.isOpened())
			car_option.enableSwapRB(); //Inference uses RGB
		car_option.setInputDimensions(std::get<0>(image_arg),std::get<1>(image_arg));
		car_model=new deploy::DetectModel(car_engine,car_option);
		tracker=new BYTETracker(std::get<2>(image_arg),std::get<2>(image_arg));
		if((mode&3)==3)
		{
			std::cout<<"Loading armor engine..."<<std::endl;
			deploy::InferOption armor_option;
			armor_option.enableSwapRB();
			armor_model=new deploy::DetectModel(armor_engine,armor_option);
		}
	}
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
					result=threaded_infer(img,car_model,armor_model,tracker,wait_forever,std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp));
				std::cerr<<"Cycle: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp).count()<<"ms"<<std::endl;
				if(output(img,result,m1,m2,std::get<2>(pose_points),rec,sport,example_img.clone(),scale,std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp).count())==27)
					running=false; //ESC to exit
				old_timestamp=now;
			}
		while(running&&(mode&64)==64&&cap.set(cv::CAP_PROP_POS_FRAMES,0));
		std::cout<<"Video stopped"<<std::endl;
	}
	else if(hikcam.isopened())
	{
		if(hikcam.start()!=hikcamera::OK)
		{
			std::cerr<<"An error occurred while starting video ["<<hikcamera::camera::last_error<<"]"<<std::endl;
			if(hikcam.set_value(hikcamera::COMMAND,"DeviceReset")==hikcamera::OK)
				std::cerr<<"Reset command has been sent, please retry later"<<std::endl;
			return 2;
		}
		hikcamera::FRAME frame;
		while(err_counter<MAX(2000/wait_timeout.count(),10000/std::get<2>(image_arg))&&running) //2s or 10 pictures
		{
			err_counter=hikcam.grab_frame(&frame,wait_timeout.count())==hikcamera::OK?0:err_counter+1;
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
				result=threaded_infer(img,car_model,armor_model,tracker,wait_forever,std::chrono::milliseconds(frame.stFrameInfo.nHostTimeStamp-old_timestamp));
			if(!err_counter)
			{
				std::cerr<<"Interval: "<<frame.stFrameInfo.nHostTimeStamp-old_timestamp<<"ms"<<std::endl;
				cv::cvtColor(img,img,cv::COLOR_RGB2BGR);
			}
			if(output(img,result,m1,m2,std::get<2>(pose_points),rec,sport,example_img.clone(),scale,frame.stFrameInfo.nHostTimeStamp-old_timestamp)==27)
				running=false; //ESC to exit
			if(!err_counter)
			{
				old_timestamp=frame.stFrameInfo.nHostTimeStamp;
				hikcam.free_frame(&frame);
			}
		}
		hikcam.stop();
		std::cout<<"Video stopped ["<<(err_counter=hikcamera::camera::last_error)<<"]"<<std::endl;
	}
	cv::destroyAllWindows();
	if((mode&1)==1)
	{
		delete tracker;
		delete car_model;
		if((mode&3)==3)
			delete armor_model;
	}
	return err_counter;
}
