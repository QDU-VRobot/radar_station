#include "hikcamera.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <future>
#include <deque>
#include <map>
#include <string>
#include <vector>
#include <utility>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "deploy/option.hpp"
#include "deploy/model.hpp"
#include "deploy/result.hpp"
#include "BYTETracker.h"
#include "STrack.h"
extern uint8_t mode;
extern size_t threads;
extern std::chrono::milliseconds wait_timeout;
deploy::DetectRes car_infer(const cv::Mat& img, deploy::DetectModel* car_model)
{
	return car_model->predict(deploy::Image(img.data,img.cols,img.rows));
}
inline std::map<int, std::pair<uint8_t,float>> armor_infer(const std::vector<cv::Mat>& armors, std::vector<int>& ids, deploy::DetectModel* armor_model)
{
	static std::map<int,std::pair<uint8_t,float>> tracking;
	std::vector<deploy::Image> imgs;
	for(auto it=armors.begin();it!=armors.end();++it)
		imgs.emplace_back(it->data,it->cols,it->rows);
	std::vector<deploy::DetectRes> result=armor_model->predict(imgs);
	std::map<uint8_t,std::pair<size_t,float>> deduplicated;
	for(size_t i=0;i<result.size();++i)
		for(size_t j=0;j<result[i].classes.size();++j)
			if(deduplicated.count(result[i].classes[j]))
			{
				if(deduplicated[result[i].classes[j]].second<result[i].scores[j])
				{
					deduplicated[result[i].classes[j]].second=result[i].scores[j];
					std::swap(result[i],result[deduplicated[result[i].classes[j]].first]);
					std::swap(ids[i],ids[deduplicated[result[i].classes[j]].first]);
					--i;
					break;
				}
			}
			else
			{
				deduplicated[result[i].classes[j]]=std::make_pair(i,result[i].scores[j]);
				break;
			}
	for(auto it=deduplicated.begin();it!=deduplicated.end();++it)
		for(auto i=tracking.begin();i!=tracking.end();++i)
			if(it->first==i->second.first&&ids[it->second.first]!=i->first)
				i=tracking.erase(i);
	for(auto it=deduplicated.begin();it!=deduplicated.end();++it)
		tracking[ids[it->second.first]]=std::make_pair(it->first,it->second.second);
	return tracking;
}
inline std::pair<const vector<STrack>, const std::map<int, std::pair<uint8_t,float>>> threaded_infer(cv::Mat img, bool isBayerBG, const deploy::DetectModel* car_model, deploy::DetectModel* armor_model=0, BYTETracker* tracker=0, bool wait_forever=false)
{
	static std::deque<std::future<deploy::DetectRes>> thread_pool;
	static std::vector<std::unique_ptr<deploy::DetectModel>> car_model_pool;
	static auto it=car_model_pool.insert(car_model_pool.begin(),car_model->clone());
	vector<STrack> tracked;
	static std::map<int,std::pair<uint8_t,float>> track_id;
	if(isBayerBG)
		cv::cvtColor(img,img,cv::COLOR_BayerBG2RGB);
	if(thread_pool.size()>0&&(wait_forever&&thread_pool.size()==threads||thread_pool.front().wait_for(wait_timeout)!=std::future_status::timeout))
	{
		deploy::DetectRes inferred=thread_pool.front().get();
		thread_pool.pop_front();
		//Track then
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
				float left=MAX(i->tlwh[0],0),top=MAX(i->tlwh[1],0),width=MIN(i->tlwh[2],img.cols-i->tlwh[0]),height=MIN(i->tlwh[3],img.rows-i->tlwh[1]);
				armors.push_back(img(cv::Rect2f(left,top,width,height)).clone());
				ids.push_back(i->track_id);
			}
			track_id=armor_infer(armors,ids,armor_model);
		}
	}
	if(!img.empty())
	{
		if(thread_pool.size()<threads)
		{
			if(thread_pool.size()==car_model_pool.size())
				it=car_model_pool.insert(it,car_model->clone());
			thread_pool.push_back(std::async(car_infer,img,it->get()));
			++it;
			if(it==car_model_pool.end())
				it=car_model_pool.begin();
		}
		else
			std::cerr<<"Warning: Dropped a new frame due to full thread pool"<<std::endl;
	}
	std::cerr<<"Threads: "<<thread_pool.size()<<std::endl;
	return std::make_pair(tracked,track_id);
}
inline void output(cv::Mat& img,const std::pair<const vector<STrack>, const std::map<int, std::pair<uint8_t,float>>>& result, cv::VideoWriter& rec)
{
	if(!img.empty())
	{
		if((mode&1)==1&&result.first.size()>0)
			for(auto it=result.first.begin();it!=result.first.end();++it)
			{
				std::cout<<it->track_id<<':'<<it->tlwh[0]<<' '<<it->tlwh[1]<<' '<<it->tlwh[2]<<' '<<it->tlwh[3]<<';';
				if(result.second.count(it->track_id))
					std::cout<<static_cast<unsigned>(result.second.at(it->track_id).first)<<','<<result.second.at(it->track_id).second;
				std::cout<<std::endl;
			}
		if(mode&12)
		{
			std::string label;
			for(auto it=result.first.begin();it!=result.first.end();++it)
			{
				cv::rectangle(img,cv::Rect(it->tlwh[0],it->tlwh[1],it->tlwh[2],it->tlwh[3]),cv::Scalar(0,255,0),2);
				cv::putText(img,std::to_string(it->score),cv::Point(it->tlwh[0],it->tlwh[1]),0,1,cv::Scalar(0,255,0),2);
				if((mode&3)==3&&result.second.count(it->track_id))
				{
					((label=std::to_string(result.second.at(it->track_id).first))+=':')+=std::to_string(result.second.at(it->track_id).second);
					cv::putText(img,label,cv::Point(it->tlwh[0],it->tlwh[1]+it->tlwh[3]),1,1,cv::Scalar(0,255,0),2);
				}
			}
			if((mode&8)==8)
				rec.write(img);
			if((mode&4)==4)
			{
				cv::imshow("Radar",img);
				cv::waitKey(1);
			}
		}
	}
}
int run(const char* car_engine, const char* armor_engine, const std::pair<const std::pair<int,int>,int>& image_arg, cv::VideoCapture& cap, hikcamera::camera& hikcam, cv::VideoWriter& rec, bool wait_forever)
{
	unsigned short err_counter=0;
	deploy::DetectModel *car_model=0,*armor_model=0;
	BYTETracker *tracker=0;
	if((mode&1)==1)
	{
		std::cout<<"Loading car engine..."<<std::endl;
		deploy::InferOption car_option;
		if(cap.isOpened())
			car_option.enableSwapRB();
		car_option.setInputDimensions(image_arg.first.first,image_arg.first.second);
		car_model=new deploy::DetectModel(car_engine,car_option);
		tracker=new BYTETracker(image_arg.second,image_arg.second);
		if((mode&3)==3)
		{
			std::cout<<"Loading armor engine..."<<std::endl;
			deploy::InferOption armor_option;
			if(cap.isOpened())
				armor_option.enableSwapRB();
			armor_model=new deploy::DetectModel(armor_engine,armor_option);
		}
	}
	cv::Mat img;
	std::pair<vector<STrack>, std::map<int, std::pair<uint8_t,float>>> result;
	std::cout<<"Video starting"<<std::endl;
	if(cap.isOpened())
	{
		while(cap.read(img)&&!img.empty())
		{
			static std::chrono::steady_clock::time_point old_timestamp=std::chrono::steady_clock::now();
			std::chrono::steady_clock::time_point now=std::chrono::steady_clock::now();
			if((mode&1)==1)
				result=threaded_infer(img,false,car_model,armor_model,tracker,wait_forever);
			std::cerr<<"Cycle: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp).count()<<"ms"<<std::endl;
			old_timestamp=now;
			output(img,result,rec);
		}
		std::cout<<"Video stopped"<<std::endl;
	}
	else if(hikcam.get_handle())
	{
		if(hikcam.start()!=MV_OK)
		{
			std::cerr<<"An error occurred while starting video ["<<hikcamera::camera::last_error<<"]"<<std::endl;
			if(hikcam.set_value(hikcamera::COMMAND,"DeviceReset")==MV_OK)
				std::cerr<<"Reset command has been sent, please retry later"<<std::endl;
			return 2;
		}
		hikcamera::FRAME frame;
		while(err_counter<wait_timeout.count()*1000)
		{
			err_counter=hikcam.grab_frame(&frame,wait_timeout.count())==MV_OK?0:err_counter+1;
			static int64_t old_timestamp=frame.stFrameInfo.nHostTimeStamp;
			img=cv::Mat(frame.stFrameInfo.nHeight,frame.stFrameInfo.nWidth,CV_8UC1,frame.pBufAddr);
			if((mode&1)==1)
				result=threaded_infer(img,true,car_model,armor_model,tracker,wait_forever);
			if(!err_counter)
			{
				std::cerr<<"Interval: "<<frame.stFrameInfo.nHostTimeStamp-old_timestamp<<"ms"<<std::endl;
				old_timestamp=frame.stFrameInfo.nHostTimeStamp;
				cv::cvtColor(img,img,cv::COLOR_BayerBG2BGR);
				output(img,result,rec);
				hikcam.free_frame(&frame);
			}
		}
		std::cout<<"Video stopped ["<<hikcamera::camera::last_error<<"]"<<std::endl;
	}
	if((mode&1)==1)
	{
		delete tracker;
		delete car_model;
		if((mode&3)==3)
			delete armor_model;
	}
	return 0;
}
