#include "infer.h"
#include <iostream>
#include <tuple>
#include <utility>
#include <chrono>
#include <future>
#include <vector>
#include <algorithm>
#include <deque>
#include <map>
#include "opencv2/core.hpp"
#include "deploy/option.hpp"
#include "deploy/model.hpp"
#include "deploy/result.hpp"
#include "BYTETracker.h"
#include "STrack.h"
namespace rm
{
	inferer::inferer(const char* car_engine, bool car_BGR, const std::tuple<int,int,float>& image_arg, const char* armor_engine, bool armor_BGR):car_model(0),armor_model(0),tracker(0)
	{
		if(car_engine)
		{
			deploy::InferOption car_option;
			if(car_BGR)
				car_option.enableSwapRB(); //Inference uses RGB
			if(std::get<0>(image_arg)&&std::get<1>(image_arg))
				car_option.setInputDimensions(std::get<0>(image_arg), std::get<1>(image_arg));
			car_model=new deploy::DetectModel(car_engine, car_option);
		}
		car_it=car_model_pool.begin();
		if(std::get<2>(image_arg))
			tracker=new BYTETracker(std::get<2>(image_arg), std::get<2>(image_arg));
		if(armor_engine)
		{
			deploy::InferOption armor_option;
			if(armor_BGR)
				armor_option.enableSwapRB();
			armor_model=new deploy::DetectModel(armor_engine, armor_option);
		}
	}
	deploy::DetectRes inferer::car_thread(cv::Mat img, deploy::DetectModel& car_model)
	{
		return car_model.predict(deploy::Image(img.data,img.cols,img.rows));
	}
	void inferer::deduplicate(std::map<int,std::pair<uint8_t,float>>& tracking, const std::vector<deploy::DetectRes>& results, const std::vector<int>& labels, float fps)
	{
		std::vector<deploy::DetectRes> result=results;
		std::vector<int> ids=labels;
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
				i->second.second=MIN(i->second.second-0.5/fps,1);
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
	}
	std::map<int, std::pair<uint8_t,float>> inferer::armor_infer(const std::vector<cv::Mat>& armors, const std::vector<int>& ids, float fps)
	{
		std::map<int,std::pair<uint8_t,float>> tracking=track_labels;
		std::vector<deploy::Image> imgs; //For inference
		for(auto it=armors.begin();it!=armors.end();++it)
			imgs.emplace_back(it->data,it->cols,it->rows); //Convert to yolo-trt format
		deduplicate(tracking,armor_model->predict(imgs),ids,fps);
		return tracking;
	}
	std::pair<const std::vector<STrack>, const std::map<int, std::pair<uint8_t, float>>> inferer::track(const deploy::DetectRes& inferred, std::chrono::milliseconds delay)
	{
		std::vector<Object> objects;
		for(size_t i=0;i<inferred.num;++i)
			objects.push_back({cv::Rect2f(inferred.boxes[i].left,inferred.boxes[i].top,inferred.boxes[i].right-inferred.boxes[i].left,inferred.boxes[i].bottom-inferred.boxes[i].top),inferred.classes[i],inferred.scores[i]});
		tracked=tracker->update(objects);
		if(armor_model&&tracked.size())
		{
			std::vector<cv::Mat> armors;
			std::vector<int> ids;
			for(auto i=tracked.begin();i!=tracked.end();++i)
			{
				float left=MAX(i->tlbr[0],0),top=MAX(i->tlbr[1],0),width=MIN(i->tlbr[2],tmp_img.cols)-left,height=MIN(i->tlbr[3],tmp_img.rows)-top; //tlbr=top, left, bottom and right edge
				armors.push_back(tmp_img(cv::Rect2f(left,top,width,height)).clone()); //Do not overflow
				ids.push_back(i->track_id);
			}
			track_labels=armor_infer(armors,ids,1000.0/delay.count());
		}
		return std::make_pair(tracked, track_labels);
	}
	deploy::DetectRes inferer::infer(const cv::Mat& img)
	{
		deploy::DetectRes inferred;
		if(thread_pool.size()>0&&(timeout==std::chrono::milliseconds(0)&&thread_pool.size()==threads||thread_pool.front().wait_for(timeout)!=std::future_status::timeout)) //Get a inferred result if available
		{
			inferred=thread_pool.front().get();
			thread_pool.pop_front();
		}
		if(!img.empty())
		{
			//Detect
			tmp_img=img;
			if(thread_pool.size()<threads)
			{
				if(thread_pool.size()>=car_model_pool.size())
					car_it=car_model_pool.insert(car_it,car_model->clone());
				thread_pool.push_back(std::async(car_thread,tmp_img,std::ref(**car_it)));
				++car_it;
				if(car_it==car_model_pool.end())
					car_it=car_model_pool.begin();
			}
			else
				std::cerr<<"Warning: Dropped a new frame due to full thread pool"<<std::endl;
		}
		std::cerr<<"Threads (Using/Pool/Max): "<<thread_pool.size()<<'/'<<car_model_pool.size()<<'/'<<threads<<std::endl;
		return inferred;
	}
	inferer::~inferer()
	{
		if(car_model)
			delete car_model;
		if(armor_model)
			delete armor_model;
		if(tracker)
			delete tracker;
	}
}
