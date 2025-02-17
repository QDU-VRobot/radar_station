#include "hikcamera.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <future>
#include <deque>
#include <vector>
#include <utility>
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "deploy/option.hpp"
#include "deploy/model.hpp"
#include "deploy/result.hpp"
extern uint8_t mode;
extern size_t threads;
extern std::chrono::milliseconds wait_timeout;
std::pair<const deploy::DetectRes,const std::vector<uint8_t>> infer(cv::Mat img, bool isBayerBG, deploy::DetectModel* car_model, deploy::DetectModel* armor_model)
{
	if(isBayerBG)
		cv::cvtColor(img,img,cv::COLOR_BayerBG2RGB);
	deploy::Image input_img(img.data,img.cols,img.rows);
	deploy::DetectRes result=car_model->predict(input_img);
	std::vector<uint8_t> id;
	//TODO: if((mode&3)==3)
	return std::make_pair(result,id);
}
inline void threaded_inference(std::deque<std::future<std::pair<const deploy::DetectRes,const std::vector<uint8_t>>>>& thread_pool, std::vector<std::pair<std::unique_ptr<deploy::DetectModel>,std::unique_ptr<deploy::DetectModel>>>& model_pool, bool isBayerBG, cv::Mat& img, const deploy::DetectModel* car_model, const deploy::DetectModel* armor_model, bool wait_forever=false)
{
	static auto it=model_pool.insert(model_pool.begin(),std::make_pair(car_model->clone(), (mode&3)==3?armor_model->clone():std::unique_ptr<deploy::DetectModel>()));
	if(thread_pool.size()>0&&(wait_forever&&thread_pool.size()==threads||thread_pool.front().wait_for(wait_timeout)!=std::future_status::timeout))
	{
		std::cout<<thread_pool.front().get().first<<std::endl;
		//Track then
		thread_pool.pop_front();
	}
	if(img.empty())
		return;
	if(thread_pool.size()<threads)
	{
		if(thread_pool.size()==model_pool.size())
			it=model_pool.insert(it,std::make_pair(car_model->clone(), (mode&3)==3?armor_model->clone():std::unique_ptr<deploy::DetectModel>()));
		thread_pool.push_back(std::async(infer,img,isBayerBG,it->first.get(),it->second.get()));
		++it;
		if(it==model_pool.end())
			it=model_pool.begin();
	}
	else
		std::cerr<<"Warning: Dropped a new frame due to full thread pool"<<std::endl;
	std::cout<<"Threads: "<<thread_pool.size()<<std::endl;
}
int run(const char* car_engine, const char* armor_engine, const std::pair<int,int>& image_arg, cv::VideoCapture& cap, hikcamera::camera& hikcam, bool wait_forever)
{
	unsigned short err_counter=0;
	deploy::DetectModel *car_model=0,*armor_model=0;
	if((mode&1)==1)
	{
		std::cout<<"Loading car engine..."<<std::endl;
		deploy::InferOption car_option;
		if(cap.isOpened())
			car_option.enableSwapRB();
		car_option.setInputDimensions(image_arg.first,image_arg.second);
		car_model=new deploy::DetectModel(car_engine,car_option);
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
	std::deque<std::future<std::pair<const deploy::DetectRes,const std::vector<uint8_t>>>> thread_pool;
	std::vector<std::pair<std::unique_ptr<deploy::DetectModel>,std::unique_ptr<deploy::DetectModel>>> model_pool;
	std::cout<<"Video starting"<<std::endl;
	if(cap.isOpened())
	{
		while(cap.read(img)&&!img.empty())
		{
			static std::chrono::steady_clock::time_point old_timestamp=std::chrono::steady_clock::now();
			std::chrono::steady_clock::time_point now=std::chrono::steady_clock::now();
			if((mode&1)==1)
				threaded_inference(thread_pool,model_pool,false,img,car_model,armor_model,wait_forever);
			std::cerr<<"Cycle latency: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-old_timestamp).count()<<"ms"<<std::endl;
			old_timestamp=now;
			//TODO: Record
			if(!img.empty()&&(mode&4)==4)
			{
				cv::imshow("Radar",img);
				cv::waitKey(1);
			}
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
		while(err_counter<255)
		{
			err_counter=hikcam.grab_frame(&frame,wait_timeout.count())==MV_OK?0:err_counter+1;
			static int64_t old_timestamp=frame.stFrameInfo.nHostTimeStamp;
			img=cv::Mat(frame.stFrameInfo.nHeight,frame.stFrameInfo.nWidth,CV_8UC1,frame.pBufAddr);
			if((mode&1)==1)
				threaded_inference(thread_pool,model_pool,true,img,car_model,armor_model,wait_forever);
			if(!err_counter)
			{
				std::cerr<<"Cycle latency: "<<frame.stFrameInfo.nHostTimeStamp-old_timestamp<<"ms"<<std::endl;
				old_timestamp=frame.stFrameInfo.nHostTimeStamp;
				//TODO: Record
				if((mode&4)==4)
				{
					cv::cvtColor(img,img,cv::COLOR_BayerBG2BGR);
					cv::imshow("Radar",img);
					cv::waitKey(1);
				}
				hikcam.free_frame(&frame);
			}
		}
		std::cout<<"Video stopped ["<<hikcamera::camera::last_error<<"]"<<std::endl;
	}
	if((mode&1)==1)
	{
		delete car_model;
		if((mode&3)==3)
			delete armor_model;
	}
	return 0;
}
