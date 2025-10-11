#ifndef _INFER_H_
#define _INFER_H_

#include <map>
#include <utility>
#include <chrono>
#include <future>
#include <tuple>
#include <deque>
#include "opencv2/core.hpp"
#include "deploy/model.hpp"
#include "deploy/result.hpp"
#include "BYTETracker.h"
#include "STrack.h"
namespace rm
{
    class inferer
    {
        std::deque<std::future<deploy::DetectRes>> thread_pool;
        std::vector<std::unique_ptr<deploy::DetectModel>> car_model_pool;
        std::vector<std::unique_ptr<deploy::DetectModel>>::iterator car_it;
        deploy::DetectModel* car_model;
        deploy::DetectModel* armor_model;
        BYTETracker* tracker;
        cv::Mat tmp_img; //Sometimes img may be empty, so use the old one instead
        static deploy::DetectRes car_thread(cv::Mat img, deploy::DetectModel& car_model);
        std::map<int, std::pair<uint8_t,float>> armor_infer(const std::vector<cv::Mat>& armors, const std::vector<int>& ids, float fps);
        static void deduplicate(std::map<int,std::pair<uint8_t,float>>& tracking, const std::vector<deploy::DetectRes>& result, const std::vector<int>& labels, float fps=0);
    public:
        static size_t threads;
        static std::chrono::milliseconds timeout;
        std::vector<STrack> tracked;
        std::map<int,std::pair<uint8_t,float>> track_labels;
        inferer(const char* car_engine=0, bool car_BGR=true, const std::tuple<int,int,float>& image_arg=std::make_tuple(0, 0, 0), const char* armor_engine=0, bool armor_BGR=true);
        #if __cplusplus >= 201103L
        inferer(const inferer&)=delete; //Prevents pointer problems
        inferer& operator=(const inferer&)=delete;
        #endif
        deploy::DetectRes infer(const cv::Mat& img);
        std::pair<const std::vector<STrack>, const std::map<int, std::pair<uint8_t, float>>> track(const deploy::DetectRes& inferred, std::chrono::milliseconds delay);
        ~inferer();
    };
}

#endif
