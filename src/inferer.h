#ifndef _INFERER_H_
#define _INFERER_H_

#include <map>
#include <utility>
#include <chrono>
#include <future>
#include <tuple>
#include <deque>
#include "opencv2/core.hpp"
#include "infer/trtyolo.hpp"
#include "BYTETracker.h"
#include "STrack.h"
struct model_config //Only used for onnx conversion
{
    uint32_t builder_flag; //See NvInfer.h
    const std::string car_file; //ONNX/engine path
    const std::string armor_file; //ONNX/engine path
    const char* save_ext; //.engine
    int64_t tensor_width;
    int64_t tensor_height;
    int64_t batch_size;
};
int convert_to_trt(const struct model_config& config, size_t threads, uint8_t& mode, const char** car_engine_name=0, const char** armor_engine_name=0);
namespace rm
{
    class inferer
    {
        std::deque<std::future<trtyolo::DetectRes>> thread_pool;
        std::vector<std::unique_ptr<trtyolo::DetectModel>> car_model_pool;
        std::vector<std::unique_ptr<trtyolo::DetectModel>>::iterator car_it;
        trtyolo::DetectModel* car_model;
        trtyolo::DetectModel* armor_model;
        BYTETracker* tracker;
        cv::Mat tmp_img; //Sometimes img may be empty, so use the old one instead
        std::map<int, std::pair<uint8_t,float>> armor_infer(const std::vector<cv::Mat>& armors, const std::vector<int>& ids, float fps);
        static void deduplicate(std::map<int,std::pair<uint8_t,float>>& tracking, const std::vector<trtyolo::DetectRes>& result, const std::vector<int>& labels, float fps=0);
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
        trtyolo::DetectRes infer(const cv::Mat& img);
        std::pair<const std::vector<STrack>, const std::map<int, std::pair<uint8_t, float>>> track(const trtyolo::DetectRes& inferred, std::chrono::milliseconds delay);
        ~inferer();
    };
}

#endif
