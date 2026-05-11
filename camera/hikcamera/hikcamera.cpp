#include "hikcamera.h"
#include <typeinfo>
#include <stdexcept>
namespace camera
{
    int hikcam::last_error=0;
    hikcam::hikcam(size_t num):handle(0)
    {
        hikcam::open(num);
    }
    hikcam::hikcam(const CAMERA_INFO* pstDevInfo):handle(0)
    {
        if(pstDevInfo)
            hikcam::open(pstDevInfo);
    }
    hikcam::hikcam(const hikcam& cam):handle(0)
    {
        if(cam.is_opened)
            throw std::runtime_error("Camera must be closed before copying");
        if(cam.handle)
        {
            CAMERA_INFO info;
            if(MV_CC_GetDeviceInfo(cam.handle,&info)==MV_OK)
                hikcam::open(&info);
        }
    }
    hikcam& hikcam::operator=(hikcam cam)
    {
        camera::operator=(cam);
        std::swap(handle,cam.handle);
        return *this;
    }
    hikcam::CAMERA_INFO_LIST hikcam::enum_cameras(unsigned int nTLayerType)
    {
        CAMERA_INFO_LIST pstDevList={0};
        last_error=MV_CC_EnumDevices(nTLayerType,&pstDevList);
        return pstDevList;
    }
    int hikcam::open(size_t num)
    {
        CAMERA_INFO_LIST devlist=enum_cameras(); //USB cameras must be enumerated before opening
        if(devlist.nDeviceNum<=num)
            throw std::runtime_error("Camera not found, check camera connection");
        return open(devlist.pDeviceInfo[num]);
    }
    int hikcam::open(const CAMERA_INFO* pstDevInfo)
    {
        unsigned int nAccessMode = MV_ACCESS_Control;
        if((!is_opened||(last_error=close())==MV_OK)&&(!pstDevInfo&&handle||(destroy(),(last_error=MV_CC_CreateHandle(&handle,pstDevInfo))==MV_OK)))
            is_opened=(last_error=MV_CC_OpenDevice(handle,nAccessMode))==MV_OK;
        return last_error;
    }
    std::any hikcam::get(enum datatype type,const char* key) const
    {
        union hikget value;
        switch(type)
        {
            case INT:
                last_error=MV_CC_GetIntValueEx(handle,key,&value.stIntValue);
                return value.stIntValue;
            case ENUM:
                last_error=MV_CC_GetEnumValue(handle,key,&value.stEnumValue);
                return value.stEnumValue;
            case FLOAT:
                last_error=MV_CC_GetFloatValue(handle,key,&value.stFloatValue);
                return value.stFloatValue;
            case BOOL:
                last_error=MV_CC_GetBoolValue(handle,key,&value.bValue);
                return value.bValue;
            case STRING:
                last_error=MV_CC_GetStringValue(handle,key,&value.stStringValue);
                return value.stStringValue;
            default:
                throw std::range_error("Invalid type");
        }
    }
    int hikcam::set(const char* key, const std::any& value)
    {
        if(!value.has_value())
            return last_error = MV_CC_SetCommandValue(handle, key);
        const std::type_info& ti = value.type();
        if(ti == typeid(int64_t))
            last_error = MV_CC_SetIntValueEx(handle, key, std::any_cast<int64_t>(value));
        else if(ti == typeid(unsigned int))
            last_error = MV_CC_SetEnumValue(handle, key, std::any_cast<unsigned int>(value));
        else if(ti == typeid(float))
            last_error = MV_CC_SetFloatValue(handle, key, std::any_cast<float>(value));
        else if(ti == typeid(bool))
            last_error = MV_CC_SetBoolValue(handle, key, std::any_cast<bool>(value));
        else if(ti == typeid(const char*))
            last_error = MV_CC_SetStringValue(handle, key, std::any_cast<const char*>(value));
        else
            throw std::bad_any_cast();
        return last_error;
    }
    int hikcam::start(void)
    {
        return last_error=MV_CC_StartGrabbing(handle);
    }
    int hikcam::grab_callback(void __stdcall (*cbOutput)(unsigned char * pData, FRAME_INFO* pFrameInfo, void* pUser),void* pUser)
    {
        return last_error=MV_CC_RegisterImageCallBackEx(handle,cbOutput,pUser);
    }
    std::any hikcam::grab(unsigned int ms)
    {
        FRAME pstFrame={0};
        last_error=MV_CC_GetImageBuffer(handle,&pstFrame,ms);
        return pstFrame;
    }
    int hikcam::free(std::any frame)
    {
        return last_error=MV_CC_FreeImageBuffer(handle,&std::any_cast<FRAME&>(frame));
    }
    int hikcam::stop(void)
    {
        return last_error=MV_CC_StopGrabbing(handle);
    }
    int hikcam::close(void)
    {
        if((last_error=MV_CC_CloseDevice(handle))==MV_OK)
            is_opened=false;
        return last_error;
    }
    int hikcam::destroy(void)
    {
        last_error=handle?MV_CC_DestroyHandle(handle):hikcam::OK;
        handle=0,is_opened=false;
        return last_error;
    }
    hikcam::~hikcam()
    {
        destroy();
    }
}
