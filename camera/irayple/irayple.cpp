#include "irayple.h"
#include <typeinfo>
#include <stdexcept>
namespace camera
{
    int irayple::last_error = 0;
    irayple::irayple(size_t num) : handle(0)
    {
        irayple::open(num);
    }
    irayple::irayple(const CAMERA_INFO* pInfo) : handle(0)
    {
        if (pInfo)
            irayple::open(pInfo);
    }
    irayple::irayple(const irayple& cam) : handle(0)
    {
        if (cam.is_opened)
            throw std::runtime_error("Camera must be closed before copying");
        if (cam.handle)
        {
            CAMERA_INFO info;
            if (IMV_GetDeviceInfo(cam.handle, &info) == IMV_OK)
                irayple::open(&info);
        }
    }
    irayple& irayple::operator=(irayple cam)
    {
        camera::operator=(cam);
        std::swap(handle, cam.handle);
        return *this;
    }
    irayple::CAMERA_INFO_LIST irayple::enum_cameras(unsigned int interfaceType)
    {
        CAMERA_INFO_LIST pstDevList = { 0 };
        last_error = IMV_EnumDevices(&pstDevList, interfaceType);
        return pstDevList;
    }
    int irayple::open(size_t num)
    {
        CAMERA_INFO_LIST devlist = enum_cameras();
        if (devlist.nDevNum <= num)
            throw std::runtime_error("Camera not found, check camera connection");
        return open(&devlist.pDevInfo[num]);
    }
    int irayple::open(const CAMERA_INFO* pInfo)
    {
        IMV_ECameraAccessPermission nAccessMode = accessPermissionControl;
        if ((!is_opened || (last_error = close()) == IMV_OK) && (!pInfo && handle || (destroy(), (last_error = IMV_CreateHandle(&handle, modeByCameraKey, (void*)pInfo->cameraKey)) == IMV_OK)))
            is_opened = (last_error = IMV_OpenEx(handle, nAccessMode)) == IMV_OK;
        return last_error;
    }
    std::any irayple::get(enum datatype type, const char* key) const
    {
        union irayple_get value;
        switch (type)
        {
        case INT:
            last_error = IMV_GetIntFeatureValue(handle, key, &value.intValue);
            return value.intValue;
        case ENUM:
            last_error = IMV_GetEnumFeatureValue(handle, key, &value.enumValue);
            return value.enumValue;
        case FLOAT:
            last_error = IMV_GetDoubleFeatureValue(handle, key, &value.doubleValue);
            return value.doubleValue;
        case BOOL:
            last_error = IMV_GetBoolFeatureValue(handle, key, &value.bValue);
            return value.bValue;
        case STRING:
            last_error = IMV_GetStringFeatureValue(handle, key, &value.stringValue);
            return value.stringValue;
        default:
            throw std::range_error("Invalid type");
        }
    }
    int irayple::set(const char* key, const std::any& value)
    {
        if (!value.has_value())
            return last_error = IMV_ExecuteCommandFeature(handle, key);
        const std::type_info& ti = value.type();
        if (ti == typeid(int64_t))
            last_error = IMV_SetIntFeatureValue(handle, key, std::any_cast<int64_t>(value));
        else if (ti == typeid(uint64_t))
            last_error = IMV_SetEnumFeatureValue(handle, key, std::any_cast<uint64_t>(value));
        else if (ti == typeid(double))
            last_error = IMV_SetDoubleFeatureValue(handle, key, std::any_cast<double>(value));
        else if (ti == typeid(bool))
            last_error = IMV_SetBoolFeatureValue(handle, key, std::any_cast<bool>(value));
        else if (ti == typeid(const char*))
            last_error = IMV_SetStringFeatureValue(handle, key, std::any_cast<const char*>(value));
        else
            throw std::bad_any_cast();
        return last_error;
    }
    int irayple::start(void)
    {
        return last_error = IMV_StartGrabbing(handle);
    }
    int irayple::grab_callback(void(*callBack)(FRAME* pFrame, void* pUser), void* pUser)
    {
        return last_error = IMV_AttachGrabbing(handle, callBack, pUser);
    }
    std::any irayple::grab(unsigned int ms)
    {
        FRAME frame;
        last_error = IMV_GetFrame(handle, &frame, ms);
        return frame;
    }
    int irayple::free(std::any frame)
    {
        return last_error = IMV_ReleaseFrame(handle, &std::any_cast<FRAME&>(frame));
    }
    int irayple::stop(void)
    {
        return last_error = IMV_StopGrabbing(handle);
    }
    int irayple::close(void)
    {
        if ((last_error = IMV_Close(handle)) == IMV_OK)
            is_opened = false;
        return last_error;
    }
    int irayple::destroy(void)
    {
        last_error = handle ? IMV_DestroyHandle(handle) : irayple::OK;
        handle = 0, is_opened = false;
        return last_error;
    }
    irayple::~irayple()
    {
        destroy();
    }
}
