#include "hikcamera.h"
#include <stdexcept>
namespace camera
{
	int hikcam::last_error=0;
	hikcam::hikcam(const CAMERA_INFO* pstDevInfo):handle(0)
	{
		if(pstDevInfo)
			hikcam::open(pstDevInfo);
	}
	hikcam::hikcam(const hikcam& cam):handle(0)
	{
		*this=cam;
	}
	hikcam& hikcam::operator=(const hikcam& cam)
	{
		if(cam.is_opened)
			throw std::runtime_error("Camera must be closed before copying");
		hikcam::~hikcam();
		if(cam.handle)
		{
			CAMERA_INFO info;
			if(MV_CC_GetDeviceInfo(cam.handle,&info)==MV_OK)
				hikcam::open(&info);
		}
		return *this;
	}
	hikcam::CAMERA_INFO_LIST hikcam::enum_cameras(unsigned int nTLayerType)
	{
		MV_CC_DEVICE_INFO_LIST pstDevList={0};
		last_error=MV_CC_EnumDevices(nTLayerType,&pstDevList);
		return pstDevList;
	}
	int hikcam::open(const CAMERA_INFO* pstDevInfo)
	{
		unsigned int nAccessMode = MV_ACCESS_Control;
		if((!is_opened||(last_error=close())==MV_OK)&&(handle||(last_error=MV_CC_CreateHandle(&handle,pstDevInfo))==MV_OK))
			is_opened=(last_error=MV_CC_OpenDevice(handle,nAccessMode))==MV_OK;
		return last_error;
	}
	union hikget hikcam::get(enum datatype type,const char* strKey) const
	{
		union hikget value;
		switch(type)
		{
			case INT:
				last_error=MV_CC_GetIntValueEx(handle,strKey,&value.stIntValue);
				break;
			case ENUM:
				last_error=MV_CC_GetEnumValue(handle,strKey,&value.stEnumValue);
				break;
			case FLOAT:
				last_error=MV_CC_GetFloatValue(handle,strKey,&value.stFloatValue);
				break;
			case BOOL:
				last_error=MV_CC_GetBoolValue(handle,strKey,&value.bValue);
				break;
			case STRING:
				last_error=MV_CC_GetStringValue(handle,strKey,&value.stStringValue);
				break;
			default:
				throw std::range_error("Invalid type");
		}
		return value;
	}
	int hikcam::set(enum datatype type,const char* strKey,const union hikset& value)
	{
		switch(type)
		{
			case INT:
				last_error=MV_CC_SetIntValueEx(handle,strKey,value.iValue);
				break;
			case ENUM:
				last_error=MV_CC_SetEnumValue(handle,strKey,value.eValue);
				break;
			case FLOAT:
				last_error=MV_CC_SetFloatValue(handle,strKey,value.fValue);
				break;
			case BOOL:
				last_error=MV_CC_SetBoolValue(handle,strKey,value.bValue);
				break;
			case STRING:
				last_error=MV_CC_SetStringValue(handle,strKey,value.strValue);
				break;
			case COMMAND:
				last_error=MV_CC_SetCommandValue(handle,strKey);
				break;
			default:
				return -1;
		}
		return last_error;
	}
	int hikcam::start(void)
	{
		return last_error=MV_CC_StartGrabbing(handle);
	}
	int hikcam::grab_callback(void(cbOutput)(unsigned char * pData, FRAME_INFO* pFrameInfo, void* pUser),void* pUser)
	{
		return last_error=MV_CC_RegisterImageCallBackEx(handle,cbOutput,pUser);
	}
	hikcam::FRAME hikcam::grab(unsigned int nMsec)
	{
		FRAME pstFrame;
		last_error=MV_CC_GetImageBuffer(handle,&pstFrame,nMsec);
		return pstFrame;
	}
	int hikcam::free(FRAME& pstFrame)
	{
		return last_error=MV_CC_FreeImageBuffer(handle,&pstFrame);
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
	hikcam::~hikcam()
	{
		if(handle)
			last_error=MV_CC_DestroyHandle(handle);
		handle=0,is_opened=false;
	}
}
