#include "hikcamera.h"
#include <stdexcept>
namespace hikcamera
{
	int camera::last_error=0;
	camera::camera(const CAMERA_INFO* pstDevInfo):handle(0),is_opened(false)
	{
		if(pstDevInfo)
			camera::open(pstDevInfo);
	}
	camera::camera(const camera& cam):handle(0),is_opened(false)
	{
		*this=cam;
	}
	camera& camera::operator=(const camera& cam)
	{
		if(cam.is_opened)
			throw std::runtime_error("Camera must be closed before copying");
		camera::~camera();
		if(cam.handle)
		{
			CAMERA_INFO info;
			if(MV_CC_GetDeviceInfo(cam.handle,&info)==MV_OK)
				camera::open(&info);
		}
		return *this;
	}
	CAMERA_INFO_LIST camera::enum_cameras(unsigned int nTLayerType)
	{
		MV_CC_DEVICE_INFO_LIST pstDevList={0};
		last_error=MV_CC_EnumDevices(nTLayerType,&pstDevList);
		return pstDevList;
	}
	int camera::open(const CAMERA_INFO* pstDevInfo, unsigned int nAccessMode)
	{
		if((!is_opened||(last_error=close())==MV_OK)&&(handle||(last_error=MV_CC_CreateHandle(&handle,pstDevInfo))==MV_OK))
			is_opened=(last_error=MV_CC_OpenDevice(handle,nAccessMode))==MV_OK;
		return last_error;
	}
	int camera::get_value(enum datatype type,const char* strKey,union gettype* value) const
	{
		if(!value&&type!=COMMAND)
			return -1;
		switch(type)
		{
			case INT:
				last_error=MV_CC_GetIntValueEx(handle,strKey,&value->stIntValue);
				break;
			case ENUM:
				last_error=MV_CC_GetEnumValue(handle,strKey,&value->stEnumValue);
				break;
			case FLOAT:
				last_error=MV_CC_GetFloatValue(handle,strKey,&value->stFloatValue);
				break;
			case BOOL:
				last_error=MV_CC_GetBoolValue(handle,strKey,&value->bValue);
				break;
			case STRING:
				last_error=MV_CC_GetStringValue(handle,strKey,&value->stStringValue);
				break;
			default:
				return -1;
		}
		return last_error;
	}
	int camera::set_value(enum datatype type,const char* strKey,const union settype& value)
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
	int camera::start(void(cbOutput)(unsigned char * pData, FRAME_INFO* pFrameInfo, void* pUser),void* pUser)
	{
		if(cbOutput&&(last_error=MV_CC_RegisterImageCallBackEx(handle,cbOutput,pUser))!=MV_OK)
			return last_error;
		return last_error=MV_CC_StartGrabbing(handle);
	}
	int camera::grab_frame(FRAME* pstFrame, unsigned int nMsec) const
	{
		return last_error=MV_CC_GetImageBuffer(handle,pstFrame,nMsec);
	}
	int camera::free_frame(FRAME* pstFrame) const
	{
		return last_error=MV_CC_FreeImageBuffer(handle,pstFrame);
	}
	int camera::stop(void)
	{
		return last_error=MV_CC_StopGrabbing(handle);
	}
	int camera::close(void)
	{
		if((last_error=MV_CC_CloseDevice(handle))==MV_OK)
			is_opened=false;
		return last_error;
	}
	camera::~camera()
	{
		if(handle)
			last_error=MV_CC_DestroyHandle(handle);
		handle=0,is_opened=false;
	}
}
