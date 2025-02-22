#ifndef _HIKCAMERA_H_
#define _HIKCAMERA_H_

#include "MvCameraControl.h"
namespace hikcamera
{
	enum datatype {INT,ENUM,FLOAT,BOOL,STRING,COMMAND};
	union gettype
	{
		MVCC_INTVALUE_EX stIntValue;
		MVCC_ENUMVALUE stEnumValue;
		MVCC_FLOATVALUE stFloatValue;
		bool bValue;
		MVCC_STRINGVALUE stStringValue;
	};
	union settype
	{
		int64_t iValue;
		unsigned int eValue;
		float fValue;
		bool bValue;
		const char* strValue;
	};
	typedef MV_CC_DEVICE_INFO_LIST CAMERA_INFO_LIST;
	typedef MV_CC_DEVICE_INFO CAMERA_INFO;
	typedef MV_FRAME_OUT_INFO_EX FRAME_INFO;
	typedef MV_FRAME_OUT FRAME;
	class camera
	{
	protected:
		void* handle;
		bool is_opened;
	public:
		static int last_error;
		explicit camera(const CAMERA_INFO* pstDevInfo=0);
		camera(const camera& cam);
		camera& operator=(const camera& cam);
		static CAMERA_INFO_LIST enum_cameras(unsigned int nTLayerType = MV_GIGE_DEVICE|MV_USB_DEVICE);
		int open(unsigned int nAccessMode = MV_ACCESS_Control);
		inline bool isopened(void) const
		{
			return is_opened;
		}
		inline void* get_handle(void) const
		{
			return handle;
		}
		int get_value(enum datatype type,const char* strKey,union gettype* value=0) const;
		int set_value(enum datatype type,const char* strKey,const union settype& value=*reinterpret_cast<union hikcamera::settype*>(0));
		int start(void(cbOutput)(unsigned char * pData, FRAME_INFO* pFrameInfo, void* pUser)=0,void* pUser=0);
		int grab_frame(FRAME* pstFrame, unsigned int nMsec=-1) const;
		int free_frame(FRAME* pstFrame) const;
		int stop(void);
		int close(void);
		~camera();
	};
}

#endif
