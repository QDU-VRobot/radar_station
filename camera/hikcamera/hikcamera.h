#ifndef _HIKCAMERA_H_
#define _HIKCAMERA_H_

#include "camera.h"
#include "MvCameraControl.h"
namespace camera
{
	union hikget
	{
		MVCC_INTVALUE_EX stIntValue;
		MVCC_ENUMVALUE stEnumValue;
		MVCC_FLOATVALUE stFloatValue;
		bool bValue;
		MVCC_STRINGVALUE stStringValue;
	};
	//Unify different data types when getting/setting values
	union hikset
	{
		int64_t iValue;
		unsigned int eValue;
		float fValue;
		bool bValue;
		const char* strValue;
	};
	class hikcam:public camera<MV_CC_DEVICE_INFO,MV_FRAME_OUT,union hikget,union hikset>
	{
	private:
		void* handle;
	public:
		//Alias of MV_*, providing convenience when changing camera type
		typedef MV_CC_DEVICE_INFO_LIST CAMERA_INFO_LIST;
		typedef MV_CC_DEVICE_INFO CAMERA_INFO;
		typedef MV_FRAME_OUT_INFO_EX FRAME_INFO;
		typedef MV_FRAME_OUT FRAME;
		enum {OK=MV_OK};
		static int last_error;
		static CAMERA_INFO_LIST enum_cameras(unsigned int nTLayerType = MV_GIGE_DEVICE|MV_USB_DEVICE);
		hikcam(const CAMERA_INFO* pstDevInfo=0);
		hikcam(const hikcam& cam);
		hikcam& operator=(const hikcam& cam);
		#if __cplusplus >= 201103L
		hikcam(hikcam&& cam)=default;
		hikcam& operator=(hikcam&& cam)=default;
		#endif
		int open(const CAMERA_INFO* pstDevInfo=0);
		const char* type(void) const
		{
			return "Hikcamera";
		}
		inline void* get_handle(void) const
		{
			return handle;
		}
		union hikget get(enum datatype type,const char* strKey) const;
		int set(enum datatype type,const char* strKey,const union hikset& value=*reinterpret_cast<union hikset*>(0));
		int start(void);
		int grab_callback(void(cbOutput)(unsigned char * pData, FRAME_INFO* pFrameInfo, void* pUser),void* pUser=0);
		FRAME grab(unsigned int nMsec=-1);
		int free(FRAME& pstFrame);
		int stop(void);
		int close(void);
		~hikcam();
	};
}

#endif
