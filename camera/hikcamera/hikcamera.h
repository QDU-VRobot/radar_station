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
    class hikcam:public camera
    {
    private:
        void* handle;
    public:
        //Alias of MV_*, providing convenience when changing camera type
        typedef MVCC_INTVALUE_EX INT_VAL;
        typedef MVCC_ENUMVALUE ENUM_VAL;
        typedef MVCC_FLOATVALUE FLOAT_VAL;
        typedef MVCC_STRINGVALUE STRING_VAL;
        typedef MV_CC_DEVICE_INFO_LIST CAMERA_INFO_LIST;
        typedef MV_CC_DEVICE_INFO CAMERA_INFO;
        typedef MV_FRAME_OUT_INFO_EX FRAME_INFO;
        typedef MV_FRAME_OUT FRAME;
        enum {OK=MV_OK};
        static int last_error;
        static CAMERA_INFO_LIST enum_cameras(unsigned int nTLayerType = MV_GIGE_DEVICE|MV_USB_DEVICE);
        hikcam(size_t num);
        hikcam(const CAMERA_INFO* pstDevInfo=0);
        hikcam(const hikcam& cam);
        hikcam& operator=(hikcam cam);
        hikcam(hikcam&& cam)=default;
        int open(size_t num);
        int open(const CAMERA_INFO* pstDevInfo=0);
        inline void* get_handle(void) const
        {
            return handle;
        }
        std::any get(enum datatype type,const char* key) const;
        int set(const char* key,const std::any& value=0);
        int start(void);
        int grab_callback(void(cbOutput)(unsigned char * pData, FRAME_INFO* pFrameInfo, void* pUser),void* pUser=0);
        std::any grab(unsigned int ms=-1); //Return FRAME
        int free(std::any frame);
        int stop(void);
        int close(void);
        int destroy(void);
        ~hikcam();
    };
}

#endif
