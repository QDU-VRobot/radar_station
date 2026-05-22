#ifndef _IRAYPLE_H_
#define _IRAYPLE_H_

#include "camera.h"
#include "IMVApi.h"
namespace camera
{
    union irayple_get
    {
        int64_t intValue;
        double doubleValue;
        bool bValue;
        uint64_t enumValue;
        IMV_String stringValue;
    };
    class irayple : public camera
    {
    private:
        IMV_HANDLE handle;
    public:
        typedef IMV_DeviceList CAMERA_INFO_LIST;
        typedef IMV_DeviceInfo CAMERA_INFO;
        typedef IMV_Frame FRAME;
        enum { OK = IMV_OK };
        static int last_error;
        static CAMERA_INFO_LIST enum_cameras(unsigned int interfaceType = interfaceTypeAll);
        irayple(size_t num);
        irayple(const CAMERA_INFO* pInfo = 0);
        irayple(const irayple& cam);
        irayple& operator=(irayple cam);
        irayple(irayple&& cam) = default;
        int open(size_t num);
        int open(const char* serialNo);
        int open(const CAMERA_INFO* pInfo = 0);
        inline IMV_HANDLE get_handle() const { return handle; }
        std::any get(enum datatype type, const char* key) const;
        int set(const char* key, const std::any& value = 0);
        int start(void);
        int grab_callback(void(*callBack)(FRAME* pFrame, void* pUser), void* pUser=0);
        std::any grab(unsigned int ms = -1);
        int free(std::any frame);
        int stop(void);
        int close(void);
        int destroy(void);
        ~irayple();
    };
}

#endif
