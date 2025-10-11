#ifndef _CAMERA_H_
#define _CAMERA_H_

namespace camera
{
    enum datatype {INT,ENUM,FLOAT,BOOL,STRING,COMMAND};
    template<class DEV_TYPE,class FRAME_TYPE,class GET_TYPE,class SET_TYPE>
    class camera
    {
    protected:
        bool is_opened;
    public:
        camera():is_opened(false){}
        camera(const camera& cam);
        camera& operator=(const camera& cam);
        #if __cplusplus >= 201103L
        camera(camera&& cam)=default;
        camera& operator=(camera&& cam)=default;
        #endif
        virtual int open(const DEV_TYPE* dev)=0;
        inline bool isopened(void) const
        {
            return is_opened;
        }
        virtual const char* type(void) const
        {
            return "Camera";
        }
        virtual GET_TYPE get(enum datatype type,const char* strKey) const=0;
        virtual int set(enum datatype type,const char* strKey,const SET_TYPE& value)=0;
        virtual int start(void)=0;
        virtual FRAME_TYPE grab(unsigned int nMsec)=0;
        virtual int stop(void)=0;
        virtual int close(void)=0;
        virtual ~camera(){}
    };
}

#endif
