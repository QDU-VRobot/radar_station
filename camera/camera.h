#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stddef.h>
#include <any>
namespace camera
{
    enum datatype {INT,ENUM,FLOAT,BOOL,STRING,COMMAND};
    class camera
    {
    protected:
        bool is_opened;
    public:
        camera():is_opened(false){}
        virtual int open(size_t num)=0;
        inline bool isopened(void) const
        {
            return is_opened;
        }
        virtual std::any get(enum datatype type,const char* key) const=0;
        virtual int set(const char* key,const std::any& value)=0;
        virtual int start(void)=0;
        virtual std::any grab(unsigned int ms)=0;
        virtual int free(std::any frame)=0;
        virtual int stop(void)=0;
        virtual int close(void)=0;
        virtual ~camera()=default;
    };
}

#endif
