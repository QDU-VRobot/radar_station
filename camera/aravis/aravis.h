#ifndef _ARAVIS_H_
#define _ARAVIS_H_

#include "camera.h"
#include "arv.h"
#include <memory>
namespace camera
{
    class aravis : public camera
    {
        std::unique_ptr<ArvCamera,decltype(&g_object_unref)> handle;
        std::unique_ptr<ArvStream,decltype(&g_object_unref)> stream;
        ArvBuffer* buffer;
    public:
        static GError* last_error;
        static void enum_cameras(void);
        aravis(size_t index);
        aravis(const char* serial = 0);
        aravis(aravis&& cam) = default;
        aravis& operator=(aravis&& cam) = default;
        inline const void* get_handle(void) const{return handle.get();}
        int open(size_t index = 0);
        int open(const char* serial);
        values get(enum value_type type,const char* key) const;
        int set(const char* key,const values& value);
        int start(void);
        int grab_callback(ArvStreamCallback callback, void* user_data=0);
        std::tuple<const void*,enum pixel_format,unsigned,unsigned> grab(unsigned int ms=-1);
        int free(void);
        int stop(void);
        int close(void);
        ~aravis();
    };
}

#endif
