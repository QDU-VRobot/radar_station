#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stddef.h>
#include <stdint.h>
#include <variant>
namespace camera
{
    enum value_type {INT,ENUM,FLOAT,BOOL,STRING,COMMAND};
    enum pixel_format : uint32_t
    {
        PIXEL_FORMAT_NONE             = 0u,

        // ── Mono ──────────────────────────────────────────────
        PIXEL_FORMAT_MONO_8           = 0x01080001u,
        PIXEL_FORMAT_MONO_8_SIGNED    = 0x01080002u,
        PIXEL_FORMAT_MONO_10          = 0x01100003u,
        PIXEL_FORMAT_MONO_10_PACKED   = 0x010c0004u,
        PIXEL_FORMAT_MONO_12          = 0x01100005u,
        PIXEL_FORMAT_MONO_12_PACKED   = 0x010c0006u,
        PIXEL_FORMAT_MONO_16          = 0x01100007u,
        PIXEL_FORMAT_MONO_14          = 0x01100025u,

        // ── Bayer 8-bit ───────────────────────────────────────
        PIXEL_FORMAT_BAYER_GR_8       = 0x01080008u,
        PIXEL_FORMAT_BAYER_RG_8       = 0x01080009u,
        PIXEL_FORMAT_BAYER_GB_8       = 0x0108000au,
        PIXEL_FORMAT_BAYER_BG_8       = 0x0108000bu,

        // ── Bayer 10-bit ──────────────────────────────────────
        PIXEL_FORMAT_BAYER_GR_10      = 0x0110000cu,
        PIXEL_FORMAT_BAYER_RG_10      = 0x0110000du,
        PIXEL_FORMAT_BAYER_GB_10      = 0x0110000eu,
        PIXEL_FORMAT_BAYER_BG_10      = 0x0110000fu,

        // ── Bayer 10-bit Packed (legacy) ──────────────────────
        PIXEL_FORMAT_BAYER_GR_10_PACKED = 0x010c0026u,
        PIXEL_FORMAT_BAYER_RG_10_PACKED = 0x010c0027u,
        PIXEL_FORMAT_BAYER_GB_10_PACKED = 0x010c0028u,
        PIXEL_FORMAT_BAYER_BG_10_PACKED = 0x010c0029u,

        // ── Bayer 12-bit ──────────────────────────────────────
        PIXEL_FORMAT_BAYER_GR_12      = 0x01100010u,
        PIXEL_FORMAT_BAYER_RG_12      = 0x01100011u,
        PIXEL_FORMAT_BAYER_GB_12      = 0x01100012u,
        PIXEL_FORMAT_BAYER_BG_12      = 0x01100013u,

        // ── Bayer 12-bit Packed (legacy) ──────────────────────
        PIXEL_FORMAT_BAYER_GR_12_PACKED = 0x010c002au,
        PIXEL_FORMAT_BAYER_RG_12_PACKED = 0x010c002bu,
        PIXEL_FORMAT_BAYER_GB_12_PACKED = 0x010c002cu,
        PIXEL_FORMAT_BAYER_BG_12_PACKED = 0x010c002du,

        // ── RGB / BGR ─────────────────────────────────────────
        PIXEL_FORMAT_RGB_8_PACKED     = 0x02180014u,
        PIXEL_FORMAT_BGR_8_PACKED     = 0x02180015u,
        PIXEL_FORMAT_RGBA_8_PACKED    = 0x02200016u,
        PIXEL_FORMAT_BGRA_8_PACKED    = 0x02200017u,
        PIXEL_FORMAT_RGB_10_PACKED    = 0x02300018u,
        PIXEL_FORMAT_BGR_10_PACKED    = 0x02300019u,
        PIXEL_FORMAT_RGB_12_PACKED    = 0x0230001au,
        PIXEL_FORMAT_BGR_12_PACKED    = 0x0230001bu,

        // ── YUV ───────────────────────────────────────────────
        PIXEL_FORMAT_YUV_411_PACKED   = 0x020c001eu,
        PIXEL_FORMAT_YUV_422_PACKED   = 0x0210001fu,
        PIXEL_FORMAT_YUV_444_PACKED   = 0x02180020u,
        PIXEL_FORMAT_YUV_422_YUYV_PACKED = 0x02100032u,

        // ── RGB Planar ────────────────────────────────────────
        PIXEL_FORMAT_RGB_8_PLANAR     = 0x02180021u,
        PIXEL_FORMAT_RGB_10_PLANAR    = 0x02300022u,
        PIXEL_FORMAT_RGB_12_PLANAR    = 0x02300023u,
        PIXEL_FORMAT_RGB_16_PLANAR    = 0x02300024u,
    };
    inline uint8_t get_bit_per_pixel(enum pixel_format f){return (f >> 16) & 0xff;}
    using values=std::variant<std::monostate,int64_t,double,bool,uint64_t,const char*>;
    class camera
    {
    protected:
        bool is_opened;
    public:
        camera():is_opened(false){}
        camera(camera&&)=default;
        camera& operator=(camera&&)=default;
        virtual int open(size_t index)=0;
        virtual int open(const char* serial)=0;
        inline bool isopened(void) const{return is_opened;}
        virtual values get(enum value_type type,const char* key) const=0;
        virtual int set(const char* key,const values& value)=0;
        virtual int start(void)=0;
        virtual std::tuple<const void*,enum pixel_format,unsigned,unsigned> grab(unsigned int ms)=0; // data, format, width, height
        virtual int free(void)=0;
        virtual int stop(void)=0;
        virtual int close(void)=0;
        virtual ~camera()=default;
    };
}

#endif
