#include "aravis.h"
#include <string.h>
#include <stdexcept>
#include <variant>
namespace camera
{
    GError* aravis::last_error = nullptr;
    aravis::aravis(size_t index): handle(nullptr), stream(nullptr), buffer(nullptr)
    {
        aravis::open(index);
    }
    aravis::aravis(const char* serial): handle(nullptr), stream(nullptr), buffer(nullptr)
    {
        if(serial)
            aravis::open(serial);
    }
    void aravis::enum_cameras(void)
    {
        arv_update_device_list();
    }
    int aravis::open(size_t index)
    {
        close();
        g_clear_error(&last_error);
        enum_cameras();
        if (arv_get_n_devices() <= index)
            throw std::runtime_error("Camera not found, check camera connection");
        handle = ArvCameraPtr(arv_camera_new(arv_get_device_id(index), &last_error));
        is_opened = last_error == nullptr;
        return last_error != nullptr;
    }
    int aravis::open(const char* serial)
    {
        close();
        g_clear_error(&last_error);
        enum_cameras();
        for (size_t i = 0; i < arv_get_n_devices(); ++i)
            if (strcmp(serial, arv_get_device_serial_nbr(i)) == 0)
            {
                handle = ArvCameraPtr(arv_camera_new(arv_get_device_id(i), &last_error));
                is_opened = last_error == nullptr;
                return last_error != nullptr;
            }
        throw std::runtime_error("Camera not found, check camera connection");
    }
    values aravis::get(enum value_type type, const char* key) const
    {
        values value;
        g_clear_error(&last_error);
        switch (type)
        {
            case INT:
                value = arv_camera_get_integer(handle.get(), key, &last_error);
            break;
            case ENUM:
            {
                const char* str = arv_camera_get_string(handle.get(), key, &last_error);
                if (last_error == nullptr) {
                    if (strcmp(key, "AcquisitionMode") == 0)
                        value = static_cast<uint64_t>(arv_acquisition_mode_from_string(str));
                    else if (strcmp(key, "ExposureMode") == 0)
                        value = static_cast<uint64_t>(arv_exposure_mode_from_string(str));
                    else if (strcmp(key, "ExposureAuto") == 0)
                        value = static_cast<uint64_t>(arv_auto_from_string(str));
                    else
                        value = static_cast<uint64_t>(arv_camera_get_integer(handle.get(), key, &last_error));
                }
            }
            break;
            case FLOAT:
                value = arv_camera_get_float(handle.get(), key, &last_error);
            break;
            case BOOL:
                value = arv_camera_get_boolean(handle.get(), key, &last_error);
            break;
            case STRING:
                value = arv_camera_get_string(handle.get(), key, &last_error);
            break;
            default:
                throw std::invalid_argument("Invalid type");
        }
        return value;
    }
    int aravis::set(const char* key, const values& value)
    {
        g_clear_error(&last_error);
        if (std::holds_alternative<std::monostate>(value))
            arv_camera_execute_command(handle.get(), key, &last_error);
        else if (const int64_t* val = std::get_if<int64_t>(&value))
            arv_camera_set_integer(handle.get(), key, *val, &last_error);
        else if (const double* val = std::get_if<double>(&value))
            arv_camera_set_float(handle.get(), key, *val, &last_error);
        else if (const bool* val = std::get_if<bool>(&value))
            arv_camera_set_boolean(handle.get(), key, *val, &last_error);
        else if (const uint64_t* val = std::get_if<uint64_t>(&value))
        {
            const char* str = nullptr;
            if (strcmp(key, "AcquisitionMode") == 0)
                str = arv_acquisition_mode_to_string(static_cast<ArvAcquisitionMode>(*val));
            else if (strcmp(key, "ExposureMode") == 0)
                str = arv_exposure_mode_to_string(static_cast<ArvExposureMode>(*val));
            else if (strcmp(key, "ExposureAuto") == 0)
                str = arv_auto_to_string(static_cast<ArvAuto>(*val));
            if (str != nullptr)
                arv_camera_set_string(handle.get(), key, str, &last_error);
            else
                arv_camera_set_integer(handle.get(), key, *val, &last_error);
        }
        else if (const char* const* val = std::get_if<const char*>(&value))
            arv_camera_set_string(handle.get(), key, *val, &last_error);
        else
            throw std::bad_variant_access();
        return last_error != nullptr;
    }
    int aravis::start(void)
    {
        g_clear_error(&last_error);
        if(!stream)
            stream = ArvStreamPtr(arv_camera_create_stream(handle.get(), nullptr, nullptr, &last_error));
        if(!ARV_IS_STREAM(stream.get()))
            return 1;
        size_t payload = arv_camera_get_payload(handle.get(), &last_error);
        if(last_error != nullptr)
            return 1;
        for(size_t i = 0; i < 2; ++i)
            arv_stream_push_buffer(stream.get(), arv_buffer_new(payload, NULL));
        arv_camera_start_acquisition(handle.get(), &last_error);
        return last_error != nullptr;
    }
    int aravis::grab_callback(ArvStreamCallback callback, void* user_data)
    {
        if(stream)
            return 1;
        g_clear_error(&last_error);
        stream = ArvStreamPtr(arv_camera_create_stream(handle.get(), callback, user_data?user_data:stream.get(), &last_error));
        return !ARV_IS_STREAM(stream.get());
    }
    std::tuple<const void*,enum pixel_format,unsigned,unsigned> aravis::grab(unsigned int ms)
    {
        free();
        buffer = arv_stream_pop_buffer(stream.get());
        return ARV_IS_BUFFER(buffer) ? std::make_tuple(arv_buffer_get_data(buffer, nullptr), static_cast<enum pixel_format>(arv_buffer_get_image_pixel_format(buffer)), arv_buffer_get_image_width(buffer), arv_buffer_get_image_height(buffer)) : std::make_tuple(nullptr, PIXEL_FORMAT_NONE, 0, 0);
    }
    int aravis::free(void)
    {
        if(buffer)
            arv_stream_push_buffer(stream.get(), buffer), buffer = nullptr;
        return 0;
    }
    int aravis::stop(void)
    {
        g_clear_error(&last_error);
        free();
        arv_camera_stop_acquisition(handle.get(), &last_error);
        stream.reset();
        return last_error != nullptr;
    }
    int aravis::close(void)
    {
        if(handle)
            stop();
        handle.reset();
        is_opened = false;
        return 0;
    }
    aravis::~aravis()
    {
        close();
    }
}
