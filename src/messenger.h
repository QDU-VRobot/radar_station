#ifndef _MESSENGER_H_
#define _MESSENGER_H_

#include "serial.h"
#include <chrono>
#include <stdint.h>
#include <vector>
#include <thread>
#include "opencv2/core.hpp"
#define MAX_BUF 256
namespace rm
{
    //See RoboMaster file
    #pragma pack(push, 1)
    struct header
    {
        uint8_t SOF;
        uint16_t data_length;
        uint8_t seq;
        uint8_t CRC8;
    };
    struct game_status
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    };
    union game_robot_HP
    {
        uint16_t ally_HP[8]; //123457OB
        uint16_t opponent_HP[6]; //1234_7
    };
    struct referee_warning
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    };
    struct dart_info
    {
        uint8_t dart_remaining_time;
        uint8_t dart_shot : 3;
        uint8_t dart_count : 3;
        uint8_t dart_target : 3;
        uint8_t reserved;
    };
    struct robot_status
    {
        uint8_t robot_id;
        //Unused below
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
    };
    struct radar_mark_data
    {
        uint8_t foe_mark : 6;
        uint8_t ally_mark : 6;
        uint8_t reserved : 4;
    };
    struct radar_info
    {
        uint8_t vulnerability : 2;
        uint8_t vulnerating : 1;
        uint8_t encrypt_level : 2;
        uint8_t encrypt_changeable : 1;
        uint8_t reserved : 2;
    };
    union map_robot
    {
        struct
        {
            uint16_t hero_position_x;
            uint16_t hero_position_y;
            uint16_t engineer_position_x;
            uint16_t engineer_position_y;
            uint16_t infantry_3_position_x;
            uint16_t infantry_3_position_y;
            uint16_t infantry_4_position_x;
            uint16_t infantry_4_position_y;
            uint16_t infantry_5_position_x;
            uint16_t infantry_5_position_y;
            uint16_t sentry_position_x;
            uint16_t sentry_position_y;
        };
        uint16_t data[12];
    };
    union both_map_robot
    {
        struct
        {
            union map_robot opponent;
            union map_robot ally;
        };
        uint16_t data[24];
    };
    //Communicate with sentry
    union robot_pos
    {
        struct
        {
            float hero_x;
            float hero_y;
            float engineer_x;
            float engineer_y;
            float standard_3_x;
            float standard_3_y;
            float standard_4_x;
            float standard_4_y;
            float standard_5_x;
            float standard_5_y;
            float sentry_x;
            float sentry_y;
        };
        float data[12];
    };
    union both_robot_pos
    {
        struct
        {
            union robot_pos opponent;
            union robot_pos ally;
        };
        float data[24];
    };
    struct robot_interaction_data
    {
        struct
        {
            uint16_t data_cmd_id;
            uint16_t sender_id;
            uint16_t receiver_id;
        } header;
        union
        {
            struct
            {
                uint8_t radar_cmd;
                uint8_t password_cmd;
                uint8_t password[6];
            } cmd;
            union robot_pos robot_pos;
        };
    };
    union frame
    {
        struct
        {
            struct header header;
            uint16_t cmd_id;
            union
            {
                struct game_status game_status;
                uint8_t winner;
                union game_robot_HP game_robot_HP;
                struct referee_warning referee_warning;
                struct dart_info dart_info;
                struct robot_status robot_status;
                struct radar_mark_data radar_mark_data;
                struct radar_info radar_info;
                struct robot_interaction_data robot_interaction_data;
                union map_robot map_robot;
            };
        };
        uint8_t data[MAX_BUF];
    };
    typedef uint16_t frame_tail;
    #pragma pack(pop)
    struct serial_info //See RoboMaster doc
    {
        uint16_t stage_remain_time;
        uint16_t ally_outpost_base_HP;
        uint16_t foe_outpost_base_HP;
        uint8_t robot_id;
        uint8_t game_progress:3;
        uint8_t foe_mark:6;
        uint8_t ally_mark:6;
        uint8_t vulnerability:2;
        bool vulnerating:1;
        uint8_t encrypt_level:2;
        bool encrypt_changeable:1;
    };
    class msg: public serial::terminal
    {
        union frame recv_buf;
        std::chrono::steady_clock::time_point timestamp[3];
        volatile uint8_t cmd;
        uint8_t seq;
        bool running;
        std::thread rx,tx;
        void referee_callback(void);
        void receive(void);
        void poll(void);
    public:
        using timer=std::chrono::steady_clock;
        static std::vector<std::vector<uint8_t>> robot_classes; //HERO=0,ENGINEER,INFANTRY3,INFANTRY4,INFANTRY5,SENTRY
        static std::chrono::milliseconds interval;
        static bool persistent_cache; //Send robot's last pos if lost, and guess drone's pos, instead of ignoring
        volatile struct serial_info serial_info;
        std::pair<std::vector<cv::Point2f>, std::vector<uint8_t>> pos_from_camera;
        volatile union both_robot_pos pos_from_sentry;
        volatile union map_robot pos_from_radio;
        volatile int8_t pos_cache[12]; //Cache control, positive for sentry, negative for radio and zero for video
        uint8_t guessed_password[6];
        volatile uint8_t password_retries;
        volatile uint8_t vulnerated; //Double vulnerability trigger
        msg():timestamp{timer::now(),timer::now(),timer::now()},cmd(0),seq(0),pos_cache{0},guessed_password{0},password_retries(0),vulnerated(0){}
        msg(serial::terminal&& s):serial::terminal(s),timestamp{timer::now(),timer::now(),timer::now()},cmd(0),seq(0),pos_cache{0},guessed_password{0},password_retries(0),vulnerated(0){}
        msg(const msg&)=delete; //Prevents thread problems
        msg& operator=(const msg&)=delete;
        msg(msg&&)=default;
        msg& operator=(msg&&)=default;
        static bool serial_quick_setup(serial::terminal& port, const char* filename=0);
        static std::vector<uint8_t> generate_password(size_t len);
        bool referee_send(uint16_t cmd_id, uint16_t data_length, const void* data);
        bool run(const char* filename=0);
        void stop(void);
        ~msg();
    };
}

#endif
