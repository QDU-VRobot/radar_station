#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include "serial.h"
#include <chrono>
#include <vector>
#include "opencv2/core.hpp"
#define MAX_BUF 256
namespace rm
{
    //See RoboMaster file
    #pragma pack(1)
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
    struct game_robot_HP
    {
        uint16_t red_HP[8]; //123457OB
        uint16_t blue_HP[8];
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
        uint8_t dart_target : 2;
        uint8_t reserved;
    };
    struct robot_status
    {
        uint8_t robot_id;
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
    struct robot_interaction_data_header
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
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
                struct game_robot_HP game_robot_HP;
                struct referee_warning referee_warning;
                struct dart_info dart_info;
                struct robot_status robot_status;
                uint8_t mark_progress;
                uint8_t radar_info;
                struct
                {
                    struct robot_interaction_data_header rheader;
                    union
                    {
                        uint8_t radar_cmd;
                        union robot_pos robot_pos;
                    };
                };
                union map_robot map_robot;
            };
        };
        uint8_t data[MAX_BUF];
    };
    typedef uint16_t frame_tail;
    #pragma pack()
    struct serial_info //See RoboMaster doc
    {
        uint16_t stage_remain_time;
        uint16_t our_outpost_base_HP;
        uint16_t oppo_outpost_base_HP;
        uint8_t robot_id;
        uint8_t mark_progress:5; //Robot 12347
        uint8_t game_progress:3;
        uint8_t more_damage:2;
        bool damaging:1;
    };
    class msg
    {
        serial::terminal& sport;
        union frame buf;
        std::chrono::steady_clock::time_point last_time;
        volatile uint8_t cmd;
        volatile bool locked; //Simple mutex if not too frequent
        uint8_t seq;
    public:
        static std::vector<std::vector<uint8_t>> robot_classes; //HERO=0,ENGINEER,INFANTRY3,INFANTRY4,INFANTRY5,SENTRY
        static const union frame buf_temp;
        volatile struct serial_info serial_info;
        volatile cv::Point2f pos_from_sentry[6];
        volatile uint8_t count_remaining[6]; //cache for a while
        volatile uint8_t damaged; //Double vulnerabilitiy trigger
        msg(serial::terminal& s):sport(s),last_time(std::chrono::steady_clock::now()),cmd(0),seq(0),locked(false),count_remaining({0}),damaged(0){}
        void receive(void);
        uint8_t send_pos(const std::vector<cv::Point2f>& center_points,const std::vector<uint8_t>& ids);
    };
}

#endif
