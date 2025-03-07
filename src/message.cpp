#include "serial.h"
#include "common.h"
#include "RM_CRC.h"
#include <cstddef>
#include <stdio.h>
#include <algorithm>
#include <string>
#define MAX_BUF 256
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
struct referee_warning
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
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
			struct referee_warning referee_warning;
			struct robot_status robot_status;
			uint8_t mark_progress;
			uint8_t radar_info;
			struct
			{
				struct robot_interaction_data_header rheader;
				uint8_t radar_cmd;
			};
			union map_robot map_robot;
		};
	};
	uint8_t data[MAX_BUF];
};
typedef uint16_t frame_tail;
#pragma pack()
struct serial_info serial_info;
extern std::vector<std::vector<uint8_t>> robot_classes;//HERO=0,ENGINEER,INFANTRY3,INFANTRY4,INFANTRY5,SENTRY
void receive(serial::terminal* sport,const std::string& filename)
{
	union frame buf;
	int state;
	while(sport->get_fd()>=0||(state=!open_serial(*sport,filename.c_str())))
	{
		state=sport->read_wCRC8(&buf.header,sizeof(buf.header));
		if(state!=0&&state<static_cast<int>(sizeof(buf.header)))
			break;
		if(buf.header.SOF!=0xA5)
		{
			fprintf(stderr,"Invalid header: %#0hhX\n",buf.header.SOF);
			state=sizeof(buf.header);
		}
		if(state==sizeof(buf.header))
		{
			sport->read(&buf.data,sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail));
			continue;
		}
		state=sport->read_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail),sizeof(buf.header));
		if(state!=0&&state<static_cast<int>(sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
			break;
		if(state==sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))
			continue;
		switch(buf.cmd_id)
		{
			case 0x0001:
				if(buf.game_status.game_progress!=serial_info.game_progress)
					printf("Game state changed to: %hhu\n",buf.game_status.game_progress);
				printf("Timer: %hu:%hu\n",buf.game_status.stage_remain_time/60,buf.game_status.stage_remain_time%60);
				serial_info.game_progress=buf.game_status.game_progress,serial_info.stage_remain_time=buf.game_status.stage_remain_time;
				break;
			case 0x0002:
				switch(buf.winner)
				{
					case 0:
						printf("Result: Draw!\n");
						break;
					case 1:
						printf("Result: Red wins!\n");
						break;
					case 2:
						printf("Result: Blue wins!\n");
						break;
				}
				break;
			case 0x0104:
				printf("Warning: Robot %hhu is %s %hhu time(s)!\n",buf.referee_warning.offending_robot_id,buf.referee_warning.level>=3?"punished ":"warned ",buf.referee_warning.count);
				break;
			case 0x0201:
				printf("Robot ID: %hhu (%s)\n",buf.robot_status.robot_id,buf.robot_status.robot_id>=100?"Blue":"Red");
				serial_info.robot_id=buf.robot_status.robot_id;
				break;
			case 0x020C:
				printf("Marked: ");
				for(uint8_t i=1<<5;i>0;i>>=1)
					printf("%c",serial_info.mark_progress&i?'1':'0');
				serial_info.mark_progress=buf.mark_progress;
				break;
			case 0x020E:
				printf("Double damages: %hhu%c\n",buf.radar_info&3,buf.radar_info&4?'*':' ');
				serial_info.more_damage=buf.radar_info&3,serial_info.damaging=buf.radar_info&4;
				break;
			case 0x0301:
				printf("Received data from another robot:\ncmd_id: %#0hx sender_id: %#0hx\ndata: ",buf.rheader.data_cmd_id,buf.rheader.sender_id);
				for(size_t i=sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(buf.rheader);i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length;++i)
					printf("%0hhX ",buf.data[i]);
				printf("\n");
				break;
		}
	}
	if(state!=0)
	{
		perror("Fatal error detected, disabling serial port");
		sport->~terminal();
		mode&=7;
	}
}
uint8_t send_pos(serial::terminal* sport,const std::vector<cv::Point2f>& center_points,const std::vector<uint8_t>& ids, bool* lock)
{
	union frame buf={{{0x5A,sizeof(union map_robot),0},0x0305,{.map_robot={.data={0}}}}};
	uint8_t team=serial_info.robot_id/100;
	size_t count=0;
	static uint8_t damaged=0;
	for(size_t i=0,id;i<ids.size();++i)
		if(ids[i]!=-1&&(id=std::find(robot_classes[team].begin(),robot_classes[team].end(),ids[i])-robot_classes[team].begin())<6)
			buf.map_robot.data[id*2]=center_points[i].x*100,buf.map_robot.data[id*2+1]=center_points[i].y*100,++count; //Convert unit (m->cm)
	Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
	if(sport->write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
	{
		if(lock)
			*lock=false;
		return 0;
	}
	uint8_t ret;
	if(count>=3&&!serial_info.damaging&&damaged<serial_info.more_damage)
	{
		buf.header.data_length=sizeof(buf.rheader)+sizeof(buf.radar_cmd);
		buf.cmd_id=0x0301;
		buf.rheader.data_cmd_id=0x0121,buf.rheader.sender_id=serial_info.robot_id,buf.rheader.receiver_id=0x8080;
		buf.radar_cmd=++damaged;
		Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
		if(sport->write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
			ret=1;
		else ret=2;
	}
	else ret=1;
	if(lock)
		*lock=false;
	return ret;
}
