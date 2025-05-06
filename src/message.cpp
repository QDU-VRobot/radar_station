#include "serial.h"
#include "common.h"
#include "RM_CRC.h"
#include <stdio.h>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>
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
			struct referee_warning referee_warning;
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
volatile struct serial_info serial_info;
static volatile cv::Point2f pos_from_sentry[6];
static volatile uint8_t count_remaining[6]={0}; //cache for a while
extern std::vector<std::vector<uint8_t>> robot_classes;//HERO=0,ENGINEER,INFANTRY3,INFANTRY4,INFANTRY5,SENTRY
void receive(serial::terminal& sport)
{
	union frame buf;
	int state;
	while((mode&8)==8)
	{
		while(sport.get_fd()>=0)
		{
			state=sport.read(&buf.header,1);
			if(state<0)
				break;
			if(buf.header.SOF!=0xA5)
			{
				#ifndef NDEBUG
				fprintf(stderr,"Skipping non-header data: %02hhx\n",buf.header.SOF);
				#endif
				continue;
			}
			state=sport.read_wCRC8(&buf.header,sizeof(buf.header),1);
			if(state!=0&&state<static_cast<int>(sizeof(buf.header)-1))
				break;
			if(buf.header.data_length>MAX_BUF-(sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(frame_tail)))
				buf.header.data_length=0;
			if(state||!buf.header.data_length)
			{
				sport.read(&buf.data,sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail));
				continue;
			}
			#ifndef NDEBUG
			fprintf(stderr,"Received header: ");
			for(size_t i=0;i<sizeof(buf.header);++i)
				fprintf(stderr,"%02hhx ",buf.data[i]);
			fprintf(stderr,"\n");
			#endif
			state=sport.read_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail),sizeof(buf.header));
			if(state!=0&&state<static_cast<int>(sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
				break;
			if(state)
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
					printf("Robot %hhu is %s %hhu time(s)!\n",buf.referee_warning.offending_robot_id,buf.referee_warning.level>=3?"punished":"warned",buf.referee_warning.count);
					break;
				case 0x0201:
					printf("Robot ID: %hhu (%s)\n",buf.robot_status.robot_id,buf.robot_status.robot_id>=100?"Blue":"Red");
					serial_info.robot_id=buf.robot_status.robot_id;
					break;
				case 0x020C:
					printf("Marked: ");
					for(uint8_t i=1<<4;i>0;i>>=1)
						printf("%c",serial_info.mark_progress&i?'1':'0');
					printf("\n");
					serial_info.mark_progress=buf.mark_progress;
					break;
				case 0x020E:
					printf("Double damages: %hhu%c\n",buf.radar_info&3,buf.radar_info&4?'*':' ');
					serial_info.more_damage=buf.radar_info&3,serial_info.damaging=buf.radar_info&4;
					break;
				case 0x0301:
					printf("Received data from another robot:\ncmd_id: %#06hx sender_id: %#06hx ",buf.rheader.data_cmd_id,buf.rheader.sender_id);
					#ifndef NDEBUG
					printf("data: ");
					for(size_t i=sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(buf.rheader);i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length;++i)
						printf("%02hhx ",buf.data[i]);
					#endif
					printf("\n");
					switch(buf.rheader.data_cmd_id)
					{
						case 0x0279:
							for(size_t i=0;i<6;++i)
								if(buf.robot_pos.data[i*2]>0&&buf.robot_pos.data[i*2+1]>0)
								{
									count_remaining[i]=0; //Make sure the pos being modified is not used
									printf("Robot position updated (%zu)\n",i);
									pos_from_sentry[i].x=buf.robot_pos.data[i*2],pos_from_sentry[i].y=buf.robot_pos.data[i*2+1];
									count_remaining[i]=5;
								}
							break;
					}
					break;
				#ifndef NDEBUG
					default:
						fprintf(stderr,"Received an unhandled packet, cmd_id: %#06hx\n",buf.cmd_id);
						break;
				#endif
			}
		}
		if(sport.get_fd()>=0)
		{
			perror("Fatal error detected, re-enabling serial port");
			while(!open_serial(sport))
				std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}
uint8_t send_pos(serial::terminal& sport,const std::vector<cv::Point2f>& center_points,const std::vector<uint8_t>& ids, bool* lock)
{
	static uint8_t seq=0;
	union frame buf={{{0x5A,sizeof(union map_robot),seq++},0x0305,{.map_robot={.data={0}}}}};
	uint8_t team=serial_info.robot_id/100,ret=0;
	union robot_pos pos={.data={0}};
	size_t count=0;
	static uint8_t damaged=0;
	static std::chrono::steady_clock::time_point last_time=std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point time=std::chrono::steady_clock::now();
	for(size_t i=0;i<6;++i)
		if(count_remaining[i]&&--count_remaining[i])
			pos.data[i*2]=pos_from_sentry[i].x,pos.data[i*2+1]=pos_from_sentry[i].y,++count;
	for(size_t i=0,id;i<ids.size();++i)
		if(ids[i]!=-1&&(id=std::find(robot_classes[team].begin(),robot_classes[team].end(),ids[i])-robot_classes[team].begin())<6&&!count_remaining[id])
			pos.data[id*2]=center_points[i].x,pos.data[id*2+1]=center_points[i].y,++count; //Convert unit (m->cm)
	if(count>0)
	{
		if(std::chrono::duration_cast<std::chrono::milliseconds>(time-last_time).count()>=100)
		{
			last_time=time;
			for(size_t i=0;i<sizeof(union robot_pos)/sizeof(float);++i)
				buf.map_robot.data[i]=pos.data[i]*100;
			Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
			if(sport.write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
			{
				if(lock)
					*lock=false;
				return ret;
			}
			fprintf(stderr,"Sent data to server ");
			#ifndef NDEBUG
			for(size_t i=0;i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);++i)
				fprintf(stderr,"%02hhx ",buf.data[i]);
			#endif
			fprintf(stderr,"\n");
			ret|=1;
			buf.header.seq=seq++;
			buf.header.data_length=sizeof(buf.rheader)+sizeof(buf.robot_pos);
			buf.cmd_id=0x0301;
			buf.rheader.data_cmd_id=0x0297,buf.rheader.sender_id=serial_info.robot_id,buf.rheader.receiver_id=team*100+7;
			buf.robot_pos=pos;
			Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
			if(sport.write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
			{
				if(lock)
					*lock=false;
				return ret;
			}
			fprintf(stderr,"Sent data to robot ");
			#ifndef NDEBUG
			for(size_t i=0;i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);++i)
				fprintf(stderr,"%02hhx ",buf.data[i]);
			#endif
			fprintf(stderr,"\n");
			ret|=4;
		}
		if(count>=3&&!serial_info.damaging&&damaged<serial_info.more_damage)
		{
			buf.header.seq=seq++;
			buf.header.data_length=sizeof(buf.rheader)+sizeof(buf.radar_cmd);
			buf.cmd_id=0x0301;
			buf.rheader.data_cmd_id=0x0121,buf.rheader.sender_id=serial_info.robot_id,buf.rheader.receiver_id=0x8080;
			buf.radar_cmd=++damaged;
			Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
			if(sport.write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
			{
				if(lock)
					*lock=false;
				return ret;
			}
			fprintf(stderr,"Applying more damages ");
			#ifndef NDEBUG
			for(size_t i=0;i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);++i)
				fprintf(stderr,"%02hhx ",buf.data[i]);
			#endif
			fprintf(stderr,"\n");
			ret|=2;
		}
	}
	if(lock)
		*lock=false;
	return ret;
}
