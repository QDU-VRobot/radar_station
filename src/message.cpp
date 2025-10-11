#include "message.h"
#include "common.h"
#include "serial.h"
#include "RM_CRC.h"
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <thread>
const union rm::frame rm::msg::buf_temp={{{0xA5,sizeof(union map_robot)},0x0305,{.map_robot={.data={0}}}}};
void rm::msg::receive(void)
{
	int state;
	while(sport.get_fd()>=0) //Loop if not closed manually
	{
		while(sport.get_fd()>=0) //Loop unless an error occurred
		{
			state=sport.read(&buf.header,1); //Read SOF
			if(state<0)
				break;
			if(buf.header.SOF!=buf_temp.header.SOF)
			{
				#ifndef NDEBUG
				fprintf(stderr,"Skipping non-header data: %02hhx\n",buf.header.SOF);
				#endif
				continue;
			}
			state=sport.read_wCRC8(&buf.header,sizeof(buf.header),1); //Read the rest of header
			if(state!=0&&state<static_cast<int>(sizeof(buf.header)-1))
				break;
			if(buf.header.data_length>MAX_BUF-(sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(frame_tail)))
				buf.header.data_length=0; //Prevent overflow
			if(state||!buf.header.data_length)
			{
				sport.read(&buf.data,sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)); //Skip wrong data
				continue;
			}
			#ifndef NDEBUG
			fprintf(stderr,"Received header:");
			for(size_t i=0;i<sizeof(buf.header);++i)
				fprintf(stderr," %02hhx",buf.data[i]);
			fprintf(stderr,"\n");
			#endif
			state=sport.read_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail),sizeof(buf.header));
			if(state!=0&&state<static_cast<int>(sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
				break;
			if(state)
			{
				#ifndef NDEBUG
				fprintf(stderr,"Received a wrong packet, cmd_id: 0x%04hx\n",buf.cmd_id);
				#endif
				continue;
			}
			switch(buf.cmd_id)
			{
				case 0x0001:
					if(buf.game_status.game_progress!=serial_info.game_progress)
						printf("Game state changed to: %hhu\n",buf.game_status.game_progress);
					printf("Timer: %hu:%hu\n",buf.game_status.stage_remain_time/60,buf.game_status.stage_remain_time%60);
					if(buf.game_status.game_progress==4)
					{
						if(buf.game_status.stage_remain_time<90&&damaged<1||buf.game_status.stage_remain_time<50&&damaged<2)
						{
							++damaged;
							printf("Double vulnerabilitiy triggered due to timeout\n");
						}
					}
					else damaged=0,cmd=0;
					serial_info.game_progress=buf.game_status.game_progress;
					if(buf.game_status.game_progress!=5)
						serial_info.stage_remain_time=buf.game_status.stage_remain_time;
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
					serial_info.stage_remain_time=buf.winner;
					break;
				case 0x0003:
					printf("Robot HP:");
					for(size_t i=0;i<8;++i)
						printf(" %hu",buf.game_robot_HP.red_HP[i]);
					printf(";");
					for(size_t i=0;i<8;++i)
						printf(" %hu",buf.game_robot_HP.blue_HP[i]);
					printf("\n");
					if(serial_info.game_progress==4&&((buf.game_robot_HP.red_HP[6]==0||buf.game_robot_HP.blue_HP[6]==0)&&damaged<1||(buf.game_robot_HP.red_HP[7]<=2500||buf.game_robot_HP.blue_HP[7]<=2500)&&damaged<2))
					{
						++damaged;
						printf("Double vulnerabilitiy triggered due to low HP\n");
					}
					if(serial_info.robot_id/100)
						serial_info.our_outpost_base_HP=buf.game_robot_HP.blue_HP[6]+buf.game_robot_HP.blue_HP[7],serial_info.oppo_outpost_base_HP=buf.game_robot_HP.red_HP[6]+buf.game_robot_HP.red_HP[7];
					else
						serial_info.our_outpost_base_HP=buf.game_robot_HP.red_HP[6]+buf.game_robot_HP.red_HP[7],serial_info.oppo_outpost_base_HP=buf.game_robot_HP.blue_HP[6]+buf.game_robot_HP.blue_HP[7];
					break;
				case 0x0104:
					printf("Robot %hhu is %s %hhu time(s)!\n",buf.referee_warning.offending_robot_id,buf.referee_warning.level>=3?"punished":"warned",buf.referee_warning.count);
					break;
				case 0x0105:
					static uint8_t pre_dart_target=0;
					if(buf.dart_info.dart_target!=pre_dart_target)
					{
						printf("Dart target switched\n");
						pre_dart_target=buf.dart_info.dart_target;
						if(serial_info.game_progress==4&&serial_info.more_damage>0)
						{
							++damaged;
							printf("Double vulnerabilitiy triggered manually\n");
						}
					}
					break;
				case 0x0201:
					printf("Robot ID: %hhu (%s)\n",buf.robot_status.robot_id,buf.robot_status.robot_id/100?"Blue":"Red");
					serial_info.robot_id=buf.robot_status.robot_id;
					break;
				case 0x020C:
					printf("Marked: ");
					for(uint8_t i=1;i<=1<<4;i<<=1)
						printf("%c",serial_info.mark_progress&i?'1':'0');
					printf("\n");
					serial_info.mark_progress=buf.mark_progress;
					break;
				case 0x020E:
					printf("Double vulnerabilities: %hhu%c\n",buf.radar_info&3,buf.radar_info&4?'*':' ');
					if(serial_info.more_damage<(buf.radar_info&3))
						++cmd;
					serial_info.more_damage=buf.radar_info&3,serial_info.damaging=buf.radar_info&4;
					break;
				case 0x0301:
					printf("Received data from another robot:\ncmd_id: 0x%04hx sender_id: 0x%04hx ",buf.rheader.data_cmd_id,buf.rheader.sender_id);
					#ifndef NDEBUG
					printf("data:");
					for(size_t i=sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(buf.rheader);i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length;++i)
						printf(" %02hhx",buf.data[i]);
					#endif
					printf("\n");
					switch(buf.rheader.data_cmd_id)
					{
						case 0x0279: //Receive pos from sentry
							for(size_t i=0;i<6;++i)
								if(buf.robot_pos.data[i*2]>0&&buf.robot_pos.data[i*2+1]>0)
								{
									count_remaining[i]=0; //Make sure the pos being modified is not used
									printf("Robot position updated (%zu)\n",i);
									pos_from_sentry[i].x=buf.robot_pos.data[i*2],pos_from_sentry[i].y=buf.robot_pos.data[i*2+1];
									count_remaining[i]=25; //Override radar's pos for 1 sec
								}
							break;
					}
					break;
				#ifndef NDEBUG
					default:
						fprintf(stderr,"Received an unhandled packet, cmd_id: 0x%04hx\n",buf.cmd_id);
						break;
				#endif
			}
		}
		if(sport.get_fd()>=0)
		{
			fprintf(stderr,"Fatal error detected, re-enabling serial port\n");
			while(!open_serial(sport))
				std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}
uint8_t rm::msg::send_pos(const std::vector<cv::Point2f>& center_points,const std::vector<uint8_t>& ids)
{
	if(locked)
		return -1;
	else locked=true;
	union frame buf=buf_temp;
	uint8_t team=serial_info.robot_id/100,ret=0;
	union robot_pos pos={.data={0}};
	size_t count=0;
	std::chrono::steady_clock::time_point time=std::chrono::steady_clock::now();
	if(std::chrono::duration_cast<std::chrono::milliseconds>(time-last_time).count()<40) //About 25Hz
	{
		locked=false; //Unlock if lock available
		return ret;
	}
	for(size_t i=0;i<6;++i)
		if(count_remaining[i]&&--count_remaining[i])
			pos.data[i*2]=pos_from_sentry[i].x,pos.data[i*2+1]=pos_from_sentry[i].y,++count; //Override pos with sentry's
	for(size_t i=0,id;i<ids.size();++i)
		if(ids[i]!=-1&&(id=std::find(robot_classes[team].begin(),robot_classes[team].end(),ids[i])-robot_classes[team].begin())<6&&!count_remaining[id])
		{
			pos.data[id*2]=center_points[i].x,pos.data[id*2+1]=center_points[i].y,++count;
			#ifndef NDEBUG
			fprintf(stderr,"Robot %zu: (%.2f,%.2f)\n",id,center_points[i].x,center_points[i].y);
			#endif
		}
	if(count>0)
	{
		last_time=time;
		//Send pos
		buf.header.seq=seq++;
		for(size_t i=0;i<sizeof(union robot_pos)/sizeof(float);++i)
			buf.map_robot.data[i]=MAX(pos.data[i],0)*100; //Convert unit (m->cm) and ensure not overflow
		Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
		if(sport.write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
		{
			locked=false; //Unlock if lock available
			return ret;
		}
		fprintf(stderr,"Sent data to server ");
		#ifndef NDEBUG
		for(size_t i=0;i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);++i)
			fprintf(stderr,"%02hhx ",buf.data[i]);
		#endif
		fprintf(stderr,"\n");
		ret|=1; //Complete, return bit 0 set to 1
		if(serial_info.game_progress==4&&serial_info.more_damage>0)
		{
			//Apply double vulnerabilitiy
			buf.header.seq=seq++;
			buf.header.data_length=sizeof(buf.rheader)+sizeof(buf.radar_cmd);
			buf.cmd_id=0x0301;
			buf.rheader.data_cmd_id=0x0121,buf.rheader.sender_id=serial_info.robot_id,buf.rheader.receiver_id=0x8080;
			buf.radar_cmd=MIN(damaged,cmd);
			Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
			if(sport.write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
			{
				locked=false;
				return ret;
			}
			fprintf(stderr,"Sent radar cmd ");
			#ifndef NDEBUG
			for(size_t i=0;i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);++i)
				fprintf(stderr,"%02hhx ",buf.data[i]);
			#endif
			fprintf(stderr,"\n");
			ret|=2;
		}
		//Send to another robot
		buf.header.seq=seq++;
		buf.header.data_length=sizeof(buf.rheader)+sizeof(buf.robot_pos);
		buf.cmd_id=0x0301;
		buf.rheader.data_cmd_id=0x0297,buf.rheader.sender_id=serial_info.robot_id,buf.rheader.receiver_id=team*100+7;
		buf.robot_pos=pos;
		Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(&buf.header),sizeof(buf.header));
		if(sport.write_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail))<static_cast<int>(sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))
		{
			locked=false;
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
	locked=false;
	return ret;
}
