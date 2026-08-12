#include "messenger.h"
#include "serial.h"
#include "RM_CRC.h"
#include <string.h>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <chrono>
#include <thread>
#include <random>
#define SERIAL_SPEED B115200
namespace rm
{
    std::vector<uint8_t> msg::generate_password(size_t len)
    {
        static const uint8_t hard_bytes[] = {
            0x5A, 0x55, 0x33, 0x30, 0x6F
        };
        static std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
        static std::uniform_int_distribution<size_t> dist(0, std::size(hard_bytes) - 1);
        std::vector<uint8_t> password;
        password.reserve(len);
        for(;len>0;--len)
            password.push_back(hard_bytes[dist(rng)]);
        return password;
    }
    bool msg::serial_quick_setup(serial::terminal& port, const char* filename)
    {
        if(filename)
        {
            if(port.open(filename)<0)
                return false;
        }
        else if(port.open()<0)
            return false;
        port.set_rate(SERIAL_SPEED);
        port.options.c_cflag&=~(CSIZE|CSTOPB|PARENB);
        port.options.c_cflag|=(CS8|CREAD|CLOCAL);
        port.options.c_iflag&=~(BRKINT|ICRNL|INPCK|ISTRIP|IXON);
        port.options.c_oflag&=~OPOST;
        port.options.c_lflag&=~(ICANON|ECHO|ISIG);
        port.options.c_cc[VMIN] = 32;
        port.options.c_cc[VTIME] = 2;
        return port.apply_options()==0;
    }
    void msg::referee_callback(union frame& buf)
    {
        switch(buf.cmd_id)
        {
            case 0x0001:
                if(buf.game_status.game_progress!=serial_info.game_progress)
                    printf("Game state changed to: %hhu\n",buf.game_status.game_progress);
                printf("Timer: %hu:%hu\n",buf.game_status.stage_remain_time/60,buf.game_status.stage_remain_time%60);
                if(buf.game_status.game_progress==4)
                {
                    if(buf.game_status.stage_remain_time<300&&vulnerated<1||buf.game_status.stage_remain_time<180&&vulnerated<2)
                    {
                        ++vulnerated;
                        printf("Double vulnerabilitiy triggered due to timeout\n");
                    }
                }
                else vulnerated=0,cmd=0;
                serial_info.game_progress=buf.game_status.game_progress;
                if(buf.game_status.game_progress!=5)
                    serial_info.stage_remain_time=buf.game_status.stage_remain_time;
            break;
            case 0x0002:
                switch(buf.winner)
                {
                    case 0:
                        printf("Result: Draw (or unexpectedly ended)!\n");
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
                    printf(" %hu",buf.game_robot_HP.ally_HP[i]);
                printf("\n");
                if(serial_info.game_progress==4&&(buf.game_robot_HP.ally_HP[6]<1000&&vulnerated<1||buf.game_robot_HP.ally_HP[7]<5000&&vulnerated<2))
                {
                    ++vulnerated;
                    printf("Double vulnerabilitiy triggered due to low HP\n");
                }
                serial_info.ally_outpost_base_HP=buf.game_robot_HP.ally_HP[6]+buf.game_robot_HP.ally_HP[7];
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
                    if(serial_info.game_progress==4&&serial_info.vulnerability>0)
                    {
                        ++vulnerated;
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
                for(uint8_t i=1;i<1<<6;i<<=1)
                    printf("%c",buf.radar_mark_data.foe_mark&i?'1':'0');
                printf(" ");
                for(uint8_t i=1;i<1<<6;i<<=1)
                    printf("%c",buf.radar_mark_data.ally_mark&i?'1':'0');
                printf("\n");
                serial_info.foe_mark=buf.radar_mark_data.foe_mark;
                serial_info.ally_mark=buf.radar_mark_data.ally_mark;
            break;
            case 0x020E:
                printf("Double vulnerabilities: %hhu%c\n",buf.radar_info.vulnerability,buf.radar_info.vulnerating?'*':' ');
                if(serial_info.vulnerability<buf.radar_info.vulnerability)
                    ++cmd;
                serial_info.vulnerability=buf.radar_info.vulnerability,serial_info.vulnerating=buf.radar_info.vulnerating;
                printf("Encrypt level: %hhu%c\n",buf.radar_info.encrypt_level,buf.radar_info.encrypt_changeable?'*':' ');
                if(serial_info.encrypt_level<buf.radar_info.encrypt_level)
                    password_retries=0;
                serial_info.encrypt_level=buf.radar_info.encrypt_level,serial_info.encrypt_changeable=buf.radar_info.encrypt_changeable;
            break;
            case 0x0301:
                printf("Received data from another robot:\ncmd_id: 0x%04hx sender_id: 0x%04hx ",buf.robot_interaction_data.header.data_cmd_id,buf.robot_interaction_data.header.sender_id);
                #ifndef NDEBUG
                printf("data:");
                for(size_t i=sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(buf.robot_interaction_data.header);i<sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length;++i)
                    printf(" %02hhx",buf.data[i]);
                #endif
                printf("\n");
                switch(buf.robot_interaction_data.header.data_cmd_id)
                {
                    case 0x0279: //Receive ally pos from sentry
                        for(size_t i=0;i<6;++i)
                            if(pos_cache[i+6]>=0&&buf.robot_interaction_data.robot_pos.data[i*2]>0&&buf.robot_interaction_data.robot_pos.data[i*2+1]>0)
                            {
                                pos_cache[i+6]=0; //Make sure the pos being modified is not used
                                printf("Robot position updated (%zu)\n",i);
                                pos_from_sentry.data[(i+6)*2]=buf.robot_interaction_data.robot_pos.data[i*2],pos_from_sentry.data[(i+6)*2+1]=buf.robot_interaction_data.robot_pos.data[i*2+1];
                                pos_cache[i+6]=20; //Override radar's pos for 2 secs
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
    void msg::receive(serial::stream& src, union frame& buf, bool(*reopen)(serial::stream&))
    {
        ssize_t state;
        while(running)
        {
            while(running&&src.get_fd()>=0)
            {
                state=src.read(&buf.header,1); //Read SOF
                if(state<=0)
                    break;
                if(buf.header.SOF!=0xA5)
                {
                    #ifndef NDEBUG
                    fprintf(stderr,"Skipping non-header data: %02hhx\n",buf.header.SOF);
                    #endif
                    continue;
                }
                state=src.read_wCRC8(&buf.header,sizeof(buf.header),1); //Read the rest of header
                if(state<=0)
                    break;
                else if((state-=static_cast<ssize_t>(sizeof(buf.header)-1))<0) //Read failed
                    continue;
                if(buf.header.data_length>MAX_BUF-(sizeof(buf.header)+sizeof(buf.cmd_id)+sizeof(frame_tail)))
                    buf.header.data_length=0; //Prevent overflow
                if(!state||!buf.header.data_length)
                {
                    src.read(&buf.data,sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)); //Skip wrong data
                    continue;
                }
                #ifndef NDEBUG
                fprintf(stderr,"Received header:");
                for(size_t i=0;i<sizeof(buf.header);++i)
                    fprintf(stderr," %02hhx",buf.data[i]);
                fprintf(stderr,"\n");
                #endif
                state=src.read_wCRC16(&buf.data,sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail),sizeof(buf.header));
                if(state<=0)
                    break;
                else if((state-=static_cast<ssize_t>(sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail)))<0)
                    continue;
                if(!state)
                {
                    #ifndef NDEBUG
                    fprintf(stderr,"Received a wrong packet, cmd_id: 0x%04hx\n",buf.cmd_id);
                    #endif
                    continue;
                }
                referee_callback(buf);
            }
            if(running)
            {
                fprintf(stderr,"Fatal error detected\n");
                if(reopen)
                    while(!reopen(src))
                        std::this_thread::sleep_for(std::chrono::milliseconds(250));
                else
                    running=false;
            }
        }
    }
    bool msg::referee_send(uint16_t cmd_id, uint16_t data_length, const void* data)
    {
        union frame buf={{{0xA5,data_length,++seq},cmd_id}};
        const size_t buf_len=sizeof(buf.header)+sizeof(buf.cmd_id)+buf.header.data_length+sizeof(frame_tail);
        Append_CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&buf.header),sizeof(buf.header));
        memcpy(buf.data+sizeof(buf.header)+sizeof(buf.cmd_id),data,data_length);
        #ifndef NDEBUG
        fprintf(stderr,"Sending data: ");
        for(size_t i=0;i<buf_len;++i)
            fprintf(stderr,"%02hhx ",buf.data[i]);
        fprintf(stderr,"\n");
        #endif
        return referee.write_wCRC16(&buf.data,buf_len)==static_cast<ssize_t>(buf_len);
    }
    void msg::poll(void)
    {
        while(running)
        {
            while(referee.get_fd()<0||!serial_info.robot_id)
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
            uint8_t team=serial_info.robot_id/100;
            union both_robot_pos pos={.data={0}};
            timer::time_point target=timer::now()+interval;
            if(timer::now()>timestamp[0])
            {
                size_t count=0;
                for(size_t i=0;i<sizeof(pos_cache)/sizeof(int8_t);++i)
                    if(pos_cache[i]>0&&--pos_cache[i])
                        pos.data[i*2]=pos_from_sentry.data[i*2],pos.data[i*2+1]=pos_from_sentry.data[i*2+1],++count; //Override pos with sentry's
                    else if(pos_cache[i]<0&&++pos_cache[i])
                        pos.data[i*2]=static_cast<float>(pos_from_radio.data[i*2]),pos.data[i*2+1]=static_cast<float>(pos_from_radio.data[i*2+1]),++count; //Override pos with radio's
                for(size_t i=0,id;i<pos_from_camera.second.size();++i)
                    if(pos_from_camera.second[i]!=(uint8_t)-1&&((id=std::find(robot_classes[team].begin(),robot_classes[team].end(),pos_from_camera.second[i])-robot_classes[team].begin())<6||(id=6+std::find(robot_classes[!team].begin(),robot_classes[!team].end(),pos_from_camera.second[i])-robot_classes[!team].begin())<12)&&!pos_cache[id])
                    {
                        pos.data[id*2]=pos_from_camera.first[i].x,pos.data[id*2+1]=pos_from_camera.first[i].y,++count;
                        #ifndef NDEBUG
                        fprintf(stderr,"Robot %zu: (%.2f,%.2f)\n",id,pos_from_camera.first[i].x,pos_from_camera.first[i].y);
                        #endif
                    }
                if(persistent_cache||count>0)
                {
                    //Send pos
                    static union both_map_robot buf; //Send cached pos when not found
                    if(persistent_cache)
                    {
                        //Guess drone's pos
                        pos.data[9]=team?15-1.82:1.82;
                        if(serial_info.stage_remain_time<400)
                            pos.data[8]=14;
                        else
                            pos.data[8]=team?400+14-serial_info.stage_remain_time:28-(400+14-serial_info.stage_remain_time);
                    }
                    else memset(&buf,0,sizeof(buf));
                    for(size_t i=0;i<sizeof(pos)/sizeof(float);++i)
                        if(pos.data[i]!=0)
                            buf.data[i]=std::max(pos.data[i]*100,1.f); //Convert unit (m->cm) and ensure not overflow
                    if(referee_send(0x0305,sizeof(buf),&buf))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent data to server\n");
                        #endif
                        timestamp[0]+=std::chrono::milliseconds(100);
                    }
                }
            }
            if(timer::now()>timestamp[1])
            {
                if(serial_info.game_progress==4&&(serial_info.vulnerability>0||serial_info.encrypt_changeable||password_retries>0))
                {
                    //Send cmd
                    struct robot_interaction_data buf={{0x0121,serial_info.robot_id,0x8080},{{std::min(vulnerated,cmd)}}};
                    if(buf.cmd.password_cmd=(password_retries>0?2:0))
                        memcpy(&buf.cmd.password,&guessed_password,sizeof(guessed_password)),--password_retries;
                    else if(buf.cmd.password_cmd=serial_info.encrypt_changeable)
                    {
                        std::vector<uint8_t> pwd=generate_password(6);
                        memcpy(&buf.cmd.password,pwd.data(),pwd.size());
                    }
                    if(referee_send(0x0301,sizeof(buf.header)+sizeof(buf.cmd),&buf))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent cmd: %hhu %hhu\n",buf.cmd.radar_cmd,buf.cmd.password_cmd);
                        #endif
                        if(buf.cmd.password_cmd)
                        {
                            fprintf(stderr,"%s password: ",buf.cmd.password_cmd==2?"Guessed":"New");
                            for(size_t i=0;i<sizeof(buf.cmd.password)/sizeof(uint8_t);++i)
                                fprintf(stderr,"%c",buf.cmd.password[i]);
                            fprintf(stderr,"\n");
                        }
                        timestamp[1]+=std::chrono::milliseconds(100);
                    }
                }
            }
            if(timer::now()>timestamp[2])
            {
                if(std::any_of(pos.opponent.data,pos.opponent.data+12,[](float x){return x!=0;}))
                {
                    struct robot_interaction_data buf={{0x0297,serial_info.robot_id,team*100+7}};
                    memcpy(&buf.robot_pos,&pos.opponent,sizeof(pos.opponent));
                    if(referee_send(0x0301,sizeof(buf.header)+sizeof(buf.robot_pos),&buf))
                    {
                        #ifndef NDEBUG
                        fprintf(stderr,"Sent data to robot\n");
                        #endif
                        timestamp[2]+=std::chrono::milliseconds(50);
                    }
                }
            }
            if(timer::now()<target)
                std::this_thread::sleep_for(target-timer::now());
        }
    }
    bool msg::run(const char* referee_tty, const char* radio_host, uint16_t radio_port)
    {
        if(referee.get_fd()<0&&!serial_quick_setup(referee,referee_tty))
            return false;
        running=true;
        rx=std::thread(&rm::msg::receive,this,std::ref(referee),std::ref(recv_buf),[](serial::stream& s){return serial_quick_setup(static_cast<serial::terminal&>(s));}),tx=std::thread(&rm::msg::poll,this);
        if(radio.get_fd()<0&&radio_host&&radio_port)
            radio.connect(radio_host,radio_port);
        if(radio.get_fd()>=0)
            rf=std::thread(&rm::msg::receive,this,std::ref(radio),std::ref(rf_buf),nullptr);
        return true;
    }
    void msg::stop(void)
    {
        running=false;
        radio.close();
        referee.close();
        if(tx.joinable())
            tx.join();
        if(rx.joinable())
            rx.join();
        if(rf.joinable())
            rf.join();
    }
    msg::~msg()
    {
        stop();
    }
}
