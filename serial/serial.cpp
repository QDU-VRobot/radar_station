#include "serial.h"
#include "RM_CRC.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <string>
namespace serial
{
	#if __cplusplus >= 201103L
	const std::map<unsigned,speed_t> terminal::speed{{0,B0},{50,B50},{75,B75},{110,B110},{134,B134},{150,B150},{200,B200},{300,B300},{600,B600},{1200,B1200},{1800,B1800},{2400,B2400},{4800,B4800},{9600,B9600},{19200,B19200},{38400,B38400},{57600,B57600},{115200,B115200},{230400,B230400},{460800,B460800},{500000,B500000},{576000,B576000},{921600,B921600},{1000000,B1000000},{1152000,B1152000},{1500000,B1500000},{2000000,B2000000},{2500000,B2500000},{3000000,B3000000},{3500000,B3500000},{4000000,B4000000},{-1,__MAX_BAUD}};
	#endif
	static const char* errors[]={"Failed to get options","Failed to set options","Failed to open port","Failed to set speed","Error reading from serial port","CRC Error!\n"};
	terminal::terminal(int ofd,int oflag):fd(ofd),flag(oflag)
	{
		if(ofd>=0&&tcgetattr(ofd,&options)!=0)
			perror(errors[0]);
	}
	terminal::terminal(const std::string& filename,int oflag)
	{
		fd=-1;
		open(filename,oflag);
	}
	terminal& terminal::operator=(const terminal& term)
	{
		terminal::~terminal();
		fd=term.fd,flag=term.flag,options=term.options,file=term.file;
		return *this;
	}
	int terminal::open(const std::string& filename,int oflag)
	{
		terminal::~terminal();
		if(filename.size())
			file=filename;
		flag=oflag;
		if((fd=::open(file.c_str(),oflag))>=0)
		{
			if(tcgetattr(fd,&options)!=0)
				perror(errors[0]);
		}
		else
			perror(errors[2]);
		return fd;
	}
	int terminal::set_rate(speed_t speed)
	{
		int status;
		if((status=cfsetispeed(&options,speed))!=0)
		{
			perror(errors[3]);
			return status;
		}
		if((status=cfsetospeed(&options,speed))!=0)
			perror(errors[3]);
		return status;
	}
	int terminal::write(const void* data,size_t count)
	{
		return ::write(fd,data,count);
	}
	int terminal::read(void* data,size_t count)
	{
		size_t ncount=0;
		for(int status;ncount<count;ncount+=status)
			if((status=::read(fd,data,count))<0)
			{
				perror(errors[4]);
				return ncount?ncount:-1;
			}
		return ncount;
	}
	int terminal::write_wCRC8(void* data,size_t count,size_t offset)
	{
		Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(data),count);
		return write(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
	}
	int terminal::read_wCRC8(void* data,size_t count,size_t offset)
	{
		int status=read(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
		if(status<static_cast<int>(count-offset))
			return status;
		if(Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(data),count))
			status=0;
		else
			fputs(errors[5],stderr);
		return status;
	}
	int terminal::write_wCRC16(void* data,size_t count,size_t offset)
	{
		Append_CRC16_Check_Sum(reinterpret_cast<unsigned char*>(data),count);
		return write(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
	}
	int terminal::read_wCRC16(void* data,size_t count,size_t offset)
	{
		int status=read(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
		if(status<static_cast<int>(count-offset))
			return status;
		if(Verify_CRC16_Check_Sum(reinterpret_cast<unsigned char*>(data),count))
			status=0;
		else
			fputs(errors[5],stderr);
		return status;
	}
	int terminal::apply_options(int mode)
	{
		int status=tcsetattr(fd,mode,&options);
		if(status!=0)
			perror(errors[1]);
		return status;
	}
	terminal::~terminal()
	{
		if(fd>=0)
		{
			int tmp=fd;
			fd=-1,close(tmp);
		}
	}
}
