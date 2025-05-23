#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <termios.h>
#include <fcntl.h>
#include <map>
#include <string>
namespace serial
{
	//Provide unified serial interface for different systems
	class terminal
	{
		int fd;
		int flag;
		std::string file;
	public:
		struct termios options;
		terminal(int ofd=-1,int oflag=O_RDWR|O_NOCTTY);
		terminal(const std::string& filename,int oflag=O_RDWR|O_NOCTTY);
		terminal(const terminal& term);
		terminal& operator=(const terminal& term);
		#if __cplusplus >= 201103L
		terminal(terminal&& term)=default;
		terminal& operator=(terminal&& term)=default;
		static const std::map<unsigned,speed_t> speed;
		#endif
		inline int get_fd(void) const
		{
			return fd;
		}
		int open(const std::string& filename=std::string(),int oflag=O_RDWR|O_NOCTTY);
		int set_rate(speed_t speed);
		int write(const void* data,size_t count);
		int read(void* data,size_t count);
		int write_wCRC8(void* data,size_t count,size_t offset=0);
		int read_wCRC8(void* data,size_t count,size_t offset=0);
		int write_wCRC16(void* data,size_t count,size_t offset=0);
		int read_wCRC16(void* data,size_t count,size_t offset=0);
		int apply_options(int mode=TCSAFLUSH);
		~terminal();
	};
}

#endif
