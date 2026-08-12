#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <termios.h>
#include <fcntl.h>
#include <map>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
namespace serial
{
    class stream
    {
    protected:
        int fd;
    public:
        stream(int ofd=-1):fd(ofd){}
        stream(const stream& other):fd(-1){}
        stream& operator=(stream other)
        {
            std::swap(fd,other.fd);
            return *this;
        }
        #if __cplusplus >= 201103L
        stream(stream&& other) noexcept:fd(other.fd){other.fd=-1;}
        #endif
        virtual ssize_t read(void* data, size_t count);
        virtual ssize_t write(const void* data, size_t count);
        virtual int close();
        inline int get_fd(void) const{return fd;}
        ssize_t write_wCRC8(void* data, size_t count, size_t offset = 0);
        ssize_t read_wCRC8(void* data, size_t count, size_t offset = 0);
        ssize_t write_wCRC16(void* data, size_t count, size_t offset = 0);
        ssize_t read_wCRC16(void* data, size_t count, size_t offset = 0);
        virtual ~stream(){close();}
    };

    class terminal : public stream
    {
        int flag;
        std::string file;
    public:
        struct termios options;
        explicit terminal(int ofd=-1,int oflag=O_RDWR|O_NOCTTY);
        explicit terminal(const std::string& filename,int oflag=O_RDWR|O_NOCTTY);
        terminal(const terminal& term);
        terminal& operator=(terminal term);
        #if __cplusplus >= 201103L
        terminal(terminal&& term) noexcept:stream(std::move(term)),flag(term.flag),file(std::move(term.file)),options(term.options){}
        static const std::map<unsigned,speed_t> speed;
        #endif
        int open(const std::string& filename=std::string(),int oflag=O_RDWR|O_NOCTTY);
        int set_rate(speed_t speed);
        int apply_options(int mode=TCSAFLUSH);
    };

    class tcp : public stream
    {
        int domain;
        std::string host;
        uint16_t port;
    public:
        explicit tcp(int ofd=-1);
        tcp(const std::string& host, uint16_t port);
        explicit tcp(uint16_t port);
        tcp(const tcp& other);
        tcp& operator=(tcp other);
        #if __cplusplus >= 201103L
        tcp(tcp&& other) noexcept:stream(std::move(other)),domain(other.domain),host(std::move(other.host)),port(other.port){}
        #endif
        int connect(const std::string& host = std::string(), uint16_t port = 0);
        int open(uint16_t port = 0, int backlog = 5);
        int accept(tcp& client);
        int set_option(int level, int optname, const void* optval, socklen_t optlen);
    };
}

#endif
