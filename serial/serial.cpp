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
    static const char* errors[]={"Failed to get options","Failed to set options","Failed to open port","Failed to set speed","Error reading from stream","CRC8 Error!\n","CRC16 Error!\n","Failed to resolve address","Failed to create socket","Failed to connect","Failed to bind","Failed to listen","Failed to accept","Failed to set socket option","Failed to get socket name"};
    terminal::terminal(int ofd,int oflag):stream(ofd),flag(oflag)
    {
        if(ofd>=0&&tcgetattr(ofd,&options)!=0)
            perror(errors[0]);
    }
    terminal::terminal(const std::string& filename,int oflag):stream()
    {
        open(filename,oflag);
    }
    int terminal::open(const std::string& filename,int oflag)
    {
        close();
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
    ssize_t stream::write_wCRC8(void* data,size_t count,size_t offset)
    {
        Append_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(data),count);
        return write(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
    }
    ssize_t stream::read_wCRC8(void* data,size_t count,size_t offset)
    {
        ssize_t status=read(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
        if(status==static_cast<ssize_t>(count-offset))
        {
            if(Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char*>(data),count))
                ++status;
            else
                fputs(errors[5],stderr);
        }
        return status;
    }
    ssize_t stream::write_wCRC16(void* data,size_t count,size_t offset)
    {
        Append_CRC16_Check_Sum(reinterpret_cast<unsigned char*>(data),count);
        return write(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
    }
    ssize_t stream::read_wCRC16(void* data,size_t count,size_t offset)
    {
        ssize_t status=read(reinterpret_cast<unsigned char*>(data)+offset,count-offset);
        if(status==static_cast<ssize_t>(count-offset))
        {
            if(Verify_CRC16_Check_Sum(reinterpret_cast<unsigned char*>(data),count))
                ++status;
            else
                fputs(errors[6],stderr);
        }
        return status;
    }
    ssize_t stream::read(void* data,size_t count)
    {
        size_t ncount=0;
        for(ssize_t status;ncount<count;ncount+=status)
            if((status=::read(fd,data,count))<=0)
            {
                if(status<0)
                    perror(errors[4]);
                else
                    fputs(errors[4],stderr),fputs("\n",stderr);
                return ncount?ncount:status;
            }
        return ncount;
    }
    ssize_t stream::write(const void* data,size_t count)
    {
        return ::write(fd,data,count);
    }
    int stream::close()
    {
        int ret=0;
        if(fd>=0)
            ret=::close(fd),fd=-1;
        return ret;
    }
    int terminal::apply_options(int mode)
    {
        int status=tcsetattr(fd,mode,&options);
        if(status!=0)
            perror(errors[1]);
        return status;
    }
    terminal::terminal(const terminal& term):stream(term),flag(term.flag),file(term.file),options(term.options)
    {
        if(term.fd>=0)
            open(term.file,term.flag);
    }
    terminal& terminal::operator=(terminal term)
    {
        stream::operator=(term);
        std::swap(flag,term.flag);
        std::swap(options,term.options);
        std::swap(file,term.file);
        return *this;
    }

    tcp::tcp(const tcp& other):stream(other),domain(other.domain),host(other.host),port(other.port)
    {
        if(other.fd>=0)
        {
            if(other.host.size())
                connect();
            else
                open();
        }
    }
    tcp& tcp::operator=(tcp other)
    {
        stream::operator=(other);
        std::swap(domain,other.domain);
        std::swap(host,other.host);
        std::swap(port,other.port);
        return *this;
    }

    tcp::tcp(int ofd):stream(ofd),domain(AF_UNSPEC),port(0)
    {
        if(fd>=0)
        {
            struct sockaddr_storage addr;
            socklen_t addr_len=sizeof(addr);
            if(::getsockname(fd,reinterpret_cast<struct sockaddr*>(&addr),&addr_len)==0)
                domain=addr.ss_family;
            else
                perror(errors[14]);
        }
    }
    tcp::tcp(const std::string& host, uint16_t port):stream(),domain(AF_UNSPEC)
    {
        connect(host,port);
    }
    tcp::tcp(uint16_t port):stream()
    {
        open(port);
    }
    int tcp::connect(const std::string& host, uint16_t port)
    {
        close();
        if(host.size())
            this->host=host;
        if(port)
            this->port=port;
        if(this->host.empty())
            return -1;
        struct sockaddr_in addr4;
        struct sockaddr_in6 addr6;
        if(inet_pton(AF_INET,this->host.c_str(),&addr4.sin_addr)==1)
        {
            fd=::socket(AF_INET,SOCK_STREAM,0);
            if(fd<0)
            {
                perror(errors[8]);
                return -1;
            }
            domain=AF_INET;
            addr4.sin_family=AF_INET;
            addr4.sin_port=htons(this->port);
            if(::connect(fd,reinterpret_cast<struct sockaddr*>(&addr4),sizeof(addr4))<0)
            {
                perror(errors[9]);
                close();
                return -1;
            }
        }
        else if(inet_pton(AF_INET6,this->host.c_str(),&addr6.sin6_addr)==1)
        {
            fd=::socket(AF_INET6,SOCK_STREAM,0);
            if(fd<0)
            {
                perror(errors[8]);
                return -1;
            }
            domain=AF_INET6;
            addr6.sin6_family=AF_INET6;
            addr6.sin6_port=htons(this->port);
            if(::connect(fd,reinterpret_cast<struct sockaddr*>(&addr6),sizeof(addr6))<0)
            {
                perror(errors[9]);
                close();
                return -1;
            }
        }
        else
        {
            perror(errors[7]);
            return -1;
        }
        return fd;
    }
    int tcp::open(uint16_t port, int backlog)
    {
        close();
        if(port)
            this->port=port;
        if(!this->port)
            return -1;
        host.clear();
        fd=::socket(AF_INET6,SOCK_STREAM,0);
        if(fd<0)
        {
            perror(errors[8]);
            return -1;
        }
        domain=AF_INET6;
        int off=0,on=1;
        if(::setsockopt(fd,IPPROTO_IPV6,IPV6_V6ONLY,&off,sizeof(off))<0||::setsockopt(fd,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on))<0)
            perror(errors[13]);
        struct sockaddr_in6 addr={0};
        addr.sin6_family=AF_INET6;
        addr.sin6_port=htons(this->port);
        addr.sin6_addr=in6addr_any;
        if(::bind(fd,reinterpret_cast<struct sockaddr*>(&addr),sizeof(addr))<0)
        {
            perror(errors[10]);
            close();
            return -1;
        }
        if(::listen(fd,backlog)<0)
        {
            perror(errors[11]);
            close();
            return -1;
        }
        return fd;
    }
    int tcp::accept(tcp& client)
    {
        client.close();
        struct sockaddr_storage addr;
        socklen_t addr_len=sizeof(addr);
        client.fd=::accept(fd,reinterpret_cast<struct sockaddr*>(&addr),&addr_len);
        if(client.fd<0)
        {
            perror(errors[12]);
            return -1;
        }
        client.domain=addr.ss_family;
        char ip[INET6_ADDRSTRLEN];
        if(addr.ss_family==AF_INET)
        {
            client.port=ntohs(reinterpret_cast<struct sockaddr_in*>(&addr)->sin_port);
            inet_ntop(AF_INET,&reinterpret_cast<struct sockaddr_in*>(&addr)->sin_addr,ip,sizeof(ip));
        }
        else
        {
            client.port=ntohs(reinterpret_cast<struct sockaddr_in6*>(&addr)->sin6_port);
            inet_ntop(AF_INET6,&reinterpret_cast<struct sockaddr_in6*>(&addr)->sin6_addr,ip,sizeof(ip));
        }
        client.host=ip;
        return client.fd;
    }
    int tcp::set_option(int level,int optname,const void* optval,socklen_t optlen)
    {
        int ret=::setsockopt(fd,level,optname,optval,optlen);
        if(ret!=0)
            perror(errors[13]);
        return ret;
    }
}
