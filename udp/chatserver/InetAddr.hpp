#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
// 网络地址和主机地址之间进行转换的类

class InetAddr
{
public:
    InetAddr(struct sockaddr_in& addr) : _addr(addr)
    {
        // 网络转主机
        _port = ntohs(_addr.sin_port);           // 从网络中拿到的！网络序列
        // _ip = inet_ntoa(_addr.sin_addr); // 4字节网络风格的IP -> 点分十进制的字符串风格的IP
        char ipbuffer[64];
        inet_ntop(AF_INET, &_addr.sin_addr, ipbuffer, sizeof(ipbuffer));
        _ip = ipbuffer;
    }
    InetAddr(const std::string& ip, uint16_t port) :_ip(ip), _port(port)
    {
        // 主机转网络
        memset(&_addr, 0, sizeof(_addr));
        _addr.sin_family = AF_INET;
        inet_pton(AF_INET, _ip.c_str(), &_addr.sin_addr);
        _addr.sin_port = htons(_port);
        //local.sin_addr.s_addr = inet_addr(_ip.c_str()); // TODO
    }
    uint16_t Port() { return _port; }
    std::string Ip() { return _ip; }
    const struct sockaddr_in& NetAddr() { return _addr; }
    bool operator==(const InetAddr& addr)
    {
        return addr._ip == _ip && addr._port == _port;
    }
    std::string StringAddr()
    {
        return _ip + ":" + std::to_string(_port);
    }
    ~InetAddr()
    {
    }

private:
    struct sockaddr_in _addr;
    std::string _ip;
    uint16_t _port;
};