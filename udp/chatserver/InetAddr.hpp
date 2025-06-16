#pragma once
#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
// �����ַ��������ַ֮�����ת������

class InetAddr
{
public:
    InetAddr(struct sockaddr_in& addr) : _addr(addr)
    {
        // ����ת����
        _port = ntohs(_addr.sin_port);           // ���������õ��ģ���������
        // _ip = inet_ntoa(_addr.sin_addr); // 4�ֽ��������IP -> ���ʮ���Ƶ��ַ�������IP
        char ipbuffer[64];
        inet_ntop(AF_INET, &_addr.sin_addr, ipbuffer, sizeof(ipbuffer));
        _ip = ipbuffer;
    }
    InetAddr(const std::string& ip, uint16_t port) :_ip(ip), _port(port)
    {
        // ����ת����
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