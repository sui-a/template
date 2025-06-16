#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <strings.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "Log.hpp"
#include "InetAddr.hpp"

using namespace LogModule;

using func_t = std::function<void(int sockfd, const std::string&, InetAddr&)>;

const int defaultfd = -1;

class UdpServer
{
public:
    UdpServer(uint16_t port, func_t func)
        : _sockfd(defaultfd),
        //   _ip(ip),
        _port(port),
        _isrunning(false),
        _func(func)
    {
    }
    void Init()
    {
        // 1. 创建套接字
        _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (_sockfd < 0)
        {
            LOG(LogLevel::FATAL) << "socket error!";
            exit(1);
        }
        LOG(LogLevel::INFO) << "socket success, sockfd : " << _sockfd;

        struct sockaddr_in local;
        bzero(&local, sizeof(local));
        local.sin_family = AF_INET;
        local.sin_port = htons(_port);

        local.sin_addr.s_addr = INADDR_ANY;


        // InetAddr addr("0", _port);
        // addr.NetAddr();

        // 为什么服务器端要显式的bind呢？IP和端口必须是众所周知且不能轻易改变的！
        int n = bind(_sockfd, (struct sockaddr*)&local, sizeof(local));
        if (n < 0)
        {
            LOG(LogLevel::FATAL) << "bind error";
            exit(2);
        }
        LOG(LogLevel::INFO) << "bind success, sockfd : " << _sockfd;
    }
    void Start()
    {
        _isrunning = true;
        while (_isrunning)
        {
            char buffer[1024];
            struct sockaddr_in peer;
            socklen_t len = sizeof(peer);
            
            ssize_t s = recvfrom(_sockfd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&peer, &len);
            if (s > 0)
            {
                InetAddr client(peer);
                buffer[s] = 0;
                // TODO
                _func(_sockfd, buffer, client);


                // LOG(LogLevel::DEBUG) << "[" << peer_ip << ":" << peer_port<< "]# " << buffer; // 1. 消息内容 2. 谁发的？？

                // 2. 发消息
                // std::string echo_string = "server echo@ ";
                // echo_string += buffer;
                // sendto(_sockfd, result.c_str(), result.size(), 0, (struct sockaddr*)&peer, len);
            }
        }
    }
    ~UdpServer()
    {
    }

private:
    int _sockfd;
    uint16_t _port;
    // std::string _ip; // 用的是字符串风格，点分十进制, "192.168.1.1"
    bool _isrunning;

    func_t _func; // 服务器的回调函数，用来进行对数据进行处理
};