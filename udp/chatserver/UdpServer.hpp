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
        // 1. �����׽���
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

        // Ϊʲô��������Ҫ��ʽ��bind�أ�IP�Ͷ˿ڱ�����������֪�Ҳ������׸ı�ģ�
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


                // LOG(LogLevel::DEBUG) << "[" << peer_ip << ":" << peer_port<< "]# " << buffer; // 1. ��Ϣ���� 2. ˭���ģ���

                // 2. ����Ϣ
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
    // std::string _ip; // �õ����ַ�����񣬵��ʮ����, "192.168.1.1"
    bool _isrunning;

    func_t _func; // �������Ļص��������������ж����ݽ��д���
};