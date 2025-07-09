#pragma once 
#include "InetAddr.hpp"
#include "my_log.hpp"
#include <memory>
#include <iostream>

//实现模板模式套接字
namespace sui
{
    //使用模板进行初始化
    class mySocket
    {
    public:
        mySocket() = default;
        //创建套接字
        virtual void socket_create() = 0;
        //绑定地址和端口
        virtual void socket_bind(uint16_t& port) = 0;
        //开启监听
        virtual void socket_listen() = 0;
        //接受客户端连接
        virtual std::unique_ptr<mySocket> socket_accept(InetAddr* client_addr) = 0;
        //读取套接字
        virtual int my_recv(std::string& out) = 0;
        //发送数据
        virtual int my_send(std::string data) = 0;
        //关闭套接字
        virtual void Close() = 0;
        //链接客户端
        virtual int Connect(const std::string server_ip, uint16_t port) = 0;

        //服务端口初始化
        void mySocketServerInit(uint16_t port)
        {
            //创建套接字
            socket_create();
            //绑定地址和端口
            socket_bind(port);
            //开始监听
            socket_listen();
        }

        //客户端口初始化
        void mySocketClientInit()
        {
            //创建套接字
            socket_create();
        }

        //服务端监听

        ~mySocket() = default;
    };

    class myTcpSocket : public mySocket
    {
    public:
        //设置好端口和ip 用于寻找服务端
        myTcpSocket()
        :_socket(-1)
        {}

        myTcpSocket(int socket)
        :_socket(socket)
        {}

        //创建套接字
        void socket_create() override
        {
            _socket = socket(AF_INET, SOCK_STREAM, 0);
            if (_socket == -1) 
            {
                LOG(LogLevel::ERROR) << "套接字创建失败，直接终止程序";
                exit(1);
            }
        }

        //绑定本地地址和端口
        void socket_bind(uint16_t& port) override
        {
            InetAddr addr(port);
            if (bind(_socket, addr.NetAddrPtr(), addr.NetAddrLen()) == -1) 
            {
                LOG(LogLevel::ERROR) << "绑定失败： " << strerror(errno);
                close(_socket);
                exit(0);
            }
        }

        //开始监听
        void socket_listen() override
        {
            //开始监听并设置等待队列连接长度
            if(listen(_socket, 5) == -1)
            {
                //监听出错
                LOG(LogLevel::ERROR) << "监听出现错误: " << strerror(errno);
                close(_socket);
                exit(0);
            }
        }

        //后续出现错误交给外界处理
        //连接客户端
        std::unique_ptr<mySocket> socket_accept(InetAddr* client_addr) override
        {
            sockaddr_in my_addr;
            socklen_t my_addr_len = client_addr->NetAddrLen();
            int client_fd = accept(_socket, (sockaddr*)&my_addr, &my_addr_len);

            client_addr->SetAddr(my_addr);

            if (client_fd == -1) 
            {
                //连接失败，返回空指针
                return nullptr;
            }

            //返回套接字
            return std::make_unique<myTcpSocket>(client_fd);
        }

        //读取套接字
        int my_recv(std::string& out) override
        {
            char buff[1024];
            ssize_t bytes_received = recv(_socket, buff, sizeof(buff), 0);
            if(bytes_received > 0)
            {
                out += buff;
            }
            return bytes_received;
        }

        //发送数据
        int my_send(std::string data) override
        {
            return send(_socket, data.c_str(), data.size(), 0);
        }

        //关闭套接字
        void Close() override
        {
            if(_socket > 0)
            {
                ::close(_socket);
                _socket = -1;
            }
        }

        //链接客户端
        int Connect(const std::string server_ip, uint16_t port) override
        {
            InetAddr server(server_ip, port);
            return ::connect(_socket, server.NetAddrPtr(), server.NetAddrLen());
        }

        ~myTcpSocket()
        {}
    private:
        int _socket;
    };


}