#pragma once 
#include <iostream>
#include <WS2tcpip.h>
#include <string>
#include "log.hpp"
#include "intaddr.hpp"

#pragma comment(lib, "Ws2_32.lib")

namespace sui
{
	class my_tcp_server
	{
	public:
		my_tcp_server(int port = 0)
			:_server_addr(port)
		{

		}

		void init()
		{
			//对库进行初始化
			WSADATA wsa;
			if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) 
			{
				LOG(My_level::error) << "对 Windows 套接字库初始化失败";
				exit(1);
			}
			//创建套接字
			_server = socket(AF_INET, SOCK_STREAM, 0);
			if (_server == INVALID_SOCKET) 
			{
				//套接字创建失败，准备返回
				LOG(My_level::error) << "套接字创建失败";
				exit(1);
			}
			
			_server_addr.setIp(); //设置为任何ip都可以访问

			//进行套接字绑定
			if (bind(_server, _server_addr.getAddrPtr(), _server_addr.size()) == -1)
			{
				//绑定失败，返回并退出程序
				LOG(My_level::error) << "套接字绑定失败";
				exit(1);
			}

			//开始监听  //设置未完成连接队列的最大长度
			listen(_server, 8);
		}
		
		//服务器开始运行
		void start()
		{
			LOG(My_level::info) << "服务器开始运行";
			while (1)
			{
				//进入无线循环等外客户端连接
				//使用accept函数等待客户端连接并返回值
				sui::IntAddr _client_addr;
				int client_addr_size = _client_addr.size();
				SOCKET client = accept(_server, (sockaddr*)&_client_addr, &client_addr_size);
				if (client < 0)
				{
					//接收失败
					LOG(My_level::error) << "接收客户端连接失败";
					break;
				}
				while (1)
				{
					char buf[1024];
					int len = recv(client, buf, sizeof(buf), 0);
					if (len == 0)
					{
						//客户端断开连接
						LOG(My_level::error) << "客户端断开连接，进行下一次连接";
						break;
					}
					else if (len < 0)
					{
						//接收数据失败
						LOG(My_level::error) << "接收数据错误，进行下一次连接";
						break;
					}

					//防止越界
					if (len > sizeof(buf) - 1)
					{
						len = sizeof(buf) - 1;
					}
					buf[len] = '\0';

					LOG(My_level::info) << "[" << _client_addr.getIp() << ":"
						<< _client_addr.getPort() << "]: " << buf;

					//发送数据给客户端
					std::string response = "接收数据成功";
					int send_len = send(client, response.c_str(), response.size(), 0);
					if (send_len == SOCKET_ERROR)
					{
						LOG(My_level::error) << "发送数据失败";
						break;
					}
					else
					{
						LOG(My_level::info) << "发送数据成功，长度: " << send_len;
					}
					
				}
				LOG(My_level::info) << "开启下一轮连接 ";
			}
		}

		~my_tcp_server()
		{
			//关闭套接字
			closesocket(_server);
			//清理Windows套接字库
			WSACleanup();
			LOG(My_level::info) << "服务器已关闭";
		}
	private:
		SOCKET _server;
		sui::IntAddr _server_addr;
	};



}