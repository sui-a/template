#pragma once
#pragma comment(lib, "ws2_32.lib")

#include <iostream>
#include <string>
#include <WS2tcpip.h>
#include "log.hpp"

namespace sui
{

	//定义服务端管理类
	class my_sock_server
	{
	public:
		my_sock_server()
		{
			//初始化winsock
			if (WSAStartup(MAKEWORD(2, 2), &was_data))
			{
				//返回值不为0， 初始化失败
				LOG(My_level::error) << "服务端winsock初始化失败";
				exit(1);
			}

			if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
			{
				//使用plv4,udp
				LOG(My_level::error) << "创建套接字失败";
				exit(1);
			}

			//服务端能绑定的地址和端口
			memset(&server, 0, sizeof(server));
			server.sin_family = AF_INET;
			server.sin_addr.s_addr = INADDR_ANY;
			server.sin_port = htons(8888);

			if (bind(s, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
			{
				//这里的bind与平时不同，当第一个参数为socket时，将套接字与网络端点连接
				//连接失败进入
				LOG(My_level::error) << "套接字与网络端口关连失败";
				exit(1);
			}

			//获取客户端地址长度
			client_len = sizeof(struct sockaddr_in);

			////设置为监听模式并指定连接数
			//listen(s, 3);  //使用udp协议无需监听模式
		}

		//发送
		int to_send(std::string& dat)
		{
			//进行发送
			int ret = sendto(s, dat.c_str(), dat.size(), 0, (struct sockaddr*)&client, client_len);
			if (ret == SOCKET_ERROR)
			{
				LOG(My_level::error) << "发送失败";
				return -1;
			}
			return 0;
		}



		//接收
		int receive(std::string& data)
		{
			//用来接收客户端消息
			//接收客户端存储位置
			char my_message[1024]; //最大1024字节
			//获取信息，并获取客户端地址信息
			auto ret = recvfrom(s, my_message, sizeof(my_message), 0, (struct sockaddr*)&client, &client_len);
			if (ret == SOCKET_ERROR)
			{
				//接收失败，返回1
				LOG(sui::My_level::error) << "接收客户端消息失败";
				return 1;
			}
			else if (ret == 0)
			{
				//目前未设置超时
				LOG(My_level::info) << "等待超时，未获取到数据";
				return 2;
			}
			else
			{
				//接收成功，返回0
				my_message[ret] = '\0'; //添加字符串结束符
				data = my_message; //将接收到的消息存储到data中
				LOG(My_level::info) << "接收客户端消息成功: " << data;
				return 0;
			}
		}

		~my_sock_server()
		{
			closesocket(s);
			WSACleanup();
			LOG(My_level::info) << "服务端套接字关闭";
		}

	private:
		//存储winsock初始化信息
		WSADATA was_data;
		//创建客户端通讯和服务监听的套接字
		SOCKET  s;
		//存储服务器地址信息
		struct sockaddr_in server, client;
		//整形，存储客户端地址长度
		int client_len;

	};
}

