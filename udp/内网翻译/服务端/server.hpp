#pragma once 
#pragma comment(lib, "ws2_32.lib")
#include <WS2tcpip.h>
#include <string>
#include <iostream>
#include "log.hpp"
#include "dictioinary.hpp"
namespace sui
{
	//服务端口
	class my_server
	{
	public:
		static my_server* get_server()
		{
			if(my_ser == nullptr)
			{
				my_ser = new my_server();
			}
			return my_ser;
		}

		//服务端初始化
		bool init()
		{
			//对网络服务接口进行初始化
			if (WSAStartup(MAKEWORD(2, 2), &_wsa_data))
			{
				//初始化失败
				LOG(My_level::error) << "WSAS 初始化失败";
				return false;
			}
			
			//创建套接字
			_server_socket = socket(AF_INET, SOCK_DGRAM, 0);
			if (_server_socket == INVALID_SOCKET)
			{
				//创建套接字失败，输出日志并返回失败
				LOG(My_level::error) << "套接字创建失败";
				return false;
			}
			//初始化服务端信息并与套接字绑定
			//_server_addr.sin_addr.S_un.S_addr = INADDR_ANY; //设置可以接受任意地址消息
			memset(&_server_addr, 0, sizeof(_server_addr)); //清空结构体
			memset(&_client_addr, 0, sizeof(_client_addr)); //清空结构体

			_server_addr.sin_addr.s_addr = INADDR_ANY;
			_server_addr.sin_family = AF_INET; //设置地址族为IPv4
			_server_addr.sin_port  = htons(8888);
			 //进行绑定
			 if (bind(_server_socket, reinterpret_cast<struct sockaddr*>(&_server_addr), sizeof(_server_addr)) == SOCKET_ERROR)
			 {
				 LOG(My_level::error) << "端口绑定失败";
				 return false;
			 }

			 //获取长度，并存储，防止多次调用函数浪费性能
			 _server_addr_len = sizeof(_server_addr);
			 _client_addr_len = sizeof(_client_addr);
		}

		//服务端开始
		void start()
		{
			LOG(My_level::info) << "服务端开始运行";

			while (1)
			{
				char buffer[1024];
				memset(buffer, 0, sizeof(buffer)); //初始化缓冲区
				//开始接收客户端消息
				auto ret_recv = recvfrom(_server_socket, buffer, sizeof(buffer), 0,
										reinterpret_cast<struct sockaddr*>(&_client_addr), &_client_addr_len);

				char client_ip[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, &_client_addr.sin_addr, client_ip, INET_ADDRSTRLEN); // 将 IP 地址转换为字符串
				unsigned short client_port = ntohs(_client_addr.sin_port); // 将端口号转换为主机字节序

				if (ret_recv == SOCKET_ERROR)
				{
					LOG(My_level::error) << "接收消息失败" << WSAGetLastError();
					Sleep(1000);
					continue;
				}
				else
				{
					buffer[ret_recv] = '\0'; //确保字符串结束
					//往回发送
					std::string back_buff; //发送结果
					bool back_mode = _dir.get_change(buffer, back_buff);

					LOG(My_level::info) << "[" << client_ip << ":" << client_port << "]@ " << buffer;


					if(!back_mode)
					{
						LOG(My_level::error) << "转换失败，可能是没有找到对应的键值对";
						back_buff = "不存在";
					}
					while (1)
					{
						auto ret_send = sendto(_server_socket, back_buff.c_str(), back_buff.size(), 0,
							reinterpret_cast<struct sockaddr*>(&_client_addr), _client_addr_len);
						if (ret_send > 0)
						{
							LOG(My_level::info) << "发送成功，发送数据为：" << back_buff;
							break;
						}
						Sleep(1000); //如果发送失败，等待1秒后重试
					}
				}
			}
		}

		//析构函数
		~my_server()
		{
			closesocket(_server_socket);
			WSACleanup();
			LOG(My_level::info) << "服务端析构";
		}
	private:
		my_server(int port = 8080)
			:_port(port)
		{
			//构建服务端口，因为服务端接收消息是不需要绑定所需要的ip，所以只需要端口号
		}

		//端口号
		int _port;
		//套接字
		SOCKET _server_socket;
		//存储was初始化信息
		WSADATA _wsa_data;
		//存储服务器信息和目标服务器信息
		struct sockaddr_in _server_addr, //自己的信息
						   _client_addr;  //当接收客户端消息以后存储在这里，准备发送处理后的消息给客户端
		//本地存储服务端信息长度
		int _server_addr_len;
		//本地存储客户端信息长度
		int _client_addr_len;
		//创建字典
		sui::my_direction _dir;

		//创建指针，隐藏构造函数，限制用户操作
		static my_server* my_ser;
	};

	my_server* my_server::my_ser = nullptr;
}