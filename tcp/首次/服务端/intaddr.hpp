#pragma once
#include <iostream>
#include <string>
#include <WS2tcpip.h>
#include "log.hpp"


namespace sui
{
	//存储addr的类
	class IntAddr
	{
	public:
		IntAddr()
		{
			//初始化sockaddr_in结构体
			_addr.sin_family = AF_INET; //设置地址族为IPv4
			_addr.sin_addr.s_addr = INADDR_ANY; //设置IP地址为任意地址
			_addr.sin_port = htons(0); //将端口号转换为网络字节序
			_ip = "";
		}

		IntAddr(int port)
			: _port(port)
		{
			//初始化sockaddr_in结构体
			_addr.sin_family = AF_INET; //设置地址族为IPv4
			_addr.sin_addr.s_addr = INADDR_ANY; //设置IP地址为任意地址
			_addr.sin_port = htons(port); //将端口号转换为网络字节序
			_ip = "";
		}
		IntAddr(int& port, std::string& ip)
			:_port(port), _ip(ip)
		{
			_addr.sin_family = AF_INET;
			_addr.sin_port = htons(port);
			inet_pton(AF_INET, ip.c_str(), &_addr.sin_addr.s_addr); //十进制点分法转换IP地址
		}

		IntAddr(std::string& ip)
			:_ip(ip)
		{
			_addr.sin_family = AF_INET;
			_addr.sin_port = htons(0); //默认端口号为0
			inet_pton(AF_INET, ip.c_str(), &_addr.sin_addr.s_addr); //十进制点分法转换IP地址
		}

		IntAddr(const IntAddr& other)
			: _addr(other._addr), _port(other._port), _ip(other._ip)
		{
		}

		IntAddr& operator=(const IntAddr& other)
		{
			if (this != &other)
			{
				_addr = other._addr;
				_port = other._port;
				_ip = other._ip;
			}
			return *this;
		}

		//获取sockaddr_in结构体的引用
		sockaddr_in& getAddr()
		{
			return _addr;
		}

		//获取指针
		sockaddr* getAddrPtr()
		{
			return (sockaddr*)(&(_addr));
		}

		//获取端口号
		int getPort() const
		{
			return _port;
		}

		//获取IP地址字符串
		std::string getIp() const
		{
			return _ip;
		}

		//重新设置端口号
		void setPort(int port)
		{
			_port = port;
			_addr.sin_port = htons(port); //更新sockaddr_in结构体中的端口号
		}

		//重新设置IP地址
		void setIp(const std::string& ip)
		{
			_ip = ip;
			inet_pton(AF_INET, _ip.c_str(), &_addr.sin_addr.s_addr); //更新sockaddr_in结构体中的IP地址
		}
		void setIp()
		{
			_ip.clear();
			_addr.sin_addr.s_addr = INADDR_ANY; //设置IP地址为任意地址
		}


		//获取端口号网络字节序
		uint16_t getPortNetworkByteOrder() const
		{
			return _addr.sin_port; //返回sockaddr_in结构体中的端口号
		}

		//获取IP地址网络字节序
		uint32_t getIpNetworkByteOrder() const
		{
			return _addr.sin_addr.s_addr; //返回sockaddr_in结构体中的IP地址
		}

		//获取大小
		int size()
		{
			return sizeof(_addr); //返回sockaddr_in结构体的大小	
		}

	private:
		sockaddr_in _addr; //存储地址的sockaddr_in结构体	
		uint16_t _port; //端口号	
		std::string _ip; //IP地址字符串	
	};
}