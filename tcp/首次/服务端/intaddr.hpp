#pragma once
#include <iostream>
#include <string>
#include <WS2tcpip.h>
#include "log.hpp"


namespace sui
{
	//�洢addr����
	class IntAddr
	{
	public:
		IntAddr()
		{
			//��ʼ��sockaddr_in�ṹ��
			_addr.sin_family = AF_INET; //���õ�ַ��ΪIPv4
			_addr.sin_addr.s_addr = INADDR_ANY; //����IP��ַΪ�����ַ
			_addr.sin_port = htons(0); //���˿ں�ת��Ϊ�����ֽ���
			_ip = "";
		}

		IntAddr(int port)
			: _port(port)
		{
			//��ʼ��sockaddr_in�ṹ��
			_addr.sin_family = AF_INET; //���õ�ַ��ΪIPv4
			_addr.sin_addr.s_addr = INADDR_ANY; //����IP��ַΪ�����ַ
			_addr.sin_port = htons(port); //���˿ں�ת��Ϊ�����ֽ���
			_ip = "";
		}
		IntAddr(int& port, std::string& ip)
			:_port(port), _ip(ip)
		{
			_addr.sin_family = AF_INET;
			_addr.sin_port = htons(port);
			inet_pton(AF_INET, ip.c_str(), &_addr.sin_addr.s_addr); //ʮ���Ƶ�ַ�ת��IP��ַ
		}

		IntAddr(std::string& ip)
			:_ip(ip)
		{
			_addr.sin_family = AF_INET;
			_addr.sin_port = htons(0); //Ĭ�϶˿ں�Ϊ0
			inet_pton(AF_INET, ip.c_str(), &_addr.sin_addr.s_addr); //ʮ���Ƶ�ַ�ת��IP��ַ
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

		//��ȡsockaddr_in�ṹ�������
		sockaddr_in& getAddr()
		{
			return _addr;
		}

		//��ȡָ��
		sockaddr* getAddrPtr()
		{
			return (sockaddr*)(&(_addr));
		}

		//��ȡ�˿ں�
		int getPort() const
		{
			return _port;
		}

		//��ȡIP��ַ�ַ���
		std::string getIp() const
		{
			return _ip;
		}

		//�������ö˿ں�
		void setPort(int port)
		{
			_port = port;
			_addr.sin_port = htons(port); //����sockaddr_in�ṹ���еĶ˿ں�
		}

		//��������IP��ַ
		void setIp(const std::string& ip)
		{
			_ip = ip;
			inet_pton(AF_INET, _ip.c_str(), &_addr.sin_addr.s_addr); //����sockaddr_in�ṹ���е�IP��ַ
		}
		void setIp()
		{
			_ip.clear();
			_addr.sin_addr.s_addr = INADDR_ANY; //����IP��ַΪ�����ַ
		}


		//��ȡ�˿ں������ֽ���
		uint16_t getPortNetworkByteOrder() const
		{
			return _addr.sin_port; //����sockaddr_in�ṹ���еĶ˿ں�
		}

		//��ȡIP��ַ�����ֽ���
		uint32_t getIpNetworkByteOrder() const
		{
			return _addr.sin_addr.s_addr; //����sockaddr_in�ṹ���е�IP��ַ
		}

		//��ȡ��С
		int size()
		{
			return sizeof(_addr); //����sockaddr_in�ṹ��Ĵ�С	
		}

	private:
		sockaddr_in _addr; //�洢��ַ��sockaddr_in�ṹ��	
		uint16_t _port; //�˿ں�	
		std::string _ip; //IP��ַ�ַ���	
	};
}