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
			//�Կ���г�ʼ��
			WSADATA wsa;
			if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) 
			{
				LOG(My_level::error) << "�� Windows �׽��ֿ��ʼ��ʧ��";
				exit(1);
			}
			//�����׽���
			_server = socket(AF_INET, SOCK_STREAM, 0);
			if (_server == INVALID_SOCKET) 
			{
				//�׽��ִ���ʧ�ܣ�׼������
				LOG(My_level::error) << "�׽��ִ���ʧ��";
				exit(1);
			}
			
			_server_addr.setIp(); //����Ϊ�κ�ip�����Է���

			//�����׽��ְ�
			if (bind(_server, _server_addr.getAddrPtr(), _server_addr.size()) == -1)
			{
				//��ʧ�ܣ����ز��˳�����
				LOG(My_level::error) << "�׽��ְ�ʧ��";
				exit(1);
			}

			//��ʼ����  //����δ������Ӷ��е���󳤶�
			listen(_server, 8);
		}
		
		//��������ʼ����
		void start()
		{
			LOG(My_level::info) << "��������ʼ����";
			while (1)
			{
				//��������ѭ������ͻ�������
				//ʹ��accept�����ȴ��ͻ������Ӳ�����ֵ
				sui::IntAddr _client_addr;
				int client_addr_size = _client_addr.size();
				SOCKET client = accept(_server, (sockaddr*)&_client_addr, &client_addr_size);
				if (client < 0)
				{
					//����ʧ��
					LOG(My_level::error) << "���տͻ�������ʧ��";
					break;
				}
				while (1)
				{
					char buf[1024];
					int len = recv(client, buf, sizeof(buf), 0);
					if (len == 0)
					{
						//�ͻ��˶Ͽ�����
						LOG(My_level::error) << "�ͻ��˶Ͽ����ӣ�������һ������";
						break;
					}
					else if (len < 0)
					{
						//��������ʧ��
						LOG(My_level::error) << "�������ݴ��󣬽�����һ������";
						break;
					}

					//��ֹԽ��
					if (len > sizeof(buf) - 1)
					{
						len = sizeof(buf) - 1;
					}
					buf[len] = '\0';

					LOG(My_level::info) << "[" << _client_addr.getIp() << ":"
						<< _client_addr.getPort() << "]: " << buf;

					//�������ݸ��ͻ���
					std::string response = "�������ݳɹ�";
					int send_len = send(client, response.c_str(), response.size(), 0);
					if (send_len == SOCKET_ERROR)
					{
						LOG(My_level::error) << "��������ʧ��";
						break;
					}
					else
					{
						LOG(My_level::info) << "�������ݳɹ�������: " << send_len;
					}
					
				}
				LOG(My_level::info) << "������һ������ ";
			}
		}

		~my_tcp_server()
		{
			//�ر��׽���
			closesocket(_server);
			//����Windows�׽��ֿ�
			WSACleanup();
			LOG(My_level::info) << "�������ѹر�";
		}
	private:
		SOCKET _server;
		sui::IntAddr _server_addr;
	};



}