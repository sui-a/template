#pragma once 
#pragma comment(lib, "ws2_32.lib")
#include <WS2tcpip.h>
#include <string>
#include <iostream>
#include "log.hpp"
#include "dictioinary.hpp"
namespace sui
{
	//����˿�
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

		//����˳�ʼ��
		bool init()
		{
			//���������ӿڽ��г�ʼ��
			if (WSAStartup(MAKEWORD(2, 2), &_wsa_data))
			{
				//��ʼ��ʧ��
				LOG(My_level::error) << "WSAS ��ʼ��ʧ��";
				return false;
			}
			
			//�����׽���
			_server_socket = socket(AF_INET, SOCK_DGRAM, 0);
			if (_server_socket == INVALID_SOCKET)
			{
				//�����׽���ʧ�ܣ������־������ʧ��
				LOG(My_level::error) << "�׽��ִ���ʧ��";
				return false;
			}
			//��ʼ���������Ϣ�����׽��ְ�
			//_server_addr.sin_addr.S_un.S_addr = INADDR_ANY; //���ÿ��Խ��������ַ��Ϣ
			memset(&_server_addr, 0, sizeof(_server_addr)); //��սṹ��
			memset(&_client_addr, 0, sizeof(_client_addr)); //��սṹ��

			_server_addr.sin_addr.s_addr = INADDR_ANY;
			_server_addr.sin_family = AF_INET; //���õ�ַ��ΪIPv4
			_server_addr.sin_port  = htons(8888);
			 //���а�
			 if (bind(_server_socket, reinterpret_cast<struct sockaddr*>(&_server_addr), sizeof(_server_addr)) == SOCKET_ERROR)
			 {
				 LOG(My_level::error) << "�˿ڰ�ʧ��";
				 return false;
			 }

			 //��ȡ���ȣ����洢����ֹ��ε��ú����˷�����
			 _server_addr_len = sizeof(_server_addr);
			 _client_addr_len = sizeof(_client_addr);
		}

		//����˿�ʼ
		void start()
		{
			LOG(My_level::info) << "����˿�ʼ����";

			while (1)
			{
				char buffer[1024];
				memset(buffer, 0, sizeof(buffer)); //��ʼ��������
				//��ʼ���տͻ�����Ϣ
				auto ret_recv = recvfrom(_server_socket, buffer, sizeof(buffer), 0,
										reinterpret_cast<struct sockaddr*>(&_client_addr), &_client_addr_len);

				char client_ip[INET_ADDRSTRLEN];
				inet_ntop(AF_INET, &_client_addr.sin_addr, client_ip, INET_ADDRSTRLEN); // �� IP ��ַת��Ϊ�ַ���
				unsigned short client_port = ntohs(_client_addr.sin_port); // ���˿ں�ת��Ϊ�����ֽ���

				if (ret_recv == SOCKET_ERROR)
				{
					LOG(My_level::error) << "������Ϣʧ��" << WSAGetLastError();
					Sleep(1000);
					continue;
				}
				else
				{
					buffer[ret_recv] = '\0'; //ȷ���ַ�������
					//���ط���
					std::string back_buff; //���ͽ��
					bool back_mode = _dir.get_change(buffer, back_buff);

					LOG(My_level::info) << "[" << client_ip << ":" << client_port << "]@ " << buffer;


					if(!back_mode)
					{
						LOG(My_level::error) << "ת��ʧ�ܣ�������û���ҵ���Ӧ�ļ�ֵ��";
						back_buff = "������";
					}
					while (1)
					{
						auto ret_send = sendto(_server_socket, back_buff.c_str(), back_buff.size(), 0,
							reinterpret_cast<struct sockaddr*>(&_client_addr), _client_addr_len);
						if (ret_send > 0)
						{
							LOG(My_level::info) << "���ͳɹ�����������Ϊ��" << back_buff;
							break;
						}
						Sleep(1000); //�������ʧ�ܣ��ȴ�1�������
					}
				}
			}
		}

		//��������
		~my_server()
		{
			closesocket(_server_socket);
			WSACleanup();
			LOG(My_level::info) << "���������";
		}
	private:
		my_server(int port = 8080)
			:_port(port)
		{
			//��������˿ڣ���Ϊ����˽�����Ϣ�ǲ���Ҫ������Ҫ��ip������ֻ��Ҫ�˿ں�
		}

		//�˿ں�
		int _port;
		//�׽���
		SOCKET _server_socket;
		//�洢was��ʼ����Ϣ
		WSADATA _wsa_data;
		//�洢��������Ϣ��Ŀ���������Ϣ
		struct sockaddr_in _server_addr, //�Լ�����Ϣ
						   _client_addr;  //�����տͻ�����Ϣ�Ժ�洢�����׼�����ʹ�������Ϣ���ͻ���
		//���ش洢�������Ϣ����
		int _server_addr_len;
		//���ش洢�ͻ�����Ϣ����
		int _client_addr_len;
		//�����ֵ�
		sui::my_direction _dir;

		//����ָ�룬���ع��캯���������û�����
		static my_server* my_ser;
	};

	my_server* my_server::my_ser = nullptr;
}