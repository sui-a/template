#pragma once
#pragma comment(lib, "ws2_32.lib")

#include <iostream>
#include <string>
#include <WS2tcpip.h>
#include "log.hpp"

namespace sui
{

	//�������˹�����
	class my_sock_server
	{
	public:
		my_sock_server()
		{
			//��ʼ��winsock
			if (WSAStartup(MAKEWORD(2, 2), &was_data))
			{
				//����ֵ��Ϊ0�� ��ʼ��ʧ��
				LOG(My_level::error) << "�����winsock��ʼ��ʧ��";
				exit(1);
			}

			if ((s = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET)
			{
				//ʹ��plv4,udp
				LOG(My_level::error) << "�����׽���ʧ��";
				exit(1);
			}

			//������ܰ󶨵ĵ�ַ�Ͷ˿�
			memset(&server, 0, sizeof(server));
			server.sin_family = AF_INET;
			server.sin_addr.s_addr = INADDR_ANY;
			server.sin_port = htons(8888);

			if (bind(s, (struct sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
			{
				//�����bind��ƽʱ��ͬ������һ������Ϊsocketʱ�����׽���������˵�����
				//����ʧ�ܽ���
				LOG(My_level::error) << "�׽���������˿ڹ���ʧ��";
				exit(1);
			}

			//��ȡ�ͻ��˵�ַ����
			client_len = sizeof(struct sockaddr_in);

			////����Ϊ����ģʽ��ָ��������
			//listen(s, 3);  //ʹ��udpЭ���������ģʽ
		}

		//����
		int to_send(std::string& dat)
		{
			//���з���
			int ret = sendto(s, dat.c_str(), dat.size(), 0, (struct sockaddr*)&client, client_len);
			if (ret == SOCKET_ERROR)
			{
				LOG(My_level::error) << "����ʧ��";
				return -1;
			}
			return 0;
		}



		//����
		int receive(std::string& data)
		{
			//�������տͻ�����Ϣ
			//���տͻ��˴洢λ��
			char my_message[1024]; //���1024�ֽ�
			//��ȡ��Ϣ������ȡ�ͻ��˵�ַ��Ϣ
			auto ret = recvfrom(s, my_message, sizeof(my_message), 0, (struct sockaddr*)&client, &client_len);
			if (ret == SOCKET_ERROR)
			{
				//����ʧ�ܣ�����1
				LOG(sui::My_level::error) << "���տͻ�����Ϣʧ��";
				return 1;
			}
			else if (ret == 0)
			{
				//Ŀǰδ���ó�ʱ
				LOG(My_level::info) << "�ȴ���ʱ��δ��ȡ������";
				return 2;
			}
			else
			{
				//���ճɹ�������0
				my_message[ret] = '\0'; //����ַ���������
				data = my_message; //�����յ�����Ϣ�洢��data��
				LOG(My_level::info) << "���տͻ�����Ϣ�ɹ�: " << data;
				return 0;
			}
		}

		~my_sock_server()
		{
			closesocket(s);
			WSACleanup();
			LOG(My_level::info) << "������׽��ֹر�";
		}

	private:
		//�洢winsock��ʼ����Ϣ
		WSADATA was_data;
		//�����ͻ���ͨѶ�ͷ���������׽���
		SOCKET  s;
		//�洢��������ַ��Ϣ
		struct sockaddr_in server, client;
		//���Σ��洢�ͻ��˵�ַ����
		int client_len;

	};
}

