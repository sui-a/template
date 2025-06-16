#pragma once
#pragma comment(lib, "Ws2_32.lib")
#include <WS2tcpip.h>
#include "log.hpp"

namespace sui
{
    class my_client
    {
    public:
        my_client(uint16_t timeout_ms = 2000)
        {
            // ��ʼ�� Winsock
            if (WSAStartup(MAKEWORD(2, 2), &_was) != 0)
            {
                LOG(My_level::error) << "WSAStartup ��ʼ��ʧ��";
                exit(1);
            }

            // �����׽���
            _my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (_my_socket == INVALID_SOCKET)
            {
                LOG(My_level::error) << "socket ����ʧ��: " << WSAGetLastError();
                WSACleanup();
                exit(1);
            }

            // ���ñ��ذ�
            struct sockaddr_in local_addr {};
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = INADDR_ANY;
            local_addr.sin_port = htons(0); // 0 ��ʾϵͳ�Զ�ѡ�˿� 
            //�������ط���˿ں͵�ַ����������Ϣʱʹ�õĵ�ַ�Ͷ˿ڡ�ע����������Ϣʱ���Բ���ʾ�󶨣�����ϵͳ�Զ�������ip��ͬ���˿ں��������
            if (bind(_my_socket, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) == SOCKET_ERROR)
            {
                LOG(My_level::error) << "���� bind ʧ��: " << WSAGetLastError();
                closesocket(_my_socket);
                WSACleanup();
                exit(1);
            }
            LOG(My_level::info) << local_addr.sin_port;
            // ���ý��ճ�ʱ
            DWORD timeout = timeout_ms;
            if (setsockopt(_my_socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) == SOCKET_ERROR)
            {
                LOG(My_level::warning) << "���ý��ճ�ʱʧ��: ";
            } 

            // ���÷�������ַ
            memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(8888);
            inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);

            _server_len = sizeof(server_addr);
        }

        ~my_client()
        {
            closesocket(_my_socket);
            WSACleanup();
        }

        // ������Ϣ
        int send(const std::string& data)
        {
            // ��������
            int sent = sendto(
                _my_socket,
                data.c_str(),
                static_cast<int>(data.size()),
                0,
                reinterpret_cast<sockaddr*>(&server_addr),
                _server_len
            );

            if (sent == SOCKET_ERROR)
            {
                LOG(My_level::error) << "����ʧ��: ";
                return -1;
            }
            //�ɹ�����0
            return 0;
        }

        int receive(std::string& recv_out)
        {

            // ���ջظ�
            char buffer[1024];
            struct sockaddr_in from_addr {};
            int from_len = sizeof(from_addr);

            int ret = recvfrom(
                _my_socket,
                buffer,
                sizeof(buffer) - 1,
                0,
                reinterpret_cast<sockaddr*>(&from_addr),
                &from_len
            );

            if (ret == SOCKET_ERROR)
            {
                int err = WSAGetLastError();
                if (err == WSAETIMEDOUT)
                {
                    LOG(My_level::info) << "��ȡ��ʱ�������ݷ���" << err;
                    return 1; // ��ʱ���޷���
                }
                else
                {
                    LOG(My_level::error) << "����ʧ��: " << err;
                    return -2; // ��������
                }
            }

            buffer[ret] = '\0';
            recv_out = buffer;
            return 0; // ���ճɹ�
        }

    private:
        WSADATA _was;
        SOCKET _my_socket;
        struct sockaddr_in server_addr;
        int _server_len;
    };
}
