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
            // 初始化 Winsock
            if (WSAStartup(MAKEWORD(2, 2), &_was) != 0)
            {
                LOG(My_level::error) << "WSAStartup 初始化失败";
                exit(1);
            }

            // 创建 UDP socket
            _my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (_my_socket == INVALID_SOCKET)
            {
                LOG(My_level::error) << "socket 创建失败: " << WSAGetLastError();
                WSACleanup();
                exit(1);
            }

            // 设置本地绑定（可选，让系统自动分配端口）
            struct sockaddr_in local_addr {};
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = INADDR_ANY;
            local_addr.sin_port = htons(0); // 0 表示系统自动选端口
            if (bind(_my_socket, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) == SOCKET_ERROR)
            {
                LOG(My_level::error) << "本地 bind 失败: " << WSAGetLastError();
                closesocket(_my_socket);
                WSACleanup();
                exit(1);
            }

            // 设置接收超时
            DWORD timeout = timeout_ms;
            if (setsockopt(_my_socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) == SOCKET_ERROR)
            {
                LOG(My_level::warning) << "设置接收超时失败: " << WSAGetLastError();
                // 不影响继续使用，只是超时可能不会生效
            }

            // 设置服务器地址
            memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(8888);
            inet_pton(AF_INET, "10.3.249.108", &server_addr.sin_addr);

            _server_len = sizeof(server_addr);
        }

        ~my_client()
        {
            closesocket(_my_socket);
            WSACleanup();
        }

        // 发送消息
        int send(const std::string& data)
        {
            // 发送数据
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
                LOG(My_level::error) << "sendto 失败: " << WSAGetLastError();
                return -1;
            }
            return 0;
        }

        int receive(std::string& recv_out)
        {

            // 接收回复
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
                    LOG(My_level::info) << "recvfrom 超时，无数据返回";
                    return 1; // 超时，无返回
                }
                else
                {
                    LOG(My_level::error) << "recvfrom 失败: " << err;
                    return -2; // 其他错误
                }
            }

            buffer[ret] = '\0';
            recv_out = buffer;
            LOG(My_level::info) << "接收到服务器回复: " << recv_out;
            return 0; // 接收成功
        }

    private:
        WSADATA _was;
        SOCKET _my_socket;
        struct sockaddr_in server_addr;
        int _server_len;
    };
}
