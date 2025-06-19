#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include "log.hpp"

int main()
{
    int sock_client = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_client == -1)
    {
        LOG(sui::LogLevel::ERROR) << "fail socket to create: " << errno;
        exit(1);
    }

    LOG(sui::LogLevel::INFO) << "socket create success";

    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8888);
    inet_pton(AF_INET, "192.168.31.108", &server_addr.sin_addr); //set ip addr

    if (connect(sock_client, (sockaddr*)&server_addr, sizeof(server_addr)) == -1)
    {
        LOG(sui::LogLevel::ERROR) << "fail to bind between sock_client and local_addr: " << errno;
        close(sock_client);
        exit(1);
    }
    LOG(sui::LogLevel::INFO) << "socket link success";
    while (1)
    {
        std::cout << "[Please input@]: ";
        std::string cli_inp;
        std::getline(std::cin, cli_inp);
        if (send(sock_client, cli_inp.c_str(), cli_inp.size(), 0) == -1)
        {
            LOG(sui::LogLevel::ERROR) << "fail to send: " << errno;
            break;
        }
        LOG(sui::LogLevel::INFO) << "socket link success";

        char buf[1024]{};
        ssize_t len = recv(sock_client, buf, sizeof(buf) - 1, 0);
        if (len < 0)
        {
            LOG(sui::LogLevel::ERROR) << "fail to recv" << errno;
            break;
        }
        buf[len] = '\0';
        LOG(sui::LogLevel::INFO) << "success to recv and data: " << buf;
    }
    close(sock_client);
    return 0;
}