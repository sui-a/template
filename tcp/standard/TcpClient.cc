#include "Socket.hpp"
#include "Common.hpp"
#include <iostream>
#include <string>
#include <memory>

using namespace SocketModule;

void Usage(std::string proc)
{
    std::cerr << "Usage: " << proc << " server_ip server_port" << std::endl;
}

// ./tcpclient server_ip server_port
int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        Usage(argv[0]);
        exit(USAGE_ERR);
    }
    std::string server_ip = argv[1];
    uint16_t server_port = std::stoi(argv[2]);
    std::unique_ptr<Socket> client = std::make_unique<TcpSocket>();
    client->BuildTcpClientSocketMethod();

    if (client->Connect(server_ip, server_port) == 0)
    {
        //³É¹¦

    }
}