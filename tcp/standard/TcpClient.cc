#include "Socket.hpp"
#include "Common.hpp"
#include "Protocol.hpp"
#include <iostream>
#include <string>
#include <memory>

using namespace SocketModule;

void Usage(std::string proc)
{
    std::cerr << "Usage: " << proc << " server_ip server_port" << std::endl;
}

void GetDataFromStdin(int* x, int* y, char* oper)
{
    std::cout << "Please Enter x: ";
    std::cin >> *x;
    std::cout << "Please Enter y: ";
    std::cin >> *y;
    std::cout << "Please Enter oper: ";
    std::cin >> *oper;
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
    std::shared_ptr<Socket> client = std::make_shared<TcpSocket>();
    client->BuildTcpClientSocketMethod();

    if (client->Connect(server_ip, server_port) != 0)
    {
        // ʧ��
        std::cerr << "connect error" << std::endl;
        exit(CONNECT_ERR);
    }

    std::unique_ptr<Protocol> protocol = std::make_unique<Protocol>();
    std::string resp_buffer;
    // ���ӷ������ɹ�
    while (true)
    {
        // 1. �ӱ�׼���뵱�л�ȡ����
        int x, y;
        char oper;
        GetDataFromStdin(&x, &y, &oper);

        // 2. ����һ������-> ����ֱ�ӷ��͵��ַ���
        std::string req_str = protocol->BuildRequestString(x, y, oper);

        // std::cout << "-----------encode req string-------------" << std::endl;
        // std::cout << req_str << std::endl;
        // std::cout << "------------------------------------------" << std::endl;

        // 3. ��������
        client->Send(req_str);

        // 4. ��ȡӦ��
        Response resp;
        bool res = protocol->GetResponse(client, resp_buffer, &resp);
        if (res == false)
            break;

        // 5. ��ʾ���
        resp.ShowResult();
    }
    client->Close();
    return 0;
}
