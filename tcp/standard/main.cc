#include "NetCal.hpp"
#include "Protocol.hpp"
#include "TcpServer.hpp"
#include "Daemon.hpp"
#include <memory>

void Usage(std::string proc)
{
    std::cerr << "Usage: " << proc << " port" << std::endl;
}

// ./tcpserver 8080
int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        Usage(argv[0]);
        exit(USAGE_ERR);
    }
    std::cout << "�������Ѿ��������Ѿ���һ���ػ�������" << std::endl;

    Daemon(0, 0);
    // daemon(1, 1);

    // Enable_Console_Log_Strategy();
    Enable_File_Log_Strategy();


    // 1. ����
    std::unique_ptr<Cal> cal = std::make_unique<Cal>();

    // 2. Э���
    std::unique_ptr<Protocol> protocol = std::make_unique<Protocol>([&cal](Request& req)->Response {
        return cal->Execute(req);
        });

    // 3. ��������
    std::unique_ptr<TcpServer> tsvr = std::make_unique<TcpServer>(std::stoi(argv[1]),
        [&protocol](std::shared_ptr<Socket>& sock, InetAddr& client) {
            protocol->GetRequest(sock, client);
        });

    tsvr->Start();
    // sleep(5);

    return 0;
}
