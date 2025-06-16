#include <iostream>
#include <memory>
#include "Route.hpp"
#include "UdpServer.hpp" // ����ͨ�ŵĹ���
#include "ThreadPool.hpp"

using namespace ThreadPoolModule;

using task_t = std::function<void()>;

// ./udpserver port
int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " port" << std::endl;
        return 1;
    }
    // std::string ip = argv[1];
    uint16_t port = std::stoi(argv[1]);
    Enable_Console_Log_Strategy();

    // 1. ·�ɷ���
    Route r;

    // 2. �̳߳�
    auto tp = ThreadPool<task_t>::GetInstance();

    // 3. ��������������ṩͨ�Ź���
    std::unique_ptr<UdpServer> usvr = std::make_unique<UdpServer>(port, [&r, &tp](int sockfd, const std::string& message, InetAddr& peer) {
        task_t t = std::bind(&Route::MessageRoute, &r, sockfd, message, peer);
        tp->Enqueue(t);
        });

    usvr->Init();
    usvr->Start();

    return 0;
}

