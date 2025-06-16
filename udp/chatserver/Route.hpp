#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "InetAddr.hpp"
#include "Log.hpp"
#include "Mutex.hpp"

using namespace LogModule;
using namespace MutexModule;

class Route
{
private:
    bool IsExist(InetAddr& peer)
    {
        for (auto& user : _online_user)
        {
            if (user == peer)
            {
                return true;
            }
        }
        return false;
    }
    void AddUser(InetAddr& peer)
    {
        LOG(LogLevel::INFO) << "����һ�������û�: " << peer.StringAddr();
        _online_user.push_back(peer);
    }

    void DeleteUser(InetAddr& peer)
    {
        for (auto iter = _online_user.begin(); iter != _online_user.end(); iter++)
        {
            if (*iter == peer)
            {
                LOG(LogLevel::INFO) << "ɾ��һ�������û�:" << peer.StringAddr() << "�ɹ�";
                _online_user.erase(iter);
                break;
            }
        }
    }

public:
    Route()
    {
    }
    void MessageRoute(int sockfd, const std::string& message, InetAddr& peer)
    {
        LockGuard lockguard(_mutex);

        if (!IsExist(peer))
        {
            AddUser(peer);
        }

        std::string send_message = peer.StringAddr() + "# " + message; // 127.0.0.1:8080# ���

        // TODO
        for (auto& user : _online_user)
        {
            sendto(sockfd, send_message.c_str(), send_message.size(), 0, (const struct sockaddr*)&(user.NetAddr()), sizeof(user.NetAddr()));
        }

        // ����û�һ���Ѿ�������
        if (message == "QUIT")
        {
            LOG(LogLevel::INFO) << "ɾ��һ�������û�: " << peer.StringAddr();
            DeleteUser(peer);
        }
    }
    ~Route()
    {
    }

private:
    // �״θ��ҷ���Ϣ����ͬ�ڵ�¼
    std::vector<InetAddr> _online_user; // �����û�
    Mutex _mutex;
};