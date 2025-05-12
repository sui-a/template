#ifndef __PROCESS_POOL_HPP__
#define __PROCESS_POOL_HPP__

#include <iostream>
#include <cstdlib> // stdlib.h stdio.h -> cstdlib cstdio
#include <vector>
#include <unistd.h>
#include <sys/wait.h>
#include "Task.hpp"

// ������
class Channel
{
public:
    Channel(int fd, pid_t id) : _wfd(fd), _subid(id)
    {
        _name = "channel-" + std::to_string(_wfd) + "-" + std::to_string(_subid);
    }
    ~Channel()
    {
    }
    void Send(int code)
    {
        int n = write(_wfd, &code, sizeof(code));
        (void)n; // ?
    }
    void Close()
    {
        close(_wfd);
    }
    void Wait()
    {
        pid_t rid = waitpid(_subid, nullptr, 0);
        (void)rid;
    }
    int Fd() { return _wfd; }
    pid_t SubId() { return _subid; }
    std::string Name() { return _name; }

private:
    int _wfd;
    pid_t _subid;
    std::string _name;
    // int _loadnum;
};

// ����֯
class ChannelManager
{
public:
    ChannelManager() : _next(0)
    {
    }
    void Insert(int wfd, pid_t subid)
    {
        _channels.emplace_back(wfd, subid);
        // Channel c(wfd, subid);
        // _channels.push_back(std::move(c));
    }
    Channel& Select()
    {
        auto& c = _channels[_next];
        _next++;
        _next %= _channels.size();
        return c;
    }
    void PrintChannel()
    {
        for (auto& channel : _channels)
        {
            std::cout << channel.Name() << std::endl;
        }
    }
    void StopSubProcess()
    {
        for (auto& channel : _channels)
        {
            channel.Close();
            std::cout << "�ر�: " << channel.Name() << std::endl;
        }
    }
    void WaitSubProcess()
    {
        for (auto& channel : _channels)
        {
            channel.Wait();
            std::cout << "����: " << channel.Name() << std::endl;
        }
    }
    ~ChannelManager() {}

private:
    std::vector<Channel> _channels;
    int _next;
};

const int gdefaultnum = 5;

class ProcessPool
{
public:
    ProcessPool(int num) : _process_num(num)
    {
        _tm.Register(PrintLog);
        _tm.Register(Download);
        _tm.Register(Upload);
    }
    void Work(int rfd)
    {
        while (true)
        {
            int code = 0;
            ssize_t n = read(rfd, &code, sizeof(code));
            if (n > 0)
            {
                if (n != sizeof(code))
                {
                    continue;
                }
                std::cout << "�ӽ���[" << getpid() << "]�յ�һ��������: " << code << std::endl;
                _tm.Execute(code);
            }
            else if (n == 0)
            {
                std::cout << "�ӽ����˳�" << std::endl;
                break;
            }
            else
            {
                std::cout << "��ȡ����" << std::endl;
                break;
            }
        }
    }
    bool Start()
    {
        for (int i = 0; i < _process_num; i++)
        {
            // 1. �����ܵ�
            int pipefd[2] = { 0 };
            int n = pipe(pipefd);
            if (n < 0)
                return false;

            // 2. �����ӽ���
            pid_t subid = fork();
            if (subid < 0)
                return false;
            else if (subid == 0)
            {
                // �ӽ���
                // 3. �رղ���Ҫ���ļ�������
                close(pipefd[1]);
                Work(pipefd[0]); //??
                close(pipefd[0]);
                exit(0);
            }
            else
            {
                // ������
                //  3. �رղ���Ҫ���ļ�������
                close(pipefd[0]); // д�ˣ�pipefd[1];
                _cm.Insert(pipefd[1], subid);
                // wfd, subid
            }
        }
        return true;
    }
    void Debug()
    {
        _cm.PrintChannel();
    }
    void Run()
    {
        // 1. ѡ��һ������
        int taskcode = _tm.Code();

        // 2. ѡ��һ���ŵ�[�ӽ���],���ؾ����ѡ��һ���ӽ��̣��������
        auto& c = _cm.Select();
        std::cout << "ѡ����һ���ӽ���: " << c.Name() << std::endl;
        // 2. ��������
        c.Send(taskcode);
        std::cout << "������һ��������: " << taskcode << std::endl;
    }
    void Stop()
    {
        // �رո��������е�wfd����
        _cm.StopSubProcess();
        // ���������ӽ���
        _cm.WaitSubProcess();
    }
    ~ProcessPool()
    {
    }

private:
    ChannelManager _cm;
    int _process_num;
    TaskManager _tm;
};

#endif